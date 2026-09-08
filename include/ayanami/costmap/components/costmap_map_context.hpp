#pragma once

#include <functional>
#include <limits>
#include <memory>
#include <shared_mutex>
#include <string>
#include <type_traits>
#include <utility>

#include <Eigen/Geometry>

#include <costmap_2d/costmap_2d.hpp>
#include <costmap_server/costmap_wrapper.hpp>

#include "ayanami/map/abstract_map_context.hpp"
#include "ayanami/costmap/core/costmap_command.hpp"
#include "ayanami/messages/nav_msgs.hpp"
#include "ayanami/costmap/core/costmap_planning_view.hpp"
#include "ayanami/costmap/core/costmap_view.hpp"
#include "ayanami/messages/geometry_msgs.hpp"
#include "ayanami/messages/timestamp.hpp"

namespace ayanami {
namespace costmap {

/**
 * @brief The costmap family's "map module".
 *
 * Both the global and the local costmap live in one shared world frame
 * (map_frame, typically "map"): obstacle observations are stamped in that
 * frame, so no cross-frame transform is needed when overlaying the two
 * grids. The local map may still be a rolling window (its origin follows
 * the robot), but its cell coordinates are map-frame coordinates.
 *
 * Ownership and holding discipline:
 * - Ownership: the host (CostmapNavigation / CostmapNavigationServer) holds
 *   CostmapMapContext::Ptr; family plugins (planner/smoother/controller/
 *   recovery) hold a ConstPtr (const smart pointer) on demand via
 *   initialize(name, ConstPtr) and never modify the map.
 * - Map reads: plugins take a CostmapView via lockGlobal()/lockLocal()
 *   (RAII shared lock + zero-copy direct master grid reads) and hold the
 *   lock for a single algorithm call (<10ms); the map writer waits for the
 *   lock to write.
 * - Mutable wrapper/layer access (globalCostmap()/localCostmap()) is
 *   provided only to the map writer (CostmapNavigationServer) and the
 *   execution lifecycle (checkActivate/checkDeactivate/isCurrent); rule
 *   plugins never use it.
 * - Robot pose lookups go through the injected transformLookup function
 *   (wired by the host at configureCoreMaps time); the context holds no
 *   reference to any global robot-state singleton.
 *
 * Family read contract (both grids share one frame):
 * - planner: reads the overlay (superposition) of the global and local map
 *   content (lockPlanning / CostmapPlanningView);
 * - smoother: reads only the local map content;
 * - controller/recovery: read the local map (family-defined).
 */
class CostmapMapContext : public map::AbstractMapContext,
                       public std::enable_shared_from_this<CostmapMapContext> {
 public:
  using Ptr = std::shared_ptr<CostmapMapContext>;
  using ConstPtr = std::shared_ptr<const CostmapMapContext>;

  /// @brief Minimal transform-query seam injected by the host (typically a
  /// thin wrapper over the TransformStore-backed robot state). Returns
  /// false when the transform is unavailable.
  using TransformLookup = std::function<bool(
      const std::string& target_frame, const std::string& source_frame,
      ayanami::Timestamp stamp, Eigen::Isometry3d& transform)>;

  CostmapMapContext() = default;

  std::string mapFamily() const override { return "costmap"; }
  map::MapCommandPort::Ptr commandPort() const override { return command_port; }

  /// @brief Path validity check on the same overlay the planner reads
  /// (global+local superposition, semantics aligned with nav2
  /// planner_server::isPathValid): checking starts from the path point
  /// nearest the robot -- segments already traversed may be occupied by new
  /// obstacles and are a source of false negatives, so they are skipped
  /// explicitly; every point must be in bounds (out of bounds treated as
  /// lethal, mirroring nav2) and its cost must be neither NO_INFORMATION nor
  /// lethal (isLethal >= 253 includes inscribed, mirroring nav2's
  /// use_radius mode). Zero-copy validation under shared locks; degrades to
  /// a non-empty check when no global wrapper exists; degrades to a
  /// full-path check (conservative) when the robot pose is unavailable.
  bool isPathValid(const ayanami::nav_msgs::Path& path) const {
    if (path.poses.empty()) {
      return false;
    }
    if (!global_costmap) {
      return true;
    }
    // Start from the nearest point (pose resolution outside the map lock).
    std::size_t start_index = 0;
    ayanami::geometry_msgs::PoseStamped robot_pose;
    if (robotPoseInFrame(robot_pose, map_frame)) {
      double best = std::numeric_limits<double>::max();
      for (std::size_t i = 0; i < path.poses.size(); ++i) {
        const double d = (path.poses[i].pose.position -
                          robot_pose.pose.position)
                             .norm();
        if (d < best) {
          best = d;
          start_index = i;
        }
      }
    }
    auto overlay = lockPlanning();
    if (!overlay.valid()) {
      return true;
    }
    const auto* grid = overlay.globalGrid();
    for (std::size_t i = start_index; i < path.poses.size(); ++i) {
      unsigned int mx = 0;
      unsigned int my = 0;
      if (!grid->worldToMap(path.poses[i].pose.position.x(),
                            path.poses[i].pose.position.y(), mx, my)) {
        return false;  // out of bounds: treated as lethal, mirroring nav2
      }
      const auto cost = overlay.costAtGlobalCell(mx, my);
      if (cost == costmap_2d::NO_INFORMATION || costmap_2d::isLethal(cost)) {
        return false;
      }
    }
    return true;
  }

  // ---- Configuration (writer / test injection) ----
  void setGlobalCostmap(costmap_2d::CostmapWrapper::Ptr wrapper) {
    global_costmap = std::move(wrapper);
  }
  void setLocalCostmap(costmap_2d::CostmapWrapper::Ptr wrapper) {
    local_costmap = std::move(wrapper);
  }
  void setFrames(std::string map_frame, std::string robot_frame) {
    this->map_frame = std::move(map_frame);
    this->robot_frame = std::move(robot_frame);
  }
  void setTransformLookup(TransformLookup lookup) {
    transform_lookup = std::move(lookup);
  }

  /// @brief Cache the immutable footprint during setup (injected by
  /// configureCoreMaps). Readers take the footprint only from this cache
  /// and never read the live LayeredCostmap (reading mutable state outside
  /// the write lock).
  void setFootprint(costmap_2d::Footprint footprint) {
    cached_footprint = std::move(footprint);
  }

  /// @brief Inject the map write-command port (during configureCoreMaps,
  /// right after the sole writer is created).
  /// Map-family plugins such as recovery fetch it via commandPort instead
  /// of side-path injection.
  void setCommandPort(map::MapCommandPort::Ptr port) {
    command_port = std::move(port);
  }

  // ---- frames / freshness ----
  const std::string& getMapFrame() const { return map_frame; }
  const std::string& getRobotFrame() const { return robot_frame; }

  /// @brief False if any wrapper is not current; no wrapper counts as current.
  bool isCurrent() const {
    return (!global_costmap || global_costmap->isCurrent()) &&
           (!local_costmap || local_costmap->isCurrent());
  }

  /// @brief The immutable footprint cached at configure time.
  const costmap_2d::Footprint& footprint() const { return cached_footprint; }

  // ---- Writer/execution lifecycle accessors (mutable map touched by the writer only) ----
  costmap_2d::CostmapWrapper::Ptr globalCostmap() {
    return global_costmap;
  }
  std::shared_ptr<const costmap_2d::CostmapWrapper> globalCostmap() const {
    return global_costmap;
  }
  costmap_2d::CostmapWrapper::Ptr localCostmap() { return local_costmap; }
  std::shared_ptr<const costmap_2d::CostmapWrapper> localCostmap() const {
    return local_costmap;
  }

  /// @brief Return a const owner for external observers and map-family plugins.
  /// Map data must still be accessed through lockGlobal/lockLocal/readGlobal/
  /// readLocal so the underlying master grid cannot escape its lock scope.
  ConstPtr getSharedConstPtr() const {
    return shared_from_this();
  }

  // ---- Reader interface (shared lock + zero copy) ----

  /// @brief Global read view under a shared lock (held for the whole
  /// algorithm call; no wrapper -> invalid view).
  CostmapView lockGlobal() const { return buildView(global_costmap); }

  /// @brief Local read view under a shared lock (same map frame as global).
  CostmapView lockLocal() const { return buildView(local_costmap); }

  /// @brief Planning overlay view (family read contract: planner = global +
  /// local superposition). Holds both shared locks at once (acquisition
  /// order local -> global; the writer takes them sequentially without
  /// nesting, so no lock-ordering cycle). Zero copy; both grids share the
  /// map frame, so no cross-frame transform is involved.
  CostmapPlanningView lockPlanning() const {
    CostmapView local = buildView(local_costmap);
    CostmapView global = buildView(global_costmap);
    return CostmapPlanningView(std::move(global), std::move(local));
  }

  /// @brief Run a callback while the global master grid is protected
  /// (shared lock).
  template <typename Fn>
  auto readGlobal(Fn&& fn) const
      -> decltype(std::forward<Fn>(fn)(
          std::declval<const costmap_2d::Costmap2D&>())) {
    using Result = decltype(std::forward<Fn>(fn)(
        std::declval<const costmap_2d::Costmap2D&>()));
    if (!global_costmap) {
      if constexpr (std::is_void_v<Result>) return;
      else return Result{};
    }
    std::shared_lock<std::shared_mutex> lock(global_costmap->mutex());
    const auto* grid = global_costmap->get().getCostmap();
    if (!grid) {
      if constexpr (std::is_void_v<Result>) return;
      else return Result{};
    }
    return std::forward<Fn>(fn)(static_cast<const costmap_2d::Costmap2D&>(*grid));
  }

  /// @brief Run a callback while the local master grid is protected
  /// (shared lock).
  template <typename Fn>
  auto readLocal(Fn&& fn) const
      -> decltype(std::forward<Fn>(fn)(
          std::declval<const costmap_2d::Costmap2D&>())) {
    using Result = decltype(std::forward<Fn>(fn)(
        std::declval<const costmap_2d::Costmap2D&>()));
    if (!local_costmap) {
      if constexpr (std::is_void_v<Result>) return;
      else return Result{};
    }
    std::shared_lock<std::shared_mutex> lock(local_costmap->mutex());
    const auto* grid = local_costmap->get().getCostmap();
    if (!grid) {
      if constexpr (std::is_void_v<Result>) return;
      else return Result{};
    }
    return std::forward<Fn>(fn)(static_cast<const costmap_2d::Costmap2D&>(*grid));
  }

 private:
  /// @brief Resolve the robot pose in the requested frame via the injected
  /// transform lookup (host-wired; no global singleton dependency here).
  bool robotPoseInFrame(ayanami::geometry_msgs::PoseStamped& pose,
                        const std::string& frame) const {
    if (!transform_lookup) {
      return false;
    }
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    if (!transform_lookup(frame, robot_frame, ayanami::Timestamp::latest(),
                          T)) {
      return false;
    }
    pose = geometry_msgs::PoseStamped::fromIsometry(frame, T);
    return true;
  }

  /// @brief Build a read view under a shared lock: grid/frame come from the
  /// wrapper; the footprint comes from the immutable configure-time cache;
  /// robot_pose is resolved in the map frame via the injected lookup.
  CostmapView buildView(
      const costmap_2d::CostmapWrapper::Ptr& wrapper) const {
    if (!wrapper) {
      return CostmapView{};
    }
    std::shared_lock<std::shared_mutex> lock(wrapper->mutex());
    const auto* grid = wrapper->get().getCostmap();
    ayanami::geometry_msgs::PoseStamped robot_pose;
    const bool has_robot_pose = robotPoseInFrame(robot_pose, map_frame);
    return CostmapView(std::move(lock), grid, map_frame, &cached_footprint,
                       std::move(robot_pose), has_robot_pose,
                       wrapper->isCurrent());
  }

  costmap_2d::CostmapWrapper::Ptr global_costmap;
  costmap_2d::CostmapWrapper::Ptr local_costmap;
  std::string map_frame{"map"};
  std::string robot_frame{"base_link"};
  TransformLookup transform_lookup;
  costmap_2d::Footprint cached_footprint;
  map::MapCommandPort::Ptr command_port;
};

}  // namespace costmap
}  // namespace ayanami
