#pragma once

#include <limits>
#include <memory>
#include <shared_mutex>
#include <string>
#include <type_traits>
#include <utility>

#include <Eigen/Geometry>

#include <costmap_2d/costmap_2d.hpp>
#include <costmap_server/costmap_wrapper.hpp>

#include "ayanami/abstract/components/robot_state_controller.hpp"
#include "ayanami/map/abstract_map_context.hpp"
#include "ayanami/costmap/core/costmap_command.hpp"
#include "ayanami/costmap/core/costmap_planning_view.hpp"
#include "ayanami/costmap/core/costmap_view.hpp"
#include "ayanami/messages/geometry_msgs.hpp"

namespace ayanami {
namespace costmap {

/**
 * @brief The costmap family's "map module".
 *
 * Aggregates the global/local CostmapWrapper pair (global:
 * StaticLayer+InflationLayer; local: rolling
 * SimpleObstacleLayer+InflationLayer).
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
 * - Fields are private; only narrow accessors are exposed.
 */
class CostmapMapContext : public map::AbstractMapContext,
                       public std::enable_shared_from_this<CostmapMapContext> {
 public:
  using Ptr = std::shared_ptr<CostmapMapContext>;
  using ConstPtr = std::shared_ptr<const CostmapMapContext>;

  CostmapMapContext() = default;

  std::string mapFamily() const override { return "costmap"; }
  std::string globalFrame() const override { return global_frame; }
  std::string localFrame() const override { return local_frame; }
  std::string robotFrame() const override { return robot_frame; }
  bool isCurrent() const override { return mapsAreCurrent(); }
  std::uint32_t capabilities() const override {
    using Capability = map::MapCapability;
    return static_cast<std::uint32_t>(Capability::CollisionQuery) |
           static_cast<std::uint32_t>(Capability::DynamicObstacleUpdate) |
           static_cast<std::uint32_t>(Capability::ClearRegion);
  }
  map::MapCommandPort::Ptr commandPort() const override { return command_port; }

  /// @brief Path validity (semantics aligned with nav2
  /// planner_server::isPathValid):
  /// checking starts from the path point nearest the robot -- segments
  /// already traversed may be occupied by new obstacles and are a source of
  /// false negatives, so they are skipped explicitly; every point must be
  /// in bounds (out of bounds treated as lethal, mirroring nav2) and its
  /// cost must be neither NO_INFORMATION nor lethal (isLethal >= 253
  /// includes inscribed, mirroring nav2's use_radius mode). Zero-copy
  /// validation under a shared lock; degrades to a non-empty check when no
  /// global wrapper exists; degrades to a full-path check (conservative)
  /// when the robot pose is unavailable.
  bool isPathValid(const ayanami::nav_msgs::Path& path) const override {
    if (path.poses.empty()) {
      return false;
    }
    if (!global_costmap) {
      return true;
    }
 // Start from the nearest point (pose resolution outside the map lock;
    // robot state single facade = RobotStateController)
    std::size_t start_index = 0;
    ayanami::geometry_msgs::PoseStamped robot_pose;
    if (robotPoseInFrame(robot_pose, global_frame)) {
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
    return readGlobal([&path, start_index](const costmap_2d::Costmap2D& grid) {
      for (std::size_t i = start_index; i < path.poses.size(); ++i) {
        unsigned int mx = 0;
        unsigned int my = 0;
        if (!grid.worldToMap(path.poses[i].pose.position.x(),
                             path.poses[i].pose.position.y(), mx, my)) {
          return false;  // out of bounds: treated as lethal, mirroring nav2
        }
        const auto cost = grid.getCost(mx, my);
        if (cost == costmap_2d::NO_INFORMATION || costmap_2d::isLethal(cost)) {
          return false;
        }
      }
      return true;
    });
  }

  // ---- Configuration (writer / test injection) ----
  void setGlobalCostmap(costmap_2d::CostmapWrapper::Ptr wrapper) {
    global_costmap = std::move(wrapper);
  }
  void setLocalCostmap(costmap_2d::CostmapWrapper::Ptr wrapper) {
    local_costmap = std::move(wrapper);
  }
  void setFrames(std::string global_frame, std::string local_frame,
                 std::string robot_frame) {
    this->global_frame = std::move(global_frame);
    this->local_frame = std::move(local_frame);
    this->robot_frame = std::move(robot_frame);
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
  const std::string& getGlobalFrame() const { return global_frame; }
  const std::string& getRobotFrame() const { return robot_frame; }

  /// @brief False if any wrapper is not current; no wrapper counts as current.
  bool mapsAreCurrent() const {
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
  CostmapView lockGlobal() const { return buildView(global_costmap, global_frame); }

  /// @brief Local read view under a shared lock (uses the local frame).
  CostmapView lockLocal() const { return buildView(local_costmap, local_frame); }

  /// @brief Planning overlay view (family read contract: planner = global +
  /// local superposition). Holds both shared locks at once (acquisition
  /// order local -> global; the writer takes them sequentially without
  /// nesting, so no lock-ordering cycle). Zero copy. The local<-global
  /// transform resolves once via RobotStateController; on a transient TF
  /// miss the view degrades to global-only.
  CostmapPlanningView lockPlanning() const {
    CostmapView local = buildView(local_costmap, local_frame);
    CostmapView global = buildView(global_costmap, global_frame);
    Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
    bool ready = false;
    if (local.valid() && global.valid()) {
      ready = components::RobotStateController::instance().lookupTransform(
          local_frame, global_frame, ayanami::Timestamp::latest(), t);
    }
    return CostmapPlanningView(std::move(global), std::move(local), t, ready);
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
  /// @brief Resolve the robot pose in the requested frame via
  /// RobotStateController (single robot-state query facade; lock-free).
  bool robotPoseInFrame(ayanami::geometry_msgs::PoseStamped& pose,
                        const std::string& frame) const {
    Eigen::Isometry3d T;
    if (!components::RobotStateController::instance().lookupTransform(
            frame, robot_frame, ayanami::Timestamp::latest(), T)) {
      return false;
    }
    pose = geometry_msgs::PoseStamped::fromIsometry(frame, T);
    return true;
  }

  /// @brief Build a read view under a shared lock: grid/frame come from the
  /// wrapper; the footprint comes from the immutable configure-time cache;
  /// robot_pose is resolved in the view's own frame (local view =
  /// local_frame, global view = global_frame) via the lock-free
  /// RobotStateController.
  CostmapView buildView(
      const costmap_2d::CostmapWrapper::Ptr& wrapper,
      const std::string& frame) const {
    if (!wrapper) {
      return CostmapView{};
    }
    std::shared_lock<std::shared_mutex> lock(wrapper->mutex());
    const auto* grid = wrapper->get().getCostmap();
    ayanami::geometry_msgs::PoseStamped robot_pose;
    const bool has_robot_pose = robotPoseInFrame(robot_pose, frame);
    return CostmapView(std::move(lock), grid, frame, &cached_footprint,
                       std::move(robot_pose), has_robot_pose,
                       wrapper->isCurrent());
  }

  costmap_2d::CostmapWrapper::Ptr global_costmap;
  costmap_2d::CostmapWrapper::Ptr local_costmap;
  std::string global_frame{"map"};
  std::string local_frame{"odom"};
  std::string robot_frame{"base_link"};
  costmap_2d::Footprint cached_footprint;
  map::MapCommandPort::Ptr command_port;
};

}  // namespace costmap
}  // namespace ayanami
