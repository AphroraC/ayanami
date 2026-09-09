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
#include "ayanami/costmap/core/planning_overlay.hpp"
#include "ayanami/costmap/core/costmap_view.hpp"
#include "ayanami/messages/nav_msgs.hpp"
#include "ayanami/messages/geometry_msgs.hpp"
#include "ayanami/messages/timestamp.hpp"

namespace ayanami {
namespace costmap {

/**
 * @brief The costmap family "map module" (two framed grids).
 *
 * The global and the local costmap live in DIFFERENT frames:
 * - global costmap frame = global_costmap_frame (typically "map"):
 *   static layer + long-lived obstacle marks;
 * - local costmap frame = local_costmap_frame (typically "odom"): rolling
 *   window of dynamic obstacle marks. Being in the odometry frame makes the
 *   local map immune to localization (AMCL) jumps: corrections move the
 *   map<-odom edge, not the odom<-sensor edges the local marks were stamped
 *   with, so short-term local geometry stays consistent (nav2 convention).
 *
 * Ownership and holding discipline:
 * - Ownership: the host (CostmapNavigation / CostmapNavigationServer) holds
 *   CostmapMapContext::Ptr; family plugins (planner/smoother/controller/
 *   recovery) hold a ConstPtr (const smart pointer) on demand via
 *   configure(context, map) and never modify the map.
 * - Map reads: plugins take a CostmapView via lockGlobal()/lockLocal()
 *   (RAII shared lock + zero-copy direct master grid reads) and hold the
 *   lock for the duration of one short algorithm call; the map writer waits
 *   for the lock to write. The planner is the exception: it materializes its
 *   input via composePlanningOverlay() (locks held only for the compose
 *   pass) and searches the returned buffer lock-free.
 * - Mutable wrapper/layer access (globalCostmap()/localCostmap()) is
 *   provided only to the map writer (CostmapNavigationServer) and the
 *   execution lifecycle (checkActivate/checkDeactivate/isCurrent); rule
 *   plugins never use it.
 * - All TF queries go through the RobotStateController global singleton
 *   (the framework has exactly one transform query surface; no per-module
 *   getRobotPose-style wrappers).
 *
 * Family read contract:
 * - planner: global+local overlay via composePlanningOverlay() (global-frame
 *   buffer; the local<-global transform is resolved exactly once per
 *   compose, never per searched node);
 * - smoother: reads only the local map (lockLocal(); paths expressed in the
 *   global frame are transformed into the local frame once per call);
 * - controller/recovery: read the local map (family-defined).
 */
class CostmapMapContext : public map::AbstractMapContext,
                       public std::enable_shared_from_this<CostmapMapContext> {
 public:
  using Ptr = std::shared_ptr<CostmapMapContext>;
  using ConstPtr = std::shared_ptr<const CostmapMapContext>;

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
  /// lethal (isLethal >= 253 includes inscribed, mirroring nav2 use_radius
  /// mode). The overlay (and therefore the locks) only lives for the compose
  /// pass; the point scan runs on the returned buffer. Degrades to a
  /// non-empty check when no global wrapper exists; degrades to a full-path
  /// check (conservative) when the robot pose is unavailable.
  bool isPathValid(const ayanami::nav_msgs::Path& path) const {
    if (path.poses.empty()) {
      return false;
    }
    if (!global_costmap) {
      return true;
    }
    PlanningOverlay overlay;
    if (!composePlanningOverlay(overlay)) {
      return true;
    }
    // Start from the nearest point (pose resolution outside the overlay
    // query; the pose was resolved at compose time).
    std::size_t start_index = 0;
    if (overlay.has_robot_pose) {
      double best = std::numeric_limits<double>::max();
      for (std::size_t i = 0; i < path.poses.size(); ++i) {
        const double d = (path.poses[i].pose.position -
                          overlay.robot_pose.pose.position)
                             .norm();
        if (d < best) {
          best = d;
          start_index = i;
        }
      }
    }
    for (std::size_t i = start_index; i < path.poses.size(); ++i) {
      unsigned int mx = 0;
      unsigned int my = 0;
      if (!overlay.worldToMap(path.poses[i].pose.position.x(),
                              path.poses[i].pose.position.y(), mx, my)) {
        return false;  // out of bounds: treated as lethal, mirroring nav2
      }
      const auto cost = overlay.costAt(mx, my);
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
  const std::string& getLocalFrame() const { return local_frame; }
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
  /// algorithm call; no wrapper -> invalid view). The grid and the resolved
  /// robot pose are both in the wrapper's own (global) frame.
  CostmapView lockGlobal() const { return buildView(global_costmap); }

  /// @brief Local read view under a shared lock (grid + robot pose in the
  /// wrapper's own (local) frame, typically odom).
  CostmapView lockLocal() const { return buildView(local_costmap); }

  /// @brief Materialize the planning input: global+local overlay composed
  /// into `overlay` while both shared locks are held (compose pass only,
  /// bounded), then all locks released -- the caller searches the buffer
  /// lock-free. The local<-global transform is resolved exactly ONCE per
  /// compose (RobotStateController, latest common time) and applied as a
  /// cached matrix during composition; the search never transforms.
  ///
  /// Overlay rule (conservative, per global-grid cell):
  /// cost = max(global_cost, local_cost) where the local value participates
  /// only when the point falls inside the local window; NO_INFORMATION never
  /// downgrades (unknown stays unknown -- local free space cannot clear
  /// global unknown space). When the local wrapper is missing or the
  /// local<-global transform is unavailable, the overlay degrades to
  /// global-only and `overlay.local_overlay_valid` is false.
  /// @return false when no global grid is readable (overlay invalidated).
  bool composePlanningOverlay(PlanningOverlay& overlay) const {
    overlay.invalidate();
    if (!global_costmap) {
      return false;
    }
    // Resolve the frame transform BEFORE taking any map lock (the writer
    // discipline: never nest the robot-state lock inside a map-lock wait).
    // Grid pointer identity is stable post-configure, so availability is
    // probed without holding the wrapper lock.
    // The wrapper's own frame is authoritative for its grid (context frames
    // are the configured defaults; the server always builds them consistent).
    const auto* local_probe =
        local_costmap ? local_costmap->get().getCostmap() : nullptr;
    const std::string local_grid_frame =
        local_costmap ? local_costmap->getGlobalFrame() : std::string{};
    Eigen::Isometry3d local_from_global = Eigen::Isometry3d::Identity();
    bool has_local = false;
    if (local_probe && local_grid_frame == global_frame) {
      has_local = true;  // same frame: identity transform, no lookup needed
    } else if (local_probe &&
               ayanami::components::RobotStateController::instance()
                   .lookupTransform(local_grid_frame, global_frame,
                                    ayanami::Timestamp::latest(),
                                    local_from_global, 0.0)) {
      has_local = true;
    }
    // Acquisition order local -> global (the writer stamps local first, then
    // global, taking the two locks sequentially without nesting, so no
    // lock-ordering cycle can form).
    CostmapView local = buildView(local_costmap);
    CostmapView global = buildView(global_costmap);
    if (!global.valid()) {
      return false;
    }
    const auto* global_grid = global.grid();
    const auto* local_grid = local.grid();
    has_local = has_local && local_grid != nullptr;

    const unsigned int size_x = global_grid->getSizeInCellsX();
    const unsigned int size_y = global_grid->getSizeInCellsY();
    overlay.frame = global_frame;
    overlay.size_x = size_x;
    overlay.size_y = size_y;
    overlay.resolution = global_grid->getResolution();
    overlay.origin_x = global_grid->getOriginX();
    overlay.origin_y = global_grid->getOriginY();
    overlay.robot_pose = global.robot_pose();
    overlay.has_robot_pose = global.has_robot_pose();
    overlay.local_overlay_valid = has_local;
    overlay.costs.resize(static_cast<std::size_t>(size_x) * size_y);
    for (unsigned int my = 0; my < size_y; ++my) {
      for (unsigned int mx = 0; mx < size_x; ++mx) {
        const unsigned char global_cost = global_grid->getCost(mx, my);
        unsigned char cost = global_cost;
        if (has_local) {
          double wx = 0.0;
          double wy = 0.0;
          global_grid->mapToWorld(mx, my, wx, wy);
          const Eigen::Vector3d p_local = local_from_global *
                                          Eigen::Vector3d(wx, wy, 0.0);
          unsigned int lx = 0;
          unsigned int ly = 0;
          if (local_grid->worldToMap(p_local.x(), p_local.y(), lx, ly)) {
            const unsigned char local_cost = local_grid->getCost(lx, ly);
            if (local_cost != costmap_2d::NO_INFORMATION && local_cost > cost) {
              cost = local_cost;  // unknown never downgrades
            }
          }
        }
        overlay.costs[static_cast<std::size_t>(my) * size_x + mx] = cost;
      }
    }
    overlay.valid = true;
    return true;
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
  /// @brief Resolve the robot pose in the requested frame via the
  /// RobotStateController singleton (the framework-wide TF query surface).
  bool robotPoseInFrame(ayanami::geometry_msgs::PoseStamped& pose,
                        const std::string& frame) const {
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    if (!ayanami::components::RobotStateController::instance().lookupTransform(
            frame, robot_frame, ayanami::Timestamp::latest(), T, 0.0)) {
      return false;
    }
    pose = geometry_msgs::PoseStamped::fromIsometry(frame, T);
    return true;
  }

  /// @brief Build a read view under a shared lock: grid and frame come from
  /// the wrapper (the wrapper's frame is authoritative for its grid); the
  /// footprint comes from the immutable configure-time cache; robot_pose is
  /// resolved in the wrapper frame via the RobotStateController.
  CostmapView buildView(
      const costmap_2d::CostmapWrapper::Ptr& wrapper) const {
    if (!wrapper) {
      return CostmapView{};
    }
    std::shared_lock<std::shared_mutex> lock(wrapper->mutex());
    const auto* grid = wrapper->get().getCostmap();
    ayanami::geometry_msgs::PoseStamped robot_pose;
    const bool has_robot_pose =
        robotPoseInFrame(robot_pose, wrapper->getGlobalFrame());
    return CostmapView(std::move(lock), grid, wrapper->getGlobalFrame(),
                       &cached_footprint, std::move(robot_pose),
                       has_robot_pose, wrapper->isCurrent());
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
