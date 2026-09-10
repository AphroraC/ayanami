#pragma once

#include <atomic>
#include <chrono>
#include <limits>
#include <memory>
#include <shared_mutex>
#include <string>
#include <type_traits>
#include <utility>

#include <spdlog/spdlog.h>

#include <Eigen/Geometry>

#include <costmap_2d/costmap_2d.hpp>
#include <costmap_server/costmap_wrapper.hpp>

#include "ayanami/abstract/components/robot_state_controller.hpp"
#include "ayanami/map/abstract_map_context.hpp"
#include "ayanami/costmap/core/costmap_command.hpp"
#include "ayanami/costmap/core/planning_overlay.hpp"
#include "ayanami/costmap/core/costmap_reader.hpp"
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
 *   CostmapMapContext::Ptr; family modules (planner/smoother/controller/
 *   recovery) hold a ConstPtr (const smart pointer) on demand via
 *   configure(context, map) and never modify the map.
 * - Map reads: modules read through readGlobal()/readLocal(), which hand a
 *   CostmapReader to a callback and release the lock when it returns
 *   (RAII shared lock + zero-copy direct master grid reads) and hold the
 *   lock for the duration of one short algorithm call; the map writer waits
 *   for the lock to write. The planner is the exception: it materializes its
 *   input via composePlanningOverlay() (locks held only for the compose
 *   pass) and searches the returned buffer lock-free.
 * - Mutable wrapper/layer access (globalCostmap()/localCostmap()) is
 *   provided only to the map writer (CostmapNavigationServer) and the
 *   execution lifecycle (checkActivate/checkDeactivate/current); rule
 *   modules never use it.
 * - All TF queries go through the RobotStateController global singleton
 *   (the framework has exactly one transform query surface; no per-module
 *   getRobotPose-style wrappers).
 *
 * Family read contract:
 * - planner: global+local overlay via composePlanningOverlay() (global-frame
 *   buffer; the local<-global transform is resolved exactly once per
 *   compose, never per searched node);
 * - smoother: reads only the local map (readLocal(); paths expressed in the
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
  /// mode). Path validation is an O(path length) SPARSE probe: both shared
  /// locks are held only for the point scan (short critical section), never
  /// a whole-map materialization -- composePlanningOverlay exists for the A*
  /// hot path. A readable local map whose local<-global transform is missing
  /// degrades the check to global-only and emits a throttled warning.
  /// Degrades to a non-empty check when no global wrapper exists; degrades
  /// to a full-path check (conservative) when the robot pose is unavailable.
  bool isPathValid(const ayanami::nav_msgs::Path& path) const {
    if (path.poses.empty()) {
      return false;
    }
    if (!global_costmap) {
      return true;
    }
    // Resolve the local participation BEFORE taking map locks (the writer
    // discipline). A missing local<-global transform is a degraded overlay:
    // local lethal obstacles would silently drop out of the check, so warn
    // (throttled) instead of failing silently.
    Eigen::Isometry3d local_from_global = Eigen::Isometry3d::Identity();
    const bool has_local = resolveLocalParticipation(local_from_global,
                                                     /*warn_missing=*/true);
    // Short critical section: both shared locks are held only for the
    // O(path length) point scan -- path validation is a sparse probe, NOT a
    // whole-map materialization (composePlanningOverlay exists for the A*
    // hot path; per-point queries here would pay O(map) for O(path) work).
    // Both maps are read inside nested callbacks (local first, then global:
    // the same acquisition order the writer and composePlanningOverlay use).
    // The locks therefore span exactly this O(path) probe and nothing else.
    return readLocal([&](const CostmapReader& local) {
      return readGlobal([&](const CostmapReader& global) {
        if (!global.valid()) {
          return true;
        }
        const auto* global_grid = global.grid();
        const auto* local_grid =
            (has_local && local.valid()) ? local.grid() : nullptr;
        // Start from the nearest point: segments already traversed may be
        // occupied by new obstacles and are a false-positive source (mirrors
        // nav2 planner_server::isPathValid). Robot pose = global reader cache.
        std::size_t start_index = 0;
        if (global.hasRobotPose()) {
          double best = std::numeric_limits<double>::max();
          for (std::size_t i = 0; i < path.poses.size(); ++i) {
            const double d = (path.poses[i].pose.position -
                              global.robot_pose().pose.position)
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
          if (!global_grid->worldToMap(path.poses[i].pose.position.x(),
                                       path.poses[i].pose.position.y(), mx,
                                       my)) {
            // out of bounds: treated as lethal, mirroring nav2
            return false;
          }
          const auto cost = overlayPolicy.costAt(
              *global_grid, local_grid, local_from_global, mx, my);
          if (cost == costmap_2d::NO_INFORMATION ||
              costmap_2d::isLethal(cost)) {
            return false;
          }
        }
        return true;
      });
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
  /// Map-family modules such as recovery fetch it via commandPort instead
  /// of side-path injection.
  void setCommandPort(map::MapCommandPort::Ptr port) {
    command_port = std::move(port);
  }

  // ---- frames / freshness ----
  const std::string& globalFrame() const { return global_frame; }
  const std::string& localFrame() const { return local_frame; }
  const std::string& robotFrame() const { return robot_frame; }

  /// @brief False if any wrapper is not current; no wrapper counts as current.
  /// @note Noun form to match CostmapReader::current(). The wrapper keeps
  ///       isCurrent() because its flag is already named `current`.
  bool current() const {
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

  /// @brief Return a const owner for external observers and map-family modules.
  /// Map data is still only reachable through readGlobal/readLocal, so the
  /// underlying master grid cannot escape its lock scope.
  ConstPtr getSharedConstPtr() const {
    return shared_from_this();
  }

  // ---- Reader interface (shared lock + zero copy) ----

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
        local_costmap ? local_costmap->globalFrame() : std::string{};
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
    // lock-ordering cycle can form). Both locks live only for the compose pass
    // below; the caller searches the returned buffer lock-free.
    return readLocal([&](const CostmapReader& local) {
      return readGlobal([&](const CostmapReader& global) {
        if (!global.valid()) {
          return false;
        }
        const auto* global_grid = global.grid();
        const auto* local_grid = local.valid() ? local.grid() : nullptr;
        const bool local_in_overlay = has_local && local_grid != nullptr;

        const unsigned int size_x = global_grid->getSizeInCellsX();
        const unsigned int size_y = global_grid->getSizeInCellsY();
        overlay.frame = global_frame;
        overlay.size_x = size_x;
        overlay.size_y = size_y;
        overlay.resolution = global_grid->getResolution();
        overlay.origin_x = global_grid->getOriginX();
        overlay.origin_y = global_grid->getOriginY();
        overlay.robot_pose = global.robot_pose();
        overlay.hasRobotPose = global.hasRobotPose();
        overlay.local_overlay_valid = local_in_overlay;
        overlay.costs.resize(static_cast<std::size_t>(size_x) * size_y);
        for (unsigned int my = 0; my < size_y; ++my) {
          for (unsigned int mx = 0; mx < size_x; ++mx) {
            // local_in_overlay = false means the transform did not resolve: the
            // local grid must NOT participate (an identity-transform fallback
            // would stamp local marks at the wrong world coordinates).
            overlay.costs[static_cast<std::size_t>(my) * size_x + mx] =
                overlayPolicy.costAt(*global_grid,
                                     local_in_overlay ? local_grid : nullptr,
                                     local_from_global, mx, my);
          }
        }
        overlay.valid = true;
        return true;
      });
    });
  }

  /// @brief Run `fn` with read access to the global map.
  ///
  /// The only way to read a map: the shared lock lives in the reader handed to
  /// `fn` and is released when it returns, so no caller can hold the map (and
  /// therefore block the writer) past its own algorithm step. `fn` receives a
  /// `const CostmapReader&`, whose grid pointer must not escape the call.
  template <typename Fn>
  auto readGlobal(Fn&& fn) const
      -> decltype(std::forward<Fn>(fn)(
          std::declval<const CostmapReader&>())) {
    return read(global_costmap, std::forward<Fn>(fn));
  }

  /// @brief Local-map counterpart of readGlobal (the grid and the robot pose are
  /// in the local map's own frame, typically odom).
  template <typename Fn>
  auto readLocal(Fn&& fn) const
      -> decltype(std::forward<Fn>(fn)(
          std::declval<const CostmapReader&>())) {
    return read(local_costmap, std::forward<Fn>(fn));
  }

 private:
  /// @brief The costmap family's overlay composition rule.
  ///
  /// Named and kept separate because two independent paths apply it -- the
  /// planner's materialization (composePlanningOverlay) and path validation
  /// (isPathValid) -- and "both use the same rule" should be a fact about the
  /// code rather than a claim in a comment that drifts when one is edited.
  struct OverlayPolicy {
    /// @brief Cost of one global-grid cell once the local map has been folded in.
    ///
    /// Conservative per-cell max: the robot is blocked if either map says so.
    /// Three cases keep the local map from manufacturing free space or
    /// manufacturing obstacles at the wrong coordinates:
    /// - no local grid (absent, or its transform did not resolve): the global
    ///   cost stands, so an identity-transform fallback can never stamp local
    ///   marks at the wrong world coordinates;
    /// - the cell falls outside the local window: the global cost stands;
    /// - the local cell is NO_INFORMATION: the global cost stands, so local
    ///   unknown cannot clear global unknown.
    ///
    /// @param local_grid the LOCAL master grid (nullptr = global only)
    /// @param local_from_global transform from the global frame into the local
    ///        grid's frame (resolved once per query pass, never per cell).
    unsigned char costAt(const costmap_2d::Costmap2D& global_grid,
                         const costmap_2d::Costmap2D* local_grid,
                         const Eigen::Isometry3d& local_from_global,
                         unsigned int mx, unsigned int my) const {
      const unsigned char global_cost = global_grid.getCost(mx, my);
      if (local_grid == nullptr) {
        return global_cost;
      }
      double wx = 0.0;
      double wy = 0.0;
      global_grid.mapToWorld(mx, my, wx, wy);
      const Eigen::Vector3d p_local =
          local_from_global * Eigen::Vector3d(wx, wy, 0.0);
      unsigned int lx = 0;
      unsigned int ly = 0;
      if (!local_grid->worldToMap(p_local.x(), p_local.y(), lx, ly)) {
        return global_cost;  // outside the local window: global only
      }
      const unsigned char local_cost = local_grid->getCost(lx, ly);
      if (local_cost == costmap_2d::NO_INFORMATION) {
        return global_cost;  // unknown never downgrades
      }
      return global_cost > local_cost ? global_cost : local_cost;
    }
  };

  OverlayPolicy overlayPolicy;

  /// @brief Resolve whether the local map participates in overlay queries:
  /// true when a readable local grid exists AND the local<-global transform
  /// resolves (identity when both frames coincide). With warn_missing, a
  /// readable local grid whose transform is unavailable emits a throttled
  /// warning -- the overlay silently degrades to global-only, which drops
  /// local obstacles from path validation.
  bool resolveLocalParticipation(Eigen::Isometry3d& local_from_global,
                                 bool warn_missing) const {
    const auto* local_probe =
        local_costmap ? local_costmap->get().getCostmap() : nullptr;
    if (local_probe == nullptr) {
      return false;
    }
    const std::string local_grid_frame = local_costmap->globalFrame();
    if (local_grid_frame == global_frame) {
      return true;  // same frame: identity transform, no lookup needed
    }
    if (ayanami::components::RobotStateController::instance().lookupTransform(
            local_grid_frame, global_frame, ayanami::Timestamp::latest(),
            local_from_global, 0.0)) {
      return true;
    }
    if (warn_missing) {
      const auto now = std::chrono::steady_clock::now();
      auto prev = local_transform_warn_at.load();
      if (now - prev >= std::chrono::duration<double>(
                            k_local_transform_warn_interval_s) &&
          local_transform_warn_at.compare_exchange_strong(prev, now)) {
        spdlog::warn(
            "CostmapMapContext: no transform {} <- {}; path validation runs "
            "on the GLOBAL overlay only (local obstacles are NOT reflected). "
            "Check the TF producers of the local costmap frame",
            local_grid_frame, global_frame);
      }
    }
    return false;
  }

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

  /// @brief Build a read view: the robot pose is resolved BEFORE the lock is
  /// taken, then the lock and the grid pointer are acquired together.
  ///
  /// The order matters and used to be wrong here: the previous version took the
  /// map's shared lock and then called robotPoseInFrame(), i.e. it performed a
  /// transform lookup while holding the map lock -- the exact nesting the
  /// compose path documents as forbidden ("never nest the robot-state lock
  /// inside a map-lock wait"), and a lock-order inversion waiting for a writer
  /// to hit it. Resolving the pose first keeps the map lock free of any
  /// transform work.
  CostmapReader buildReader(
      const costmap_2d::CostmapWrapper::Ptr& wrapper) const {
    if (!wrapper) {
      return CostmapReader{};
    }
    const std::string frame = wrapper->globalFrame();
    ayanami::geometry_msgs::PoseStamped robot_pose;
    const bool has_robot_pose = robotPoseInFrame(robot_pose, frame);
    std::shared_lock<std::shared_mutex> lock(wrapper->mutex());
    const auto* grid = wrapper->get().getCostmap();
    return CostmapReader(std::move(lock), grid, frame, &cached_footprint,
                         std::move(robot_pose), has_robot_pose,
                         wrapper->isCurrent());
  }

  /// @brief Shared implementation of readGlobal/readLocal.
  template <typename Fn>
  auto read(const costmap_2d::CostmapWrapper::Ptr& wrapper, Fn&& fn) const
      -> decltype(std::forward<Fn>(fn)(
          std::declval<const CostmapReader&>())) {
    // The callback always runs, even without a wrapper: it receives an invalid
    // reader (valid() == false) and decides for itself. Skipping the call
    // instead would make "this map is absent" and "the callback never ran" look
    // the same from the caller's side.
    const CostmapReader reader = buildReader(wrapper);
    return std::forward<Fn>(fn)(reader);
  }

  /// @brief Throttle for the missing-local-transform warning of path
  /// validation (mutable: isPathValid is const).
  static constexpr double k_local_transform_warn_interval_s = 5.0;
  mutable std::atomic<std::chrono::steady_clock::time_point>
      local_transform_warn_at{};

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
