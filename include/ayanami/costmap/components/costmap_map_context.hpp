#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>

#include <costmap/costmap_wrapper.hpp>
#include <costmap/simple_obstacle_layer.hpp>

#include "ayanami/abstract/components/robot_state_controller.hpp"
#include "ayanami/costmap/core/costmap_reader.hpp"
#include "ayanami/costmap/core/costmap_snapshot.hpp"
#include "ayanami/messages/nav_msgs.hpp"
#include "ayanami/messages/geometry_msgs.hpp"
#include "ayanami/messages/timestamp.hpp"

namespace ayanami {
namespace costmap {

/**
 * @brief The costmap family "map module" (two framed grids).
 *
 * The global and the local costmap live in DIFFERENT frames, with a strict
 * timescale split (static-global policy):
 * - global costmap frame = global_costmap_frame (typically "map"):
 *   static layer + inflation only (nav2-shaped but STATIC) -- the map-level
 *   geometry never carries fresh obstacle marks; a dynamic obstacle layer
 *   remains technically loadable for special deployments
 *   (costmap_global.layers), but production runs static-only;
 * - local costmap frame = local_costmap_frame (typically "odom"): rolling
 *   window carrying ALL dynamic obstacle marks (+ inflation). Being in the
 *   odometry frame makes the local map immune to localization (AMCL) jumps:
 *   corrections move the map<-odom edge, not the odom<-sensor edges the
 *   local marks were stamped with, so short-term local geometry stays
 *   consistent.
 *
 * Ownership and holding discipline:
 * - Ownership: the host (CostmapNavigation / CostmapNavigationServer) holds
 *   CostmapMapContext::Ptr; family modules (planner/smoother/controller/
 *   recovery) hold a ConstPtr (const smart pointer) on demand via
 *   configure(context, map) and never modify the map.
 * - Map reads: modules read through readGlobal()/readLocal(), which hand a
 *   CostmapReader to a callback and release the lock when it returns
 *   (RAII shared lock + zero-copy direct master grid reads), or take
 *   short-lock snapshots (snapshotGlobal/snapshotLocal, generation-stamped
 *   -- equal generation means byte-identical content).
 * - Mutable wrapper/layer access (globalCostmap()/localCostmap()) is
 *   provided only to the map writer (CostmapNavigationServer) and the
 *   execution lifecycle (checkActivate/checkDeactivate/current); rule
 *   modules never use it.
 * - All TF queries go through the RobotStateController global singleton
 *   (the framework has exactly one transform query surface; no per-module
 *   getRobotPose-style wrappers).
 *
 * Family read contract (timescale split + overlay):
 * - planner: held GLOBAL copy (generation-gated lazy refresh) + ONE local
 *   snapshot per planning call, planned on the max-fused overlay
 *   (FusedMapView); fresh local obstacles divert the plan, the global grid
 *   carries the map-level geometry. Absent local map / transform degrades
 *   to global-only;
 * - smoother: same overlay model (held global copy + one local snapshot
 *   per call) for two-grid collision checking;
 * - controller/recovery: read the local map (family-defined), the
 *   controller generation-gates its private copy.
 */
class CostmapMapContext
    : public std::enable_shared_from_this<CostmapMapContext> {
 public:
  using Ptr = std::shared_ptr<CostmapMapContext>;
  using ConstPtr = std::shared_ptr<const CostmapMapContext>;

  CostmapMapContext() = default;

  /// @brief Path validity check on the fused global+local overlay (the
  /// planning timescale; semantics aligned with nav2
  /// planner_server::isPathValid but overlay-aware: under the static-global
  /// policy fresh obstacle marks live ONLY in the local rolling map, so a
  /// plan is valid only when it clears BOTH the global geometry and the
  /// local marks inside the local window; points outside the local window
  /// are gated by the global grid alone).
  /// Checking starts from the path point nearest the robot -- segments
  /// already traversed may be occupied by new obstacles and are a source of
  /// false negatives, so they are skipped explicitly; every point must be
  /// in bounds (out of bounds treated as lethal, mirroring nav2) and its
  /// fused cost must be neither NO_INFORMATION nor lethal (isLethal >= 253
  /// includes inscribed, mirroring nav2 use_radius mode). The local overlay
  /// (one short-lock snapshot + one lock-free transform lookup) is prepared
  /// BEFORE the global read lock -- locks never nest; a missing local map or
  /// transform degrades the whole check to global-only. Path validation
  /// is an O(path length) SPARSE probe: the shared lock is held only for
  /// the point scan (short critical section). Degrades to a non-empty check
  /// when no global wrapper exists; degrades to a full-path check
  /// (conservative) when the robot pose is unavailable.
  ///
  /// Robot-adjacent exemption: the contiguous prefix of path points within
  /// the cached footprint's inscribed radius of the robot pose is exempt
  /// from the band check -- those points are where the robot ALREADY is. A
  /// robot that grazed the inscribed band while tracking (a legal 252-cell
  /// path plus tracking wobble) would otherwise fail every future
  /// validation AND every replan (START_INVALID deadlock); driving forward
  /// out of the band is the correct recovery, and points beyond the body
  /// radius keep full gating.
  bool isPathValid(const ayanami::nav_msgs::Path& path) const {
    if (path.poses.empty()) {
      return false;
    }
    if (!global_costmap) {
      return true;
    }
    // Local overlay prepared BEFORE the global read lock: one short-lock
    // snapshot + one lock-free transform lookup, never nested with (or
    // inside) the global shared lock. A missing local map or transform
    // degrades the whole check to global-only (static walls still gate).
    // (No generation-gated snapshot cache here by design: raw-grid test
    // seams mutate the layer raster without the wrapper's generation
    // counter, and the per-call copy is only ~15 KB at supervision rate.)
    ayanami::costmap::CostmapSnapshot local;
    Eigen::Isometry3d local_from_global = Eigen::Isometry3d::Identity();
    const bool have_local = snapshotLocal(local) && local.valid() &&
                            localFromGlobal(local_from_global);
    // Precomputed 2D rows of local_from_global: identical arithmetic to the
    // Isometry multiply for z=0 points (the z row/column contribute +0.0
    // terms only), without the 3D plumbing. This scan sits on the FSM's
    // supervision path and runs every follow tick.
    double local_r00 = 1.0;
    double local_r01 = 0.0;
    double local_r10 = 0.0;
    double local_r11 = 1.0;
    double local_tx = 0.0;
    double local_ty = 0.0;
    if (have_local) {
      local_r00 = local_from_global(0, 0);
      local_r01 = local_from_global(0, 1);
      local_r10 = local_from_global(1, 0);
      local_r11 = local_from_global(1, 1);
      local_tx = local_from_global.translation().x();
      local_ty = local_from_global.translation().y();
    }
    return readGlobal([&](const CostmapReader& global) {
      if (!global.valid()) {
        return true;
      }
      const auto* global_grid = global.grid();
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
        // Robot-adjacent exemption (see the class doc): skip the contiguous
        // prefix of points inside the robot's own body radius. The robot
        // standing there proves those points survivable; probing them turns
        // a band-grazing tracking wobble into a permanent validation failure.
        while (start_index < path.poses.size() &&
               (path.poses[start_index].pose.position -
                global.robot_pose().pose.position)
                   .norm() <= cached_inscribed_radius) {
          ++start_index;
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
        auto cost = global_grid->getCost(mx, my);
        if (have_local) {
          // Fused overlay: the local window's fresh marks gate the near
          // path segment exactly like the planner's overlay model. The
          // inline guard/cast mirrors CostmapSnapshot::worldToMap (reject
          // below-origin, trunc == floor for non-negative, bounds check).
          const double px =
              local_r00 * path.poses[i].pose.position.x() +
              local_r01 * path.poses[i].pose.position.y() + local_tx;
          const double py =
              local_r10 * path.poses[i].pose.position.x() +
              local_r11 * path.poses[i].pose.position.y() + local_ty;
          if (px >= local.origin_x && py >= local.origin_y) {
            const unsigned int lmx = static_cast<unsigned int>(
                (px - local.origin_x) / local.resolution);
            const unsigned int lmy = static_cast<unsigned int>(
                (py - local.origin_y) / local.resolution);
            if (lmx < local.size_x && lmy < local.size_y) {
              const unsigned char local_cost = local.getCost(lmx, lmy);
              if (local_cost > cost) {
                cost = local_cost;
              }
            }
          }
        }
        if (cost == costmap_2d::NO_INFORMATION ||
            costmap_2d::isLethal(cost)) {
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
    // Cache the local obstacle layer's read view: layer objects live for
    // the wrapper's lifetime (static composition), so the raw pointer is
    // stable. Const: readers only see presenceGrid()/geom accessors.
    local_obstacle_layer = nullptr;
    if (local_costmap) {
      for (const auto& layer : local_costmap->get().getModules()) {
        if (auto* obstacle = dynamic_cast<const costmap_2d::SimpleObstacleLayer*>(
                layer.get())) {
          local_obstacle_layer = obstacle;
          break;
        }
      }
    }
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
  /// the write lock). The inscribed radius (min distance from the footprint
  /// origin = the robot center to the polygon edges, LayeredCostmap
  /// semantics) is derived once here: it is the isPathValid robot-adjacent
  /// exemption radius.
  void setFootprint(costmap_2d::Footprint footprint) {
    cached_footprint = std::move(footprint);
    double radius = 0.0;
    const std::size_t n = cached_footprint.size();
    if (n >= 3) {
      radius = std::numeric_limits<double>::max();
      for (std::size_t i = 0; i < n; ++i) {
        const auto& a = cached_footprint[i];
        const auto& b = cached_footprint[(i + 1) % n];
        const double ex = b.x - a.x;
        const double ey = b.y - a.y;
        const double length_sq = ex * ex + ey * ey;
        const double t = length_sq > 0.0
                             ? std::clamp(-(a.x * ex + a.y * ey) / length_sq,
                                          0.0, 1.0)
                             : 0.0;
        radius = std::min(radius, std::hypot(a.x + t * ex, a.y + t * ey));
      }
    }
    cached_inscribed_radius = radius;
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

  /// @brief Short-lock copy of the global master grid (the planner's read
  /// contract): the shared lock is held only for the memcpy, the robot pose
  /// is resolved BEFORE the lock (never a transform lookup under a map
  /// lock), and the caller runs its search on the immutable snapshot with
  /// no map lock held -- the writer is never delayed by planning.
  /// The out buffer's costs capacity is reused across calls (assign, not a
  /// fresh allocation); the copied grid's generation is recorded in
  /// out.generation.
  /// @return false when no global wrapper exists (scalar fields are reset so
  /// valid() is false; the costs buffer keeps its capacity for reuse).
  bool snapshotGlobal(costmap::CostmapSnapshot& out) const {
    resetSnapshotScalars(out);
    if (!global_costmap) {
      return false;
    }
    const std::string frame = global_costmap->globalFrame();
    ayanami::geometry_msgs::PoseStamped robot_pose;
    const bool has_robot_pose = robotPoseInFrame(robot_pose, frame);
    std::shared_lock<std::shared_mutex> lock(global_costmap->mutex());
    const auto* grid = global_costmap->get().getCostmap();
    if (!grid || grid->getSizeInCellsX() == 0 || grid->getSizeInCellsY() == 0) {
      return false;
    }
    out.frame = frame;
    out.size_x = grid->getSizeInCellsX();
    out.size_y = grid->getSizeInCellsY();
    out.resolution = grid->getResolution();
    out.origin_x = grid->getOriginX();
    out.origin_y = grid->getOriginY();
    out.robot_pose = robot_pose;
    out.has_robot_pose = has_robot_pose;
    out.generation = global_costmap->generation();
    const auto* map = grid->getCharMap();
    out.costs.assign(map, map + static_cast<std::size_t>(out.size_x) *
                                       out.size_y);
    return true;
  }

  /// @brief Local-map counterpart of snapshotGlobal (the smoother's short
  /// lock copy of the rolling window). Same buffer-reuse and generation
  /// recording contract as snapshotGlobal.
  bool snapshotLocal(costmap::CostmapSnapshot& out) const {
    resetSnapshotScalars(out);
    if (!local_costmap) {
      return false;
    }
    const std::string frame = local_costmap->globalFrame();
    ayanami::geometry_msgs::PoseStamped robot_pose;
    const bool has_robot_pose = robotPoseInFrame(robot_pose, frame);
    std::shared_lock<std::shared_mutex> lock(local_costmap->mutex());
    const auto* grid = local_costmap->get().getCostmap();
    if (!grid || grid->getSizeInCellsX() == 0 || grid->getSizeInCellsY() == 0) {
      return false;
    }
    out.frame = frame;
    out.size_x = grid->getSizeInCellsX();
    out.size_y = grid->getSizeInCellsY();
    out.resolution = grid->getResolution();
    out.origin_x = grid->getOriginX();
    out.origin_y = grid->getOriginY();
    out.robot_pose = robot_pose;
    out.has_robot_pose = has_robot_pose;
    out.generation = local_costmap->generation();
    const auto* map = grid->getCharMap();
    out.costs.assign(map, map + static_cast<std::size_t>(out.size_x) *
                                       out.size_y);
    return true;
  }

  /// @brief Short-lock copy of the LOCAL obstacle PRESENCE mask (the
  /// SimpleObstacleLayer's raw marks BEFORE inflation -- dynamic obstacles
  /// only, no decayed halo), expressed in the local map's own frame
  /// (typically odom). Reuses the CostmapSnapshot carrier: scalars + frame
  /// + generation describe the mask's geometry, `costs` carries the raw
  /// presence bytes (nonzero = obstacle). Returns false when no local map
  /// / obstacle layer exists yet. Consumers: the ESDF mask source and
  /// point-level controllers (e.g. the neupan controller's obstacle feed).
  bool snapshotLocalObstaclePresence(costmap::CostmapSnapshot& out) const {
    resetSnapshotScalars(out);
    if (!local_costmap || !local_obstacle_layer) {
      return false;
    }
    std::shared_lock<std::shared_mutex> lock(local_costmap->mutex());
    const auto& presence = local_obstacle_layer->presenceGrid();
    if (presence.empty() || local_obstacle_layer->geomSizeX() == 0) {
      return false;
    }
    out.frame = local_costmap->globalFrame();
    out.size_x = local_obstacle_layer->geomSizeX();
    out.size_y = local_obstacle_layer->geomSizeY();
    out.resolution = local_obstacle_layer->geomResolution();
    out.origin_x = local_obstacle_layer->geomOriginX();
    out.origin_y = local_obstacle_layer->geomOriginY();
    out.generation = local_costmap->generation();
    out.costs.assign(presence.begin(), presence.end());
    return true;
  }

  /// @brief Current master-grid generation (CostmapWrapper::generation(); 0
  /// when the map does not exist). Lock-free atomic read: a consumer that
  /// holds a snapshot compares against this to skip re-copying an unchanged
  /// grid -- equal generation means byte-identical content.
  std::uint64_t globalGeneration() const {
    return global_costmap ? global_costmap->generation() : 0;
  }
  std::uint64_t localGeneration() const {
    return local_costmap ? local_costmap->generation() : 0;
  }

  /// @brief Resolve the global-frame -> local-frame transform (where the
  /// local grid sits inside the global grid), OUTSIDE any map lock, for
  /// callers that combine a global copy with a local snapshot (overlay
  /// planning). Uses the RobotStateController with latest-time semantics;
  /// returns false when the transform is unavailable.
  bool localFromGlobal(Eigen::Isometry3d& local_from_global) const {
    return ayanami::components::RobotStateController::instance()
        .lookupTransform(local_frame, global_frame,
                         ayanami::Timestamp::latest(), local_from_global, 0.0);
  }

 private:
  /// @brief Reset a snapshot's scalar fields for reuse: valid() becomes
  /// false while the costs buffer KEEPS its capacity (the next successful
  /// snapshot assign()s into it instead of reallocating).
  static void resetSnapshotScalars(costmap::CostmapSnapshot& out) {
    out.size_x = 0;
    out.size_y = 0;
    out.resolution = 0.05;
    out.origin_x = 0.0;
    out.origin_y = 0.0;
    out.frame.clear();
    out.robot_pose = ayanami::geometry_msgs::PoseStamped{};
    out.has_robot_pose = false;
    out.generation = 0;
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

  costmap_2d::CostmapWrapper::Ptr global_costmap;
  costmap_2d::CostmapWrapper::Ptr local_costmap;
  /// @brief Read view of the local obstacle layer (cached on
  /// setLocalCostmap; stable for the wrapper's lifetime).
  const costmap_2d::SimpleObstacleLayer* local_obstacle_layer{nullptr};
  std::string global_frame{"map"};
  std::string local_frame{"odom"};
  std::string robot_frame{"base_link"};
  costmap_2d::Footprint cached_footprint;
  /// @brief Inscribed radius of cached_footprint (0 when unset) -- the
  /// isPathValid robot-adjacent exemption radius.
  double cached_inscribed_radius{0.0};
};

}  // namespace costmap
}  // namespace ayanami
