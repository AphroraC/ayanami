#pragma once

#include <memory>
#include <string>

#include <costmap_server/costmap_wrapper.hpp>

#include "ayanami/abstract/components/robot_state_controller.hpp"
#include "ayanami/map/abstract_map_context.hpp"
#include "ayanami/costmap/core/costmap_command.hpp"
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
 * timescale split (nav2 convention):
 * - global costmap frame = global_costmap_frame (typically "map"):
 *   static layer + (optional) dynamic obstacle layer + inflation (nav2
 *   global_costmap shape); the dynamic layer is enabled by
 *   costmap_global.observation_sources and receives the same observations
 *   as the local map, so replanning and path validation see real
 *   obstacles;
 * - local costmap frame = local_costmap_frame (typically "odom"): rolling
 *   window of dynamic obstacle marks. Being in the odometry frame makes the
 *   local map immune to localization (AMCL) jumps: corrections move the
 *   map<-odom edge, not the odom<-sensor edges the local marks were stamped
 *   with, so short-term local geometry stays consistent.
 *
 * Ownership and holding discipline:
 * - Ownership: the host (CostmapNavigation / CostmapNavigationServer) holds
 *   CostmapMapContext::Ptr; family modules (planner/smoother/controller/
 *   recovery) hold a ConstPtr (const smart pointer) on demand via
 *   configure(context, map) and never modify the map.
 * - Map reads: modules read through readGlobal()/readLocal(), which hand a
 *   CostmapReader to a callback and release the lock when it returns
 *   (RAII shared lock + zero-copy direct master grid reads).
 * - Mutable wrapper/layer access (globalCostmap()/localCostmap()) is
 *   provided only to the map writer (CostmapNavigationServer) and the
 *   execution lifecycle (checkActivate/checkDeactivate/current); rule
 *   modules never use it.
 * - All TF queries go through the RobotStateController global singleton
 *   (the framework has exactly one transform query surface; no per-module
 *   getRobotPose-style wrappers).
 *
 * Family read contract (timescale split):
 * - planner: GLOBAL map only (the planning timescale; fresh local
 *   observations never shape the global plan) -- the module reads the grid
 *   directly under readGlobal() for the duration of one planning call
 *   (nav2-style direct reads; the map writer is incremental, so a search
 *   holding the shared lock only delays the writer, never vice versa);
 * - smoother: GLOBAL+LOCAL read capability as needed (collision checking on
 *   both timescales; paths expressed in the global frame are transformed
 *   into each grid's frame once per call);
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

  /// @brief Path validity check on the GLOBAL map only (the planning
  /// timescale; semantics aligned with nav2 planner_server::isPathValid,
  /// which validates against the planning = global costmap). When the
  /// global dynamic obstacle layer is enabled
  /// (costmap_global.observation_sources), fresh obstacle marks invalidate
  /// the plan exactly like nav2; local-only marks remain the controller's
  /// timescale and surface through the controller's own failure codes.
  /// Checking starts from the path point nearest the robot -- segments
  /// already traversed may be occupied by new obstacles and are a source of
  /// false negatives, so they are skipped explicitly; every point must be
  /// in bounds (out of bounds treated as lethal, mirroring nav2) and its
  /// cost must be neither NO_INFORMATION nor lethal (isLethal >= 253
  /// includes inscribed, mirroring nav2 use_radius mode). Path validation
  /// is an O(path length) SPARSE probe: the shared lock is held only for
  /// the point scan (short critical section). Degrades to a non-empty check
  /// when no global wrapper exists; degrades to a full-path check
  /// (conservative) when the robot pose is unavailable.
  bool isPathValid(const ayanami::nav_msgs::Path& path) const {
    if (path.poses.empty()) {
      return false;
    }
    if (!global_costmap) {
      return true;
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
        const auto cost = global_grid->getCost(mx, my);
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
  /// @return false when no global wrapper exists (out is reset to empty).
  bool snapshotGlobal(costmap::CostmapSnapshot& out) const {
    out = costmap::CostmapSnapshot{};
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
    const auto* map = grid->getCharMap();
    out.costs.assign(map, map + static_cast<std::size_t>(out.size_x) *
                                       out.size_y);
    return true;
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
  std::string global_frame{"map"};
  std::string local_frame{"odom"};
  std::string robot_frame{"base_link"};
  costmap_2d::Footprint cached_footprint;
  map::MapCommandPort::Ptr command_port;
};

}  // namespace costmap
}  // namespace ayanami
