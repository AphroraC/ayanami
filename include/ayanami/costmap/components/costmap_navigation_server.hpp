#pragma once

#include <atomic>
#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include <costmap_server/costmap_wrapper.hpp>

#include "ayanami/abstract/components/navigation_server_base.hpp"
#include "ayanami/costmap/components/costmap_action.hpp"
#include "ayanami/costmap/components/costmap_map_context.hpp"
#include "ayanami/map/map_family_factory.hpp"
#include "ayanami/costmap/core/costmap_command.hpp"
#include "ayanami/costmap/core/costmap_controller.hpp"
#include "ayanami/costmap/core/costmap_planner.hpp"
#include "ayanami/costmap/core/costmap_recovery.hpp"
#include "ayanami/costmap/core/costmap_smoother.hpp"
#include "ayanami/messages/sensor_msgs.hpp"

namespace costmap_2d_layers {
class SimpleObstacleLayer;
class StaticLayer;
}

namespace ayanami {
namespace costmap {

/**
 * @brief costmap-specialized navigation server (mirroring the MBF
 * navfn/planner combination).
 *
 * On top of NavigationServerBase (map-free):
 * - Overrides the virtual initialize*Module hooks:
 *   dynamic_pointer_cast to the costmap family interfaces
 *   (CostmapPlanner/Smoother/Controller/Recovery); on failure (not a
 *   costmap-family module) spdlog::error + return false (not registered).
 *   Algorithm modules receive only name-based initialization; no
 *   CostmapMapContext is injected -- all map access goes through lock-based
 *   direct reads in the executions.
 * - Overrides createActions(): the production path constructs Costmap*Action
 *   directly (with CostmapMapContext + execution configuration: checkers/
 *   velocity limits/robot state source), replacing the base class's
 *   new*Execution virtual factory indirection layer.
 * - configureCoreMaps(config_path) reads config_navigation and builds the
 *   global/local LayeredCostmap pair in DIFFERENT frames:
 *   global = StaticLayer+InflationLayer in global_costmap_frame (typically
 *   "map"); local = rolling SimpleObstacleLayer+InflationLayer in
 *   local_costmap_frame (typically "odom" -- the odometry frame keeps the
 *   rolling dynamic window immune to localization jumps, nav2 convention).
 *   Every change (obstacle stamping / command-port clear) is applied by this
 *   server as the sole writer via updateMap -> publishing an immutable epoch.
 * - Obstacle observations do not go through global callback slots: the host
 *   (CostmapNavigation) hands setPointcloud/setLaserScan straight to the
 *   common handleObstacle* methods to stamp the map (input/output
 *   separation discipline). Sensor points are transformed into each map's
 *   own frame (stamp-time query preferred) and stamped per map.
 * - All TF queries go through the RobotStateController singleton (the
 *   framework-wide transform surface); its frame names + velocity_timeout
 *   are configured once in configureCoreMaps. The controller execution reads
 *   the latest TF pose + latest velocity through it (zero velocity when
 *   stale).
 *
 * Before a dynamic obstacle update, clearArea the previous observation
 * window (including the old inflation band) so the InflationLayer
 * recomputes within its recompute window and no stale inflation band
 * remains after an obstacle moves away.
 */
class CostmapNavigationServer
    : public ayanami::components::NavigationServerBase {
 public:
  using Ptr = std::shared_ptr<CostmapNavigationServer>;

  explicit CostmapNavigationServer();
  ~CostmapNavigationServer() override;

  /// @brief Read config_navigation and build the global/local costmaps
  /// (StaticLayer + InflationLayer + SimpleObstacleLayer), injecting the
  /// map_context.
  /// @param factory Map-family factory (the sole context creation point);
  ///        falls back to CostmapMapFamilyFactory when empty. Contexts
  ///        created via the factory are recorded in mapFamilyFactory for
  ///        test/assembly auditing.
  void configureCoreMaps(
      const std::string& config_path,
      ayanami::map::MapFamilyFactory::Ptr factory = nullptr);

  /// @brief The map-family factory actually used by configureCoreMaps (the abstract surface; the core only knows this type).
  ayanami::map::MapFamilyFactory::Ptr mapFamilyFactory() const {
    return map_family_factory;
  }

  // ---- Virtual initialize*Module overrides: costmap family injection ----
  bool initializePlannerModule(
      const std::string& name, const ayanami::Config& module_config,
      ayanami::abstract::AbstractPlanner::Ptr) override;
  bool initializeSmootherModule(
      const std::string& name, const ayanami::Config& module_config,
      ayanami::abstract::AbstractSmoother::Ptr) override;
  bool initializeControllerModule(
      const std::string& name, const ayanami::Config& module_config,
      ayanami::abstract::AbstractController::Ptr) override;
  bool initializeRecoveryModule(
      const std::string& name, const ayanami::Config& module_config,
      ayanami::abstract::AbstractRecovery::Ptr) override;

  /// @brief Path validity (the BT-readable surface of the responsibility
  /// table's "is the path stale"): delegates to the costmap context's
  /// overlay check (global+local, lethal/unknown/out of bounds).
  bool isPathValid(const ayanami::nav_msgs::Path& path) const override {
    return costmap_context && costmap_context->isPathValid(path);
  }

  // ---- createActions override: production path = Costmap*Action (costmap_action) ----
  void createActions() override;

  // ---- Obstacle observation signal entry points ----
  /// @brief Transform the point cloud (sensor frame) into the shared map
  /// frame via the robot-state transform lookup, stamp it into the
  /// SimpleObstacleLayer of both maps and trigger updateMap (including
  /// old-band clearing).
  void handleObstaclePointCloud(const ayanami::sensor_msgs::PointCloud2& cloud);
  /// @brief Convert laser ranges to points and run the same map-stamping pipeline.
  void handleObstacleLaserScan(const ayanami::sensor_msgs::LaserScan& scan);

  // ---- Accessors (tests / host) ----
   CostmapMapContext::Ptr costmapContext() const { return costmap_context; }
   CostmapMapContext::ConstPtr costmapContextConst() const {
     return costmap_context ? costmap_context->getSharedConstPtr() : nullptr;
   }
  costmap_2d::CostmapWrapper::Ptr globalCostmap() const {
    return costmap_context ? costmap_context->globalCostmap() : nullptr;
  }
  costmap_2d::CostmapWrapper::Ptr localCostmap() const {
    return costmap_context ? costmap_context->localCostmap() : nullptr;
  }

  /// @brief Map change command port (write commands such as clear; recovery modules submit through it).
  CostmapCommandPort::Ptr commandPort() const { return command_port; }

  /// @brief Execute one clear-costmap command (sole-writer path: mutate the
  /// layer + updateMap -> publish a new epoch). Reused by the command port
  /// implementation and tests.
  bool executeClearCommand(const ClearCostmapCommand& command);

  /// @brief Periodic rolling update (called by the host's spinOnce,
  /// decoupled from the sensor stream): for maps with rolling_window
  /// enabled, center the window on the latest map-frame robot pose and
  /// recompute (when no window is dirty, updateMap exits early right after
  /// updateOrigin, so the cost is negligible). When the scan stream stops,
  /// the window still follows the robot -- otherwise DWB forward trajectories
  /// gradually "leave the map" and the robot gets stuck at the edge of a
  /// frozen window with an undetectable current flag. Pose resolution
  /// happens before taking the map lock.
  void updateRollingWindows();

 private:
  /// @brief Transform points in frame_id (at stamp time) into the shared map
  /// frame and stamp them into both maps via TransformStore (shared by
  /// handleObstacle*). The stamp queries TF using the sensor message's
  /// header.stamp.
  void markObstaclesInFrame(const std::string& frame_id,
                            const ayanami::Timestamp& stamp,
                            const std::vector<Eigen::Vector3d>& points,
                            const std::vector<Eigen::Vector3d>& clearing_points = {});

  CostmapMapContext::Ptr costmap_context;
  CostmapCommandPort::Ptr command_port;
  ayanami::map::MapFamilyFactory::Ptr map_family_factory;
  /// @brief Per-recovery module configs (stashed by initializeRecoveryModule,
  /// consumed by createActions to inject per-module execution settings such
  /// as the tick cycle period).
  std::map<std::string, ayanami::Config> recovery_configs;

  // Throttling for missing TF chain warnings (every sensor callback can
  // fail; warn at fixed intervals only, avoiding flooding at 5.5Hz laser
  // per frame). The map-frame sensor chain must lack TF for longer than
  // the interval before warning again.
  std::atomic<std::chrono::steady_clock::time_point> map_tf_warn_at{};
  std::atomic<std::chrono::steady_clock::time_point> robot_diag_at{};
  // Robot pose caches, one per map frame (global/local): on a transient pose
  // query failure, roll the window with the last successful value and never
  // fall back to (0,0,0) -- otherwise the rolling costmap window gets
  // anchored back to the frame origin and, once the robot walks out of the
  // window, dwb has no path (stuttering in place).
  bool has_last_local_robot{false};
  double last_local_robot_x{0.0}, last_local_robot_y{0.0};
  double last_local_robot_yaw{0.0};
  bool has_last_global_robot{false};
  double last_global_robot_x{0.0}, last_global_robot_y{0.0};
  double last_global_robot_yaw{0.0};
  // Each map owns an obstacle layer so dynamic markers and static content
  // combine per the map's own combination method.
  costmap_2d_layers::SimpleObstacleLayer* global_obstacle_layer{nullptr};
  costmap_2d_layers::SimpleObstacleLayer* local_obstacle_layer{nullptr};
  costmap_2d_layers::StaticLayer* global_static_layer{nullptr};
  double global_inflation_radius{0.6};
  double local_inflation_radius{0.6};
  double global_resolution{0.05};
  double local_resolution{0.05};
  double local_raytrace_range{3.0};
  // Previous observations are stored in map frame for bounded clearing.
  std::vector<Eigen::Vector2d> prev_obstacles_global;
  std::vector<Eigen::Vector2d> prev_obstacles_local;
  bool has_prev_obstacles_global{false};
  bool has_prev_obstacles{false};
 // Decoupled rolling-window updates (spinOnce): period and last trigger time
  std::chrono::duration<double> local_roll_period{0.1};   // 10Hz default
  std::chrono::duration<double> global_roll_period{0.2};  // 5Hz default
  std::chrono::steady_clock::time_point last_local_roll{};
  std::chrono::steady_clock::time_point last_global_roll{};
 // Map-stamp scan-time query stats (ok/fallback to latest) and throttled diagnostic timestamps
  std::atomic<std::uint64_t> stamp_query_ok{0};
  std::atomic<std::uint64_t> stamp_query_fallback{0};
  std::chrono::steady_clock::time_point last_stamp_diag_at{};
};

}  // namespace costmap
}  // namespace ayanami
