#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Geometry>

#include <costmap_2d/costmap_2d.hpp>

#include "ayanami/abstract/components/navigation_server_base.hpp"
#include "ayanami/map/map_family_factory.hpp"
#include "ayanami/rosemap/components/rosemap_action.hpp"
#include "ayanami/rosemap/core/rosemap_map_context.hpp"
#include "ayanami/rosemap/components/rosemap_map_family_factory.hpp"
#include "ayanami/rosemap/core/rosemap_command.hpp"
#include "ayanami/rosemap/core/rosemap_controller.hpp"
#include "ayanami/rosemap/core/rosemap_planner.hpp"
#include "ayanami/rosemap/core/rosemap_recovery.hpp"
#include "ayanami/rosemap/core/rosemap_smoother.hpp"
#include "ayanami/rosemap/core/rosemap_snapshot.hpp"
#include "ayanami/messages/sensor_msgs.hpp"

namespace ayanami {
namespace rosemap {

/**
 * @brief rosemap-family navigation server (a rolling signed-distance field
 * in place of the costmap family's layered pair).
 *
 * On top of NavigationServerBase (map-free):
 * - Overrides the virtual initialize*Module hooks:
 *   dynamic_pointer_cast to the rosemap family interfaces
 *   (RosemapPlanner/Smoother/Controller/Recovery); on failure (not a
 *   rosemap-family module) spdlog::error + return false (not registered).
 * - Overrides createActions(): constructs Rosemap*Action directly (with
 *   RosemapMapContext + the base-class checker/velocity assembly), mirroring
 *   the costmap server's production path.
 * - configureCoreMaps(config_path) reads config_navigation's "rosemap_field"
 *   section and builds ONE rolling occupancy grid (costmap_2d::Costmap2D
 *   storage) in field_frame; every writer cycle binarizes it, runs the exact
 *   signed EDT (edt.hpp) and publishes an immutable RosemapSnapshot into the
 *   context by an atomic shared_ptr swap. An empty snapshot is published at
 *   configure so consumers see valid geometry immediately.
 * - Observations: the host hands setPointcloud/setLaserScan straight to
 *   handleObstacle* (convert + enqueue, O(enqueue) on the caller's thread);
 *   the resident writer thread resolves stamp-time TFs (bounded wait; a
 *   degraded latest fallback gates this family's marks to clearing-only),
 *   filters the z band, raytrace-clears and marks the occupancy grid,
 *   recentres the rolling window on the robot and rebuilds the field. When
 *   the writer thread is not running (direct-construction tests), the same
 *   processing path executes inline on the caller's thread.
 * - The write-command surface (ResetFieldCommand via the context's
 *   commandPort) drops every occupancy mark; executed by this server as the
 *   sole writer.
 * - All TF queries go through the RobotStateController singleton; its frame
 *   names + velocity_timeout are configured once in configureCoreMaps.
 */
class RosemapNavigationServer
    : public ayanami::components::NavigationServerBase {
 public:
  using Ptr = std::shared_ptr<RosemapNavigationServer>;

  explicit RosemapNavigationServer();
  ~RosemapNavigationServer() override;

  // ---- lifecycle: start/stop also drive the resident field-writer thread ----
  void start() override;
  void stop() override;

  /// @brief Read config_navigation's rosemap_field section, build the
  /// rolling occupancy grid + the (factory-created) field context, publish
  /// the initial empty snapshot.
  /// @param factory Map-family factory (the sole context creation point);
  ///        falls back to RosemapMapFamilyFactory when empty.
  void configureCoreMaps(
      const std::string& config_path,
      ayanami::map::MapFamilyFactory::Ptr factory = nullptr);

  /// @brief The map-family factory actually used by configureCoreMaps.
  ayanami::map::MapFamilyFactory::Ptr mapFamilyFactory() const {
    return map_family_factory;
  }

  // ---- Virtual initialize*Module overrides: rosemap family injection ----
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

  /// @brief Path validity: delegates to the field context's sparse probe.
  bool isPathValid(const ayanami::nav_msgs::Path& path) const override {
    return field_context && field_context->isPathValid(path);
  }

  // ---- createActions override: production path = Rosemap*Action ----
  void createActions() override;

  // ---- Observation signal entry points (host-facing) ----
  /// @brief Buffer a point cloud frame (converted to sensor-frame points, z
  /// kept for the height filter); the writer transforms/marks/rebuilds.
  void handleObstaclePointCloud(const ayanami::sensor_msgs::PointCloud2& cloud);
  /// @brief Convert laser ranges to hit + clearing points and run the same
  /// pipeline.
  void handleObstacleLaserScan(const ayanami::sensor_msgs::LaserScan& scan);

  // ---- Accessors (tests / host) ----
  RosemapMapContext::Ptr fieldContext() const { return field_context; }
  RosemapMapContext::ConstPtr fieldContextConst() const {
    return field_context ? field_context->getSharedConstPtr() : nullptr;
  }

  /// @brief Field write-command port (ResetFieldCommand; recovery modules
  /// submit through it).
  RosemapCommandPort::Ptr commandPort() const { return command_port; }

  /// @brief Execute one reset command (sole-writer path: drop every mark +
  /// rebuild + publish). Reused by the command port implementation/tests.
  bool executeResetCommand(const ResetFieldCommand& command);

  /// @brief Run one full map cycle right now (inline driver for tests /
  /// no-writer mode): drain buffered observations, apply, rebuild, publish.
  void runMapCycleNow(bool force = true);

  /// @brief Block until every observation submitted so far has been applied
  /// by the writer (test seam; trivially true in inline mode).
  /// @return false on timeout.
  bool waitForMapIdle(double timeout_s) const;

  /// @brief Published snapshot sequence (observability/tests).
  std::uint64_t snapshotSequence() const {
    const auto snapshot =
        field_context ? field_context->currentSnapshot() : nullptr;
    return snapshot ? snapshot->sequence : 0;
  }

 private:
  /// @brief One buffered observation: sensor-frame points (z KEPT for the
  /// height band) + the laser's clearing rays.
  struct PendingObservation {
    std::string frame_id;
    ayanami::Timestamp stamp;
    std::vector<Eigen::Vector3d> points;
    std::vector<Eigen::Vector3d> clearing_points;
    /// @brief Arrival time (steady clock).
    std::chrono::steady_clock::time_point received{
        std::chrono::steady_clock::now()};
  };

  /// @brief One resolved observation inside a cycle batch.
  struct ResolvedFrame {
    std::shared_ptr<const PendingObservation> frame;
    Eigen::Isometry3d t_field_sensor{Eigen::Isometry3d::Identity()};
    bool stamp_ok{false};
    bool transform_ok{false};
  };

  /// @brief Enqueue an observation (bounded FIFO), or process inline when
  /// the writer thread is not running.
  void submitObservation(PendingObservation&& observation);
  /// @brief One mark/raytrace/rebuild/publish pass at update_frequency.
  void runMapCycle(bool force);
  /// @brief Stamp-time sensor transform with a bounded poll (up to
  /// transform_tolerance) before the latest fallback.
  bool resolveObservationTransform(const PendingObservation& observation,
                                   Eigen::Isometry3d& t_field_sensor,
                                   bool& stamp_ok);
  /// @brief Latest robot pose in the field frame, with the last-success
  /// cache fallback (never (0,0,0)).
  bool resolveRobotPose(double& robot_x, double& robot_y);
  /// @brief Raytrace-clear one ray and (for hit rays) mark the endpoint;
  /// all coordinates already in the field frame.
  void applyRay(const Eigen::Vector3d& origin, const Eigen::Vector3d& endpoint,
                bool endpoint_is_hit);
  /// @brief Rebuild the signed field from the occupancy grid and publish a
  /// new snapshot (sequence++).
  void rebuildAndPublish(double robot_x, double robot_y);

  /// @brief Convert the freshly built snapshot to OccupancyGrid and emit it
  /// on NavigationCallbacks::on_rosemap_field (writer thread; every
  /// publication carries a fresh sequence, so the emission is unconditional
  /// per update -- no rate parameter exists anywhere; receivers cache and
  /// throttle on their own). No locks held.
  void emitRosemapField(const std::shared_ptr<const RosemapSnapshot>& snapshot);
  /// @brief Re-centre the rolling window on the robot when it moved past
  /// recentre_threshold (whole-cell origin shift via Costmap2D::updateOrigin).
  /// @return true when the window moved (the field must be republished).
  bool recentreWindow(double robot_x, double robot_y);
  void startWriter();
  void stopWriter();
  void writerLoop();

  RosemapMapContext::Ptr field_context;
  RosemapCommandPort::Ptr command_port;
  ayanami::map::MapFamilyFactory::Ptr map_family_factory;
  /// @brief Per-recovery module configs (stashed by initializeRecoveryModule,
  /// consumed by createActions for the tick cycle period).
  std::map<std::string, ayanami::Config> recovery_configs;

  // ---- geometry / policy (configure-time) ----
  std::unique_ptr<costmap_2d::Costmap2D> occupancy;
  double resolution{0.05};
  double size_x_meters{16.0};
  double size_y_meters{16.0};
  double max_distance{3.0};
  double recentre_threshold{1.0};
  double min_obstacle_z{0.0};
  double max_obstacle_z{2.0};
  double obstacle_min_range{0.15};
  double obstacle_max_range{8.0};
  double raytrace_max_range{8.0};
  double transform_tolerance{0.1};
  bool window_centered{false};

  // ---- observation buffer + writer thread state (single literal writer) ----
  std::deque<std::shared_ptr<const PendingObservation>> observation_buffer;
  static constexpr std::size_t k_max_pending_observations = 8;
  std::vector<ResetFieldCommand> pending_resets;
  mutable std::mutex writer_mutex;
  std::condition_variable writer_cv;
  std::thread writer_thread;
  std::atomic<bool> writer_running{false};
  std::atomic<std::uint64_t> observation_submitted{0};
  std::atomic<std::uint64_t> observation_applied{0};
  std::atomic<std::uint64_t> observations_dropped{0};
  std::chrono::duration<double> field_period{0.1};
  std::chrono::steady_clock::time_point last_cycle{};
  /// @brief Robot pose cache in the field frame (transient pose-loss
  /// fallback; never (0,0,0)).
  bool has_last_robot{false};
  double last_robot_x{0.0};
  double last_robot_y{0.0};
  std::uint64_t snapshot_sequence{0};
  /// @brief Throttle for the missing-TF warning (per sensor callback);
  /// steady_clock nanoseconds (epoch 0 = never). atomic<int64_t> because
  /// the primary std::atomic<time_point> default ctor is deleted on older
  /// libstdc++ (pre-C++20).
  std::atomic<std::int64_t> tf_warn_at{0};
};

}  // namespace rosemap
}  // namespace ayanami
