#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <future>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Geometry>

#include <costmap_server/costmap_wrapper.hpp>

#include "ayanami/abstract/components/navigation_server_base.hpp"
#include "ayanami/costmap/components/costmap_action.hpp"
#include "ayanami/costmap/components/costmap_map_context.hpp"
#include "ayanami/map/map_family_factory.hpp"
#include "ayanami/costmap/core/costmap_command.hpp"
#include "ayanami/costmap/core/costmap_layer_plugin.hpp"
#include "ayanami/costmap/core/costmap_observation.hpp"
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

// ObservationSourceConfig moved to costmap/core/costmap_observation.hpp:
// the filtering parameters are owned by the CONSUMING obstacle layer
// plugin (and by the legacy costmap_local source blocks on the compat
// path), not by the server.

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
 *   global/local LayeredCostmap pair in DIFFERENT frames (nav2 convention):
 *   global = StaticLayer + (optional SimpleObstacleLayer) + InflationLayer
 *   in global_costmap_frame (typically "map"); the global obstacle layer is
 *   enabled by costmap_global.observation_sources and receives the SAME
 *   observations the local map consumes, so replanning and IsPathValid see
 *   real obstacles (nav2 global_costmap = static + obstacle + inflation);
 *   local = rolling SimpleObstacleLayer+InflationLayer in
 *   local_costmap_frame (typically "odom" -- the odometry frame keeps the
 *   rolling dynamic window immune to localization jumps).
 *   Every change (obstacle stamping / command-port clear) is applied by this
 *   server as the sole writer via updateMap.
 * - Obstacle observations do not go through global callback slots: the host
 *   (CostmapNavigation) hands setPointcloud/setLaserScan straight to the
 *   common handleObstacle* methods. handleObstacle* only CONVERTS and
 *   enqueues into the map-writer's latest-wins mailbox (O(enqueue) on the
 *   caller's thread); the resident map-writer thread performs the TF
 *   lookup (bounded wait up to costmap_local.transform_tolerance, polled
 *   inside the writer so no other thread ever blocks), footprint clearing,
 *   rasterization, updateMap, the rolling-window updates and the clear
 *   commands -- the single literal writer. When the writer thread is not
 *   running (direct-construction tests), the same processing path executes
 *   inline on the caller's thread (legacy synchronous semantics).
 * - All TF queries go through the RobotStateController singleton (the
 *   framework-wide transform surface); its frame names + velocity_timeout
 *   are configured once in configureCoreMaps. The controller execution reads
 *   the latest TF pose + latest velocity through it (zero velocity when
 *   stale).
 *
 * Sensor liveness: every source with expected_update_rate > 0 is checked by
 * the writer loop; a source silent for longer than its rate marks the map
 * not-current (throttled warning), which surfaces downstream as
 * MAP_STALE -> BT recovery. Data resuming re-marks the map current.
 *
 * Dynamic-mark clearing applies wherever a SimpleObstacleLayer exists: on
 * the global map a clear retires the dynamic marks and the StaticLayer
 * restores the underlying base map (nav2 clear_entirely semantics).
 */
class CostmapNavigationServer
    : public ayanami::components::NavigationServerBase {
 public:
  using Ptr = std::shared_ptr<CostmapNavigationServer>;

  explicit CostmapNavigationServer();
  ~CostmapNavigationServer() override;

  // ---- lifecycle: start/stop also drive the resident map-writer thread ----
  void start() override;
  void stop() override;

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

  /// @brief Periodic rolling update (driven by the map-writer loop when it
  /// runs; direct callers get the legacy inline roll when it does not): for
  /// maps with rolling_window enabled, center the window on the latest
  /// map-frame robot pose and recompute (when no window is dirty, updateMap
  /// exits early right after updateOrigin, so the cost is negligible). When
  /// the scan stream stops, the window still follows the robot -- otherwise
  /// the controller's forward trajectories gradually "leave the map" and the
  /// robot gets stuck at the edge of a frozen window with an undetectable
  /// current flag.
  /// Pose resolution happens before taking the map lock.
  void updateRollingWindows();

  /// @brief Dump both master grids as PGM + metadata sidecar into the dump
  /// directory (costmap.dump_frequency Hz on the writer thread; 0 disables,
  /// production runs at 0.1 Hz into the logging dir). Field diagnostics:
  /// the dumped PGM shows exactly the master costs (free=white, inflation
  /// gradient, lethal=black) and the sidecar carries resolution/origin.
  /// Runs as a short-lock snapshot copy + out-of-lock file I/O.
  void dumpCostmaps();

  /// @brief Block until every observation submitted so far has been applied
  /// by the map writer (test/observability seam; a no-op when the writer
  /// thread is not running because processing is then inline).
  /// @return false on timeout.
  bool waitForMapIdle(double timeout_s) const;

  /// @brief Loaded observation-source configuration (configure-time parse of
  /// costmap_local.observation_sources + per-source child blocks).
  const std::vector<ObservationSourceConfig>& observationSources() const {
    return observation_sources;
  }

  /// @brief Total observation frames dropped by the bounded buffer.
  std::uint64_t observationsDropped() const {
    return observations_dropped.load();
  }


 private:
  /// @brief One buffered observation in the writer's latest-wins mailbox:
  /// converted to sensor-frame points (z KEPT for the height filter) plus
  /// the source's runtime configuration.
  struct PendingObservation {
    const ObservationSourceConfig* source{nullptr};
    std::string frame_id;
    ayanami::Timestamp stamp;
    std::vector<Eigen::Vector3d> points;
    std::vector<Eigen::Vector3d> clearing_points;
    /// @brief Arrival time (steady clock): the observation keep-time window
    /// ages frames from here.
    std::chrono::steady_clock::time_point received{std::chrono::steady_clock::now()};
  };

  /// @brief One cycle of the map pipeline for one map (nav2
  /// update_frequency decoupling): take every buffered observation this map
  /// has not consumed yet (keep-time window), resolve each frame's
  /// stamp-time transform OUTSIDE the write lock, apply them to the
  /// consuming layers, then run ONE updateMap for the whole batch. Sensor
  /// rate therefore no longer multiplies the mark/raytrace/inflation work:
  /// N queued frames cost one rebuild. Runs at the map's configured
  /// update_frequency (costmap_local/global.update_frequency) and subsumes
  /// the rolling-window recenter for the writer path.
  void runMapCycle(bool is_local, bool force = false);

  /// @brief Whether the periodic global map cycle can do real work: true
  /// when the global map consumes observations (dynamic obstacle layer) or
  /// rolls. A static global (static+inflation, non-rolling) changes
  /// exclusively through clear commands, so its periodic cycle is skipped.
  bool globalMapNeedsCycle() const;

  /// @brief Generation-gated per-layer artifact emission: emits
  /// on_costmap_global_{static,inflation} (global) or
  /// on_costmap_local_{obstacle,inflation} (local) whenever the map's master
  /// generation
  /// changed since the last emission. NO rate parameters anywhere: the sole
  /// writer emits unconditionally on every real content update, and an
  /// unchanged map emits nothing. Called on the writer thread after each
  /// map cycle / clear command, and once from configureCoreMaps (a static
  /// global may never cycle again -- the initial emission is the only one).
  void emitMapLayers(bool is_local);

  /// @brief Static-zone mark filter (costmap_local.filter_static_marks):
  /// local-map cloud hits inside a globally blocked cell are not marked.
  bool filter_static_marks{true};

  /// @brief Stamp-time sensor transform for one observation into one map's
  /// frame (no locks held). Bounded wait up to transform_tolerance. A
  /// stamped frame whose stamp-time transform never arrives is DROPPED
  /// whole (marks preserved, map stale-but-stable); a frame published
  /// without a stamp resolves at latest time. Returns false when the frame
  /// is dropped or the latest transform is unavailable.
  bool resolveObservationTransform(const PendingObservation& observation,
                                   const std::string& map_frame,
                                   bool is_local_map,
                                   Eigen::Isometry3d& t_frame_sensor,
                                   double* tf_wait_ms);

  /// @brief Latest robot pose in the map's frame, with the local map's
  /// rolling pose-cache fallback (never (0,0,0); see last_local_robot).
  bool resolveRobotPoseInMapFrame(const std::string& map_frame,
                                  bool is_local_map, double& robot_x,
                                  double& robot_y, double& robot_yaw);
  /// @brief A clear command queued for the writer thread, with the promise
  /// the submitter waits on (bounded).
  struct PendingClear {
    ClearCostmapCommand command;
    std::shared_ptr<std::promise<bool>> done;
  };

  /// @brief Layer-emission bookkeeping: the master generation each map's
  /// artifacts were last emitted at (atomic: the inline no-writer test path
  /// runs cycles on arbitrary caller threads; production traffic is
  /// single-writer).
  std::atomic<std::uint64_t> emitted_global_generation{0};
  std::atomic<std::uint64_t> emitted_local_generation{0};
  std::atomic<bool> global_layers_ever_emitted{false};
  std::atomic<bool> local_layers_ever_emitted{false};

  /// @brief Enqueue an observation into the writer's bounded FIFO, or, when
  /// the writer thread is not running, process inline on the caller's
  /// thread.
  void submitObservation(PendingObservation&& observation);
  /// @brief Shared stamping body (writer thread / inline): bounded-wait TF at
  /// stamp, source-driven filtering, footprint clearing, raytrace + mark +
  /// updateMap under one write lock.
  /// @param tf_wait_ms Optional out-param: time spent waiting for the
  ///        stamp-time transform (the designed bounded wait, distinct from
  ///        rasterization work).
  /// @param stamp_fallback Optional out-param: true when the stamp-time TF
  ///        never arrived and the pass degraded to latest-time semantics.
  /// @brief The stamp-time TF lookup with a bounded poll (up to
  /// transform_tolerance, ~5ms granularity) before the latest fallback.
  /// @param wait_seconds Optional out-param: how long the poll waited for
  ///        the stamp-time transform to arrive (0 when the first lookup hit).
  bool lookupWithBoundedWait(const std::string& target_frame,
                             const std::string& source_frame,
                             const ayanami::Timestamp& stamp,
                             Eigen::Isometry3d& transform,
                             double* wait_seconds = nullptr) const;
  /// @brief Writer-thread bookkeeping: per-source liveness check (silent
  /// source -> map not-current, throttled warning).
  void checkSourceLiveness();
  /// @brief The rolling-window update body (writer loop / direct callers).
  void rollWindows();
  /// @brief First configured source of the given kind (nullptr when none).
  const ObservationSourceConfig* findSource(ObservationSourceConfig::Kind kind) const;
  void startWriter();
  void stopWriter();
  void writerLoop();

  /// @brief Stamp one buffered observation into ONE map (frame transform,
  /// layer-plugin dispatch, updateMap). Each target map resolves its own
  /// stamp-time TF and robot pose, so the global and the local map consume
  /// the same observation independently.
  /// @param is_local_map enables the rolling-window last-pose fallback and
  ///        the last_local_robot_* cache (global stamping is best-effort).
  /// @param tf_wait_ms/stamp_fallback optional out-params (local path only).
  /// @return false when the map could not be stamped (TF unavailable).
  /// @brief Load one map's layer plugin list (section "layers": ordered
  /// layer entries, list order = composition order). Each entry is either
  /// an INLINE layer name (so_name + parameters in
  /// <section>.<name> of the same config -- the production spelling) or a
  /// module json path (file spelling, test compat). The key is REQUIRED;
  /// throws on absence or on a failed load.
  void loadLayerPlugins(
      const std::string& section, const ayanami::Config& cfg,
      const std::string& config_dir,
      const costmap_2d::LayeredCostmap::Ptr& layered_costmap,
      ayanami::components::ModuleManager<
          ayanami::costmap::CostmapLayerPlugin>& manager,
      std::vector<ayanami::costmap::CostmapLayerPlugin::Ptr>& plugins);

  /// @brief Transform-and-stamp entry (deleted: the observation pipeline is
  /// runMapCycle -- inline callers force the same cycle the writer uses).
  void markObstaclesInFrame(const std::string& frame_id,
                            const ayanami::Timestamp& stamp,
                            const std::vector<Eigen::Vector3d>& points,
                            const std::vector<Eigen::Vector3d>& clearing_points = {}) = delete;

  CostmapMapContext::Ptr costmap_context;
  CostmapCommandPort::Ptr command_port;
  ayanami::map::MapFamilyFactory::Ptr map_family_factory;
  /// @brief Layer plugin family: one manager + one loaded list per map
  /// (the same layer json may be listed by both maps; each instance then
  /// owns its own layer object and parameters).
  ayanami::components::ModuleManager<ayanami::costmap::CostmapLayerPlugin>
      global_layer_modules;
  ayanami::components::ModuleManager<ayanami::costmap::CostmapLayerPlugin>
      local_layer_modules;
  std::vector<ayanami::costmap::CostmapLayerPlugin::Ptr> global_layer_plugins;
  std::vector<ayanami::costmap::CostmapLayerPlugin::Ptr> local_layer_plugins;
  /// @brief Per-recovery module configs (stashed by initializeRecoveryModule,
  /// consumed by createActions to inject per-module execution settings such
  /// as the tick cycle period).
  std::map<std::string, ayanami::Config> recovery_configs;

  // ---- observation sources + writer thread state (single literal writer) ----
  std::vector<ObservationSourceConfig> observation_sources;
  /// @brief Observation FIFO (nav2 observation_keep_time analogue): frames
  /// are drained and processed one by one with their OWN stamp-time TF, so
  /// no marking coverage is lost to a processing stall -- unlike a
  /// latest-wins slot, where every dropped frame silently removed that
  /// frame's sector from the map. The cap bounds latency: the oldest frame
  /// is dropped only after k_max_pending_observations newer ones pile up
  /// (a sustained processing stall).
  /// @brief Observation buffer shared by both map cycles (nav2
  /// ObservationBuffer role): frames enter once and each map consumes at
  /// its own update_frequency via a cursor; storage frees when both
  /// cursors pass. The cap bounds latency: the oldest frame is dropped
  /// only after k_max_pending_observations newer ones pile up.
  std::deque<std::shared_ptr<const PendingObservation>> observation_buffer;
  std::size_t local_cursor{0};
  std::size_t global_cursor{0};
  /// @brief Frames older than this are dropped at consumption time
  /// (costmap_local.observation_keep_time; <=0 keeps everything).
  double observation_keep_time{0.5};
  /// @brief False when no global layer plugin consumes any observation kind:
  /// the global cursor then tracks the buffer tail so the shared storage
  /// still compacts.
  bool global_consumes_observations{false};
  static constexpr std::size_t k_max_pending_observations = 8;
  std::vector<PendingClear> pending_clears;
  mutable std::mutex writer_mutex;
  std::condition_variable writer_cv;
  std::thread writer_thread;
  std::atomic<bool> writer_running{false};
  /// @brief Submitted-vs-applied observation counters (waitForMapIdle).
  std::atomic<std::uint64_t> observation_submitted{0};
  std::atomic<std::uint64_t> observation_applied{0};
  /// @brief Frames dropped by the bounded buffer (a sustained processing
  /// stall makes the writer unable to keep up; surfaced via the throttled
  /// warning below and the counter getter for tests/monitoring).
  std::atomic<std::uint64_t> observations_dropped{0};
  /// @brief Bounded wait for the stamp-time TF (costmap_local.transform_tolerance).
  double transform_tolerance{0.1};
  /// @brief Per-source last-observation timestamps (writer thread only),
  /// index-aligned with observation_sources.
  std::vector<std::chrono::steady_clock::time_point> source_last_seen;
  /// @brief Throttle state for the stale-source warnings (per source).
  std::vector<std::chrono::steady_clock::time_point> source_stale_warn_at;
  /// @brief Throttle for the map-update-duration warning.
  std::chrono::steady_clock::time_point last_map_update_warn_at{};
  /// @brief Throttle for the observation-buffer-overflow warning.
  std::chrono::steady_clock::time_point last_drop_warn_at{};
  // ---- periodic costmap dump (costmap.dump_frequency Hz; 0 = disabled) ----
  std::chrono::duration<double> dump_period{0.0};
  std::chrono::steady_clock::time_point last_dump{};
  /// @brief Directory the PGM dumps land in (logging section's logging_dir,
  /// same value the file logger uses; relative resolves against cwd).
  std::string dump_dir{"logs"};
  /// @brief Throttle for the outside-window dropped-points warning.
  std::chrono::steady_clock::time_point last_outside_window_warn_at{};

  // Throttling for missing TF chain warnings (every sensor callback can
  // fail; warn at fixed intervals only, avoiding flooding at 5.5Hz laser
  // per frame). The map-frame sensor chain must lack TF for longer than
  // the interval before warning again.
  // steady_clock nanoseconds (epoch 0 = never warned); atomic<int64_t>
  // rather than atomic<time_point>, whose default constructor is deleted
  // on older libstdc++ (pre-C++20) and breaks builds there.
  std::atomic<std::int64_t> map_tf_warn_at{0};
  std::atomic<std::int64_t> robot_diag_at{0};
  // Robot pose cache for the LOCAL map frame (the only rolling/stamped
  // map): on a transient pose query failure, roll the window with the last
  // successful value and never fall back to (0,0,0) -- otherwise the
  // rolling costmap window gets anchored back to the frame origin and, once
  // the robot walks out of the window, the controller has no path
  // (stuttering in place).
  bool has_last_local_robot{false};
  double last_local_robot_x{0.0}, last_local_robot_y{0.0};
  double last_local_robot_yaw{0.0};
  // (composition and observation policy live entirely in the layer plugins;
  // the server holds no layer pointers and no per-map stamping state)
 // Decoupled map update cycle periods (update_frequency; rolling + observation
 // application + rebuild run at this cadence, independent of sensor rate)
  std::chrono::duration<double> local_roll_period{0.1};   // 10Hz default
  std::chrono::duration<double> global_roll_period{0.2};  // 5Hz default
  std::chrono::steady_clock::time_point last_local_cycle{};
  std::chrono::steady_clock::time_point last_global_cycle{};
 // Map-stamp scan-time query stats (ok/fallback to latest) and throttled diagnostic timestamps
  std::atomic<std::uint64_t> stamp_query_ok{0};
  std::atomic<std::uint64_t> stamp_query_fallback{0};
  std::chrono::steady_clock::time_point last_stamp_diag_at{};
};

}  // namespace costmap
}  // namespace ayanami
