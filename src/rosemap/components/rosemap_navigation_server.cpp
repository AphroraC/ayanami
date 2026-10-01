// RosemapNavigationServer main implementation.
//
// Lifecycle discipline: the base NavigationServerBase destructor first
// stops -> resets actions -> clearModules; this server additionally joins
// the resident field-writer thread.
//
// Observation discipline: the host (RosemapNavigation::setPointcloud/
// setLaserScan) hands observations straight to handleObstacle* (no global
// NavigationCallbacks input slot -- output/observation layering, mirroring
// the costmap family). handleObstacle* only converts and enqueues; the
// resident writer thread resolves stamp-time TFs, raytrace-clears + marks
// the rolling occupancy grid, recentres on the robot and rebuilds the
// signed field (two exact EDT passes) into an immutable snapshot that is
// published by an atomic shared_ptr swap.

#include "ayanami/rosemap/components/rosemap_navigation_server.hpp"

#include <ayanami/abstract/components/robot_state_controller.hpp>
#include <ayanami/interfaces/callbacks.hpp>
#include <ayanami/messages/nav_msgs.hpp>
#include <ayanami/rosemap/core/edt.hpp>
#include <ayanami/utility/bootstrap.hpp>
#include <ayanami/utility/config.hpp>

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <utility>

namespace ayanami {
namespace rosemap {

namespace {

constexpr double k_default_resolution = 0.05;
constexpr double k_tf_lookup_timeout_s = 0.0;
constexpr double k_default_field_hz = 10.0;

// Writer-side cycle-duration warning threshold: the signed EDT rebuild over
// the whole window costs a few ms; sustained overruns mean the window is
// oversized for the update rate.
constexpr double k_field_cycle_warn_ms = 50.0;

// TF chain missing warn throttling (sensor callbacks can fail per frame).
// Throttle stamps are steady_clock nanoseconds in std::atomic<int64_t>
// (epoch 0 = never warned): atomic<time_point>'s default constructor is
// deleted on older libstdc++ (pre-C++20) and breaks builds there.
constexpr double k_tf_warn_interval_s = 5.0;
bool tfWarnDue(std::atomic<std::int64_t>& last_warn_ns) {
  const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                          std::chrono::steady_clock::now().time_since_epoch())
                          .count();
  auto prev = last_warn_ns.load();
  if (now_ns - prev < static_cast<std::int64_t>(k_tf_warn_interval_s * 1e9)) {
    return false;
  }
  return last_warn_ns.compare_exchange_strong(prev, now_ns);
}

/// @brief raytraceLine functor: clear one cell to FREE_SPACE (the action
/// receives the row-major buffer index).
struct ClearCell {
  costmap_2d::Costmap2D* map;
  void operator()(unsigned int index) const {
    map->setCost(index % map->getSizeInCellsX(),
                 index / map->getSizeInCellsX(), costmap_2d::FREE_SPACE);
  }
};

/// @brief Command port implementation: hands typed write commands to the
/// sole writer (this server). Recovery modules only hold the
/// RosemapCommandPort interface and never touch the working grid.
class CommandPortImpl : public RosemapCommandPort {
 public:
  explicit CommandPortImpl(
      std::function<bool(const ResetFieldCommand&)> handler)
      : handler(std::move(handler)) {}
  bool submit(const ResetFieldCommand& command) override {
    return handler ? handler(command) : false;
  }

 private:
  std::function<bool(const ResetFieldCommand&)> handler;
};

}  // namespace

RosemapNavigationServer::RosemapNavigationServer() {
  command_port = std::make_shared<CommandPortImpl>(
      [this](const ResetFieldCommand& command) {
        // The reset command joins the writer thread's queue (single literal
        // writer); the submitter waits bounded for the applied result. With
        // no writer thread running (tests) the command executes inline.
        if (!writer_running.load()) {
          return executeResetCommand(command);
        }
        auto done = std::make_shared<std::promise<bool>>();
        auto applied = done->get_future();
        {
          std::lock_guard<std::mutex> lock(writer_mutex);
          pending_resets.push_back(command);
        }
        writer_cv.notify_all();
        return applied.wait_for(std::chrono::seconds(2)) ==
                       std::future_status::ready
                   ? applied.get()
                   : false;
      });
}

RosemapNavigationServer::~RosemapNavigationServer() { stopWriter(); }

// ---- lifecycle ----

void RosemapNavigationServer::start() {
  NavigationServerBase::start();
  startWriter();
}

void RosemapNavigationServer::stop() {
  stopWriter();
  NavigationServerBase::stop();
}

void RosemapNavigationServer::startWriter() {
  if (writer_running.exchange(true)) {
    return;
  }
  writer_thread = std::thread([this] { writerLoop(); });
}

void RosemapNavigationServer::stopWriter() {
  if (!writer_running.exchange(false)) {
    return;
  }
  writer_cv.notify_all();
  if (writer_thread.joinable()) {
    writer_thread.join();
  }
}

bool RosemapNavigationServer::waitForMapIdle(double timeout_s) const {
  if (!writer_running.load()) {
    return true;  // inline processing: nothing pending by construction
  }
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::duration<double>(timeout_s);
  while (std::chrono::steady_clock::now() < deadline) {
    if (observation_submitted.load() == observation_applied.load() &&
        observation_buffer.empty()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  return observation_submitted.load() == observation_applied.load();
}

void RosemapNavigationServer::writerLoop() {
  while (writer_running.load()) {
    std::vector<ResetFieldCommand> resets;
    {
      std::unique_lock<std::mutex> lock(writer_mutex);
      writer_cv.wait_for(lock, std::chrono::milliseconds(10), [&] {
        return !observation_buffer.empty() || !pending_resets.empty() ||
               !writer_running.load();
      });
      if (!writer_running.load()) {
        break;
      }
      resets.swap(pending_resets);
    }

    // Reset commands first: a recovery's replan depends on the cleared
    // field.
    for (const auto& reset : resets) {
      executeResetCommand(reset);
    }

    // One mark/raytrace/rebuild pass at the configured update_frequency.
    runMapCycle(/*force=*/false);
  }
}

// ---- core maps ----

void RosemapNavigationServer::configureCoreMaps(
    const std::string& config_path, ayanami::map::MapFamilyFactory::Ptr factory) {
  // The context's sole creation point = the map-family factory (core and
  // host only know the MapFamilyFactory abstract surface).
  map_family_factory = factory ? std::move(factory)
                               : std::make_shared<RosemapMapFamilyFactory>();
  // Double configureCoreMaps guard (a rebuilt context would orphan the
  // module references of already-injected modules).
  if (field_context) {
    spdlog::warn(
        "RosemapNavigationServer::configureCoreMaps called twice, ignoring; "
        "existing modules would be orphaned by context replace");
    return;
  }

  ayanami::bootstrap(config_path);
  Config nav_cfg(GlobalConfig::getConfigPath("config_navigation"));
  const Config& cfg = nav_cfg;

  resolution = cfg.param<double>("rosemap_field", "resolution",
                                 k_default_resolution);
  size_x_meters = cfg.param<double>("rosemap_field", "size_x", size_x_meters);
  size_y_meters = cfg.param<double>("rosemap_field", "size_y", size_y_meters);
  max_distance = cfg.param<double>("rosemap_field", "max_distance", max_distance);
  recentre_threshold =
      cfg.param<double>("rosemap_field", "recentre_threshold", recentre_threshold);
  min_obstacle_z = cfg.param<double>("rosemap_field", "min_obstacle_z", min_obstacle_z);
  max_obstacle_z = cfg.param<double>("rosemap_field", "max_obstacle_z", max_obstacle_z);
  obstacle_min_range =
      cfg.param<double>("rosemap_field", "obstacle_min_range", obstacle_min_range);
  obstacle_max_range =
      cfg.param<double>("rosemap_field", "obstacle_max_range", obstacle_max_range);
  raytrace_max_range =
      cfg.param<double>("rosemap_field", "raytrace_max_range", raytrace_max_range);
  transform_tolerance =
      cfg.param<double>("rosemap_field", "transform_tolerance", transform_tolerance);

  const std::string field_frame = cfg.param<std::string>(
      "rosemap_field", "field_frame",
      cfg.param<std::string>("rosemap_field", "global_frame", std::string("map")));
  const std::string robot_frame =
      nav_cfg.param<std::string>("navigation", "robot_frame", "base_link");

  const unsigned int size_cells_x =
      static_cast<unsigned int>(std::max(1.0, size_x_meters / resolution));
  const unsigned int size_cells_y =
      static_cast<unsigned int>(std::max(1.0, size_y_meters / resolution));
  occupancy = std::make_unique<costmap_2d::Costmap2D>(
      size_cells_x, size_cells_y, resolution, 0.0, 0.0,
      costmap_2d::FREE_SPACE);

  const double field_hz = cfg.param<double>("rosemap_field", "update_frequency",
                                            k_default_field_hz);
  field_period = field_hz > 0.0
                     ? std::chrono::duration<double>(1.0 / field_hz)
                     : std::chrono::duration<double>(0.0);

  field_context = std::dynamic_pointer_cast<RosemapMapContext>(
      map_family_factory->createContext());
  if (!field_context) {
    throw std::runtime_error(
        "map family factory '" + map_family_factory->familyName() +
        "' did not create a RosemapMapContext");
  }
  field_context->setFrames(field_frame, robot_frame);
  field_context->setCommandPort(command_port);
  field_context->setStaleTimeout(
      cfg.param<double>("rosemap_field", "stale_timeout", 0.0));
  field_context->setPathMargin(
      cfg.param<double>("rosemap_field", "path_margin", 0.0));

  // One-time configuration of the global RobotStateController: frame names
  // + velocity timeout are cached only here; the hot getRobotState never
  // reads Config.
  const double velocity_timeout =
      nav_cfg.param<double>("navigation", "velocity_timeout", 0.25);
  ayanami::components::RobotStateController::instance().initialize(
      field_frame, field_frame, robot_frame, velocity_timeout);

  // Initial empty snapshot: consumers see valid (obstacle-free) geometry
  // immediately, mirroring the costmap family's configure-time
  // materialization pass (the execution safetyCheck would otherwise gate on
  // a missing snapshot).
  snapshot_sequence = 0;
  rebuildAndPublish(0.0, 0.0);
  has_last_robot = false;
  window_centered = false;
}

// ---- family module injection ----

bool RosemapNavigationServer::initializePlannerModule(
    const std::string& name, const ayanami::Config& module_config,
    ayanami::abstract::AbstractPlanner::Ptr ptr) {
  if (!field_context) {
    spdlog::error(
        "RosemapNavigationServer::initializePlannerModule('{}'): field "
        "context not configured (call configureCoreMaps first)",
        name);
    return false;
  }
  auto cast = std::dynamic_pointer_cast<RosemapPlanner>(ptr);
  if (!cast) {
    spdlog::error(
        "RosemapNavigationServer::initializePlannerModule('{}'): module is "
        "not a RosemapPlanner (rosemap family interface mismatch)",
        name);
    return false;
  }
  cast->configure(ayanami::ModuleContext{name, module_config},
                  field_context->getSharedConstPtr());
  return true;
}

bool RosemapNavigationServer::initializeSmootherModule(
    const std::string& name, const ayanami::Config& module_config,
    ayanami::abstract::AbstractSmoother::Ptr ptr) {
  if (!field_context) {
    spdlog::error(
        "RosemapNavigationServer::initializeSmootherModule('{}'): field "
        "context not configured (call configureCoreMaps first)",
        name);
    return false;
  }
  auto cast = std::dynamic_pointer_cast<RosemapSmoother>(ptr);
  if (!cast) {
    spdlog::error(
        "RosemapNavigationServer::initializeSmootherModule('{}'): module is "
        "not a RosemapSmoother (rosemap family interface mismatch)",
        name);
    return false;
  }
  cast->configure(ayanami::ModuleContext{name, module_config},
                  field_context->getSharedConstPtr());
  return true;
}

bool RosemapNavigationServer::initializeControllerModule(
    const std::string& name, const ayanami::Config& module_config,
    ayanami::abstract::AbstractController::Ptr ptr) {
  if (!field_context) {
    spdlog::error(
        "RosemapNavigationServer::initializeControllerModule('{}'): field "
        "context not configured (call configureCoreMaps first)",
        name);
    return false;
  }
  // Strict rosemap family constraint: only RosemapController is accepted;
  // plain AbstractControllers are not allowed (mirrors the costmap family).
  auto cast = std::dynamic_pointer_cast<RosemapController>(ptr);
  if (!cast) {
    spdlog::error(
        "RosemapNavigationServer::initializeControllerModule('{}'): module "
        "is not a RosemapController (rosemap family interface mismatch)",
        name);
    return false;
  }
  cast->configure(ayanami::ModuleContext{name, module_config},
                  field_context->getSharedConstPtr());
  return true;
}

bool RosemapNavigationServer::initializeRecoveryModule(
    const std::string& name, const ayanami::Config& module_config,
    ayanami::abstract::AbstractRecovery::Ptr ptr) {
  if (!field_context) {
    spdlog::error(
        "RosemapNavigationServer::initializeRecoveryModule('{}'): field "
        "context not configured (call configureCoreMaps first)",
        name);
    return false;
  }
  auto cast = std::dynamic_pointer_cast<RosemapRecovery>(ptr);
  if (!cast) {
    spdlog::error(
        "RosemapNavigationServer::initializeRecoveryModule('{}'): module is "
        "not a RosemapRecovery (rosemap family interface mismatch)",
        name);
    return false;
  }
  cast->configure(ayanami::ModuleContext{name, module_config},
                  field_context->getSharedConstPtr());
  // Stash the module config so createActions can inject per-module
  // execution settings (tick cycle period).
  recovery_configs.erase(name);
  recovery_configs.emplace(name, module_config);
  return true;
}

// ---- createActions: production path = Rosemap*Action ----

void RosemapNavigationServer::createActions() {
  planner_actions.clear();
  smoother_actions.clear();
  controller_actions.clear();
  recovery_actions.clear();

  for (const auto& name : planner_modules.getLoadedNames()) {
    auto planner = planner_modules.getModule(name);
    if (!planner) {
      continue;
    }
    planner_actions[name] =
        std::make_unique<RosemapPlannerAction>(planner, field_context);
  }
  if (!planner_actions.empty() &&
      planner_actions.find(default_planner) == planner_actions.end()) {
    spdlog::error("RosemapNavigationServer::createActions: default planner "
                  "'{}' not loaded",
                  default_planner);
  }

  for (const auto& name : smoother_modules.getLoadedNames()) {
    auto smoother = smoother_modules.getModule(name);
    if (!smoother) {
      continue;
    }
    smoother_actions[name] =
        std::make_unique<RosemapSmootherAction>(smoother, field_context);
  }
  if (!smoother_actions.empty() &&
      smoother_actions.find(default_smoother) == smoother_actions.end()) {
    spdlog::error("RosemapNavigationServer::createActions: default smoother "
                  "'{}' not loaded",
                  default_smoother);
  }

  for (const auto& name : controller_modules.getLoadedNames()) {
    auto controller = controller_modules.getModule(name);
    if (!controller) {
      continue;
    }
    controller_actions[name] = std::make_unique<RosemapControllerAction>(
        controller, field_context,
        [this](ayanami::components::ControllerExecution::Ptr exec) {
          configureControllerExecution(exec);
        });
  }
  if (!controller_actions.empty() &&
      controller_actions.find(default_controller) == controller_actions.end()) {
    spdlog::error("RosemapNavigationServer::createActions: default "
                  "controller '{}' not loaded",
                  default_controller);
  }

  for (const auto& name : recovery_modules.getLoadedNames()) {
    auto recovery = recovery_modules.getModule(name);
    if (!recovery) {
      continue;
    }
    double cycle_period_s = 0.01;
    const auto config_iter = recovery_configs.find(name);
    if (config_iter != recovery_configs.end()) {
      cycle_period_s = config_iter->second.param<double>(
          "recovery", "sim_cycle_dt", cycle_period_s);
    }
    auto action = std::make_unique<RosemapRecoveryAction>(
        recovery, field_context,
        [this, cycle_period_s](
            ayanami::components::RecoveryExecution::Ptr exec) {
          configureRecoveryExecution(exec);
          if (cycle_period_s > 0.0) {
            exec->setCyclePeriod(
                std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::duration<double>(cycle_period_s)));
          }
        });
    recovery_actions[name] = std::move(action);
  }
}

// ---- observation ingestion ----

void RosemapNavigationServer::handleObstaclePointCloud(
    const ayanami::sensor_msgs::PointCloud2& cloud) {
  if (!field_context || !occupancy) {
    spdlog::warn(
        "RosemapNavigationServer::handleObstaclePointCloud: field not "
        "configured (call configureCoreMaps first)");
    return;
  }
  PendingObservation observation;
  observation.frame_id = cloud.header.frame_id;
  observation.stamp = cloud.header.stamp;
  observation.points.reserve(cloud.cloud.points.size());
  for (const auto& p : cloud.cloud.points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y)) {
      continue;
    }
    // z is KEPT here: the field's z band filters hits in the map cycle.
    observation.points.emplace_back(static_cast<double>(p.x),
                                    static_cast<double>(p.y),
                                    static_cast<double>(p.z));
  }
  submitObservation(std::move(observation));
}

void RosemapNavigationServer::handleObstacleLaserScan(
    const ayanami::sensor_msgs::LaserScan& scan) {
  if (!field_context || !occupancy) {
    spdlog::warn(
        "RosemapNavigationServer::handleObstacleLaserScan: field not "
        "configured (call configureCoreMaps first)");
    return;
  }
  if (scan.ranges.empty()) {
    return;
  }
  PendingObservation observation;
  observation.frame_id = scan.header.frame_id;
  observation.stamp = scan.header.stamp;
  observation.points.reserve(scan.ranges.size());
  observation.clearing_points.reserve(scan.ranges.size());
  for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
    const double range = static_cast<double>(scan.ranges[i]);
    const double angle = scan.angle_min + i * scan.angle_increment;
    // Ray endpoints clamped to raytrace_max_range (a non-finite range rays
    // out to the max: "nothing up to here is blocked"); rays below
    // raytrace_min_range are skipped in the map cycle.
    const double clear_range = std::isfinite(range)
                                   ? std::clamp(range, 0.0, raytrace_max_range)
                                   : raytrace_max_range;
    observation.clearing_points.emplace_back(clear_range * std::cos(angle),
                                             clear_range * std::sin(angle),
                                             0.0);
    if (range >= scan.range_min && range <= scan.range_max) {
      observation.points.emplace_back(range * std::cos(angle),
                                      range * std::sin(angle), 0.0);
    }
  }
  submitObservation(std::move(observation));
}

void RosemapNavigationServer::submitObservation(
    PendingObservation&& observation) {
  observation_submitted.fetch_add(1);
  auto frame =
      std::make_shared<const PendingObservation>(std::move(observation));
  {
    std::lock_guard<std::mutex> lock(writer_mutex);
    if (observation_buffer.size() >= k_max_pending_observations) {
      // Sustained processing stall: drop the OLDEST frame.
      observation_buffer.pop_front();
      observations_dropped.fetch_add(1);
    }
    observation_buffer.push_back(std::move(frame));
  }
  if (!writer_running.load()) {
    // No writer thread (direct-construction tests): the caller's thread
    // runs the SAME cycle pipeline synchronously per submission -- one
    // implementation, two drivers.
    runMapCycle(/*force=*/true);
    observation_applied.store(observation_submitted.load());
    return;
  }
  writer_cv.notify_all();
}

// ---- map cycle ----

void RosemapNavigationServer::runMapCycleNow(bool force) {
  runMapCycle(force);
  observation_applied.store(observation_submitted.load());
}

void RosemapNavigationServer::runMapCycle(bool force) {
  if (!field_context || !occupancy) {
    return;
  }
  const auto now = std::chrono::steady_clock::now();
  if (!force) {
    if (field_period.count() <= 0.0) {
      return;
    }
    if (now - last_cycle < field_period) {
      return;
    }
    last_cycle = now;
  }

  // ---- take: drain the buffer under the ingest mutex ----
  std::vector<std::shared_ptr<const PendingObservation>> batch;
  {
    std::lock_guard<std::mutex> lock(writer_mutex);
    batch.assign(observation_buffer.begin(), observation_buffer.end());
    observation_buffer.clear();
  }

  // ---- resolve: stamp-time transforms + pose, no locks held ----
  std::vector<ResolvedFrame> resolved_batch(batch.size());
  for (std::size_t i = 0; i < batch.size(); ++i) {
    resolved_batch[i].frame = batch[i];
    resolved_batch[i].transform_ok = resolveObservationTransform(
        *batch[i], resolved_batch[i].t_field_sensor,
        resolved_batch[i].stamp_ok);
  }
  double robot_x = 0.0;
  double robot_y = 0.0;
  const bool have_pose = resolveRobotPose(robot_x, robot_y);
  bool window_moved = false;
  if (have_pose) {
    window_moved = recentreWindow(robot_x, robot_y);
  }

  // ---- apply: raytrace-clear + mark (single writer, no lock: the working
  // grid is only ever touched on this thread) ----
  const auto cycle_start = std::chrono::steady_clock::now();
  bool any_applied = false;
  for (auto& resolved : resolved_batch) {
    if (!resolved.transform_ok) {
      continue;
    }
    const auto& observation = *resolved.frame;
    const Eigen::Vector3d sensor_origin =
        resolved.t_field_sensor.translation();
    auto in_band = [this](double z) {
      return z >= min_obstacle_z && z <= max_obstacle_z;
    };
    if (observation.clearing_points.empty()) {
      // Pointcloud path: raytrace to every filtered hit (clear along +
      // mark the endpoint; a latest-time fallback frame clears but never
      // marks -- the omega x dt skew would bake ghost marks).
      for (const auto& p : observation.points) {
        if (!in_band(p.z())) {
          continue;
        }
        const Eigen::Vector3d endpoint = resolved.t_field_sensor * p;
        const double range =
            std::hypot(endpoint.x() - sensor_origin.x(),
                       endpoint.y() - sensor_origin.y());
        if (range < obstacle_min_range || range > obstacle_max_range) {
          continue;
        }
        applyRay(sensor_origin, endpoint, resolved.stamp_ok);
        any_applied = true;
      }
    } else {
      // Laser path: clear along the clamped rays, then mark the in-band
      // hits.
      for (const auto& p : observation.clearing_points) {
        const Eigen::Vector3d endpoint = resolved.t_field_sensor * p;
        applyRay(sensor_origin, endpoint, /*endpoint_is_hit=*/false);
        any_applied = true;
      }
      for (const auto& p : observation.points) {
        const Eigen::Vector3d endpoint = resolved.t_field_sensor * p;
        const double range = std::hypot(endpoint.x() - sensor_origin.x(),
                                        endpoint.y() - sensor_origin.y());
        if (range < obstacle_min_range || range > obstacle_max_range) {
          continue;
        }
        applyRay(sensor_origin, endpoint, resolved.stamp_ok);
        any_applied = true;
      }
    }
  }

  // Republish whenever the field content or the window geometry changed --
  // an observation-free cycle that only slides the window still produces a
  // NEW snapshot (the old one keeps the stale origin and would strand
  // consumers on the previous window).
  if (any_applied || window_moved || !resolved_batch.empty()) {
    rebuildAndPublish(have_pose ? robot_x : last_robot_x,
                      have_pose ? robot_y : last_robot_y);
  }

  const double cycle_ms =
      std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - cycle_start)
          .count();
  if (cycle_ms > k_field_cycle_warn_ms) {
    spdlog::warn(
        "RosemapNavigationServer: field cycle ({} frames) took {:.1f} ms "
        "(threshold {:.0f} ms); the window may be oversized for the update "
        "rate",
        resolved_batch.size(),
        cycle_ms,
        k_field_cycle_warn_ms);
  }
}

bool RosemapNavigationServer::resolveObservationTransform(
    const PendingObservation& observation, Eigen::Isometry3d& t_field_sensor,
    bool& stamp_ok) {
  auto& robot_state_controller =
      ayanami::components::RobotStateController::instance();
  const bool have_stamp = !observation.stamp.isLatest();
  // Marking-allowed semantics (costmap family): a frame published WITHOUT a
  // stamp is latest-time by construction -- marking stays allowed. Only a
  // frame that CARRIED a stamp but degraded to the latest transform is
  // restricted to clearing.
  stamp_ok = !have_stamp;

  if (have_stamp) {
    // Bounded POLL (never a blocking wait on the ros clock): the writer is
    // allowed to wait up to transform_tolerance for the stamp-time
    // transform, polling with zero-timeout lookups.
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::duration<double>(transform_tolerance);
    while (true) {
      if (robot_state_controller.lookupTransform(
              field_context->fieldFrame(), observation.frame_id,
              observation.stamp, t_field_sensor, k_tf_lookup_timeout_s)) {
        stamp_ok = true;
        return true;
      }
      if (std::chrono::steady_clock::now() >= deadline) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }
  if (!robot_state_controller.lookupTransform(
          field_context->fieldFrame(), observation.frame_id,
          ayanami::Timestamp::latest(), t_field_sensor,
          k_tf_lookup_timeout_s)) {
    if (tfWarnDue(tf_warn_at)) {
      spdlog::warn(
          "RosemapNavigationServer: no transform field({}) <- {} "
          "(stamp={:f}s); obstacle stamping paused. Check the TF producers",
          field_context->fieldFrame(),
          observation.frame_id,
          observation.stamp.toSeconds());
    }
    return false;
  }
  return true;
}

bool RosemapNavigationServer::resolveRobotPose(double& robot_x,
                                               double& robot_y) {
  auto& robot_state_controller =
      ayanami::components::RobotStateController::instance();
  Eigen::Isometry3d t_robot = Eigen::Isometry3d::Identity();
  if (robot_state_controller.lookupTransform(
          field_context->fieldFrame(), field_context->robotFrame(),
          ayanami::Timestamp::latest(), t_robot, k_tf_lookup_timeout_s)) {
    robot_x = t_robot.translation().x();
    robot_y = t_robot.translation().y();
    has_last_robot = true;
    last_robot_x = robot_x;
    last_robot_y = robot_y;
    return true;
  }
  if (has_last_robot) {
    // Transient pose loss: roll/consume with the last successful pose --
    // never (0,0,0).
    robot_x = last_robot_x;
    robot_y = last_robot_y;
    return true;
  }
  return false;
}

void RosemapNavigationServer::applyRay(const Eigen::Vector3d& origin,
                                       const Eigen::Vector3d& endpoint,
                                       bool endpoint_is_hit) {
  // Rays to points beyond the window still clear their in-window prefix.
  // The clip must preserve the ray DIRECTION: component-wise clamping of
  // the endpoint would bend the ray (a max-range clear ray would then sweep
  // a different sector than the original beam and miss marks along it), so
  // the endpoint is clipped via ray/AABB intersection instead.
  unsigned int x0 = 0;
  unsigned int y0 = 0;
  const bool origin_inside =
      occupancy->worldToMap(origin.x(), origin.y(), x0, y0);
  const double min_x = occupancy->getOriginX();
  const double min_y = occupancy->getOriginY();
  const double max_x = min_x + occupancy->getSizeInCellsX() * resolution;
  const double max_y = min_y + occupancy->getSizeInCellsY() * resolution;
  const Eigen::Vector3d direction = endpoint - origin;
  double t_exit = std::numeric_limits<double>::infinity();
  bool parallel_outside = false;
  for (int axis = 0; axis < 2; ++axis) {
    const double o = axis == 0 ? origin.x() : origin.y();
    const double d = axis == 0 ? direction.x() : direction.y();
    const double lo = axis == 0 ? min_x : min_y;
    const double hi = axis == 0 ? max_x : max_y;
    if (std::abs(d) < 1e-12) {
      if (o < lo || o > hi) {
        parallel_outside = true;
      }
      continue;
    }
    double t = (d > 0.0 ? hi - o : lo - o) / d;
    if (t < t_exit) {
      t_exit = t;
    }
  }
  if (parallel_outside || !(t_exit >= 0.0) ||
      !std::isfinite(t_exit)) {
    // The ray's in-window prefix is empty.
    return;
  }
  // Step the exit point a hair INSIDE the box: an exit exactly on the
  // boundary maps to the cell past the edge and would skip the raytrace.
  t_exit = std::max(0.0, t_exit - 1e-6);
  if (origin_inside) {
    const Eigen::Vector3d clipped = origin + t_exit * direction;
    unsigned int x1 = 0;
    unsigned int y1 = 0;
    if (occupancy->worldToMap(clipped.x(), clipped.y(), x1, y1)) {
      occupancy->raytraceLine(ClearCell{occupancy.get()}, x0, y0, x1, y1);
    }
  }
  if (endpoint_is_hit) {
    unsigned int hx = 0;
    unsigned int hy = 0;
    if (occupancy->worldToMap(endpoint.x(), endpoint.y(), hx, hy)) {
      occupancy->setCost(hx, hy, costmap_2d::LETHAL_OBSTACLE);
    }
  }
}

void RosemapNavigationServer::rebuildAndPublish(double robot_x, double robot_y) {
  const unsigned int width = occupancy->getSizeInCellsX();
  const unsigned int height = occupancy->getSizeInCellsY();
  const std::size_t cells =
      static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
  std::vector<unsigned char> occupied(cells, 0);
  const unsigned char* map = occupancy->getCharMap();
  for (std::size_t i = 0; i < cells; ++i) {
    occupied[i] = map[i] == costmap_2d::LETHAL_OBSTACLE ? 1 : 0;
  }
  auto snapshot = std::make_shared<RosemapSnapshot>();
  snapshot->frame = field_context->fieldFrame();
  snapshot->resolution = resolution;
  snapshot->size_x = static_cast<int>(width);
  snapshot->size_y = static_cast<int>(height);
  snapshot->origin_x = occupancy->getOriginX();
  snapshot->origin_y = occupancy->getOriginY();
  snapshot->max_distance = max_distance;
  snapshot->sequence = ++snapshot_sequence;
  snapshot->received = std::chrono::steady_clock::now();
  snapshot->occupied = std::move(occupied);
  signedEdt2d(snapshot->occupied, snapshot->size_x, snapshot->size_y,
              resolution, max_distance, snapshot->distance);
  (void)robot_x;
  (void)robot_y;
  emitRosemapField(snapshot);
  field_context->publishSnapshot(std::move(snapshot));
}

void RosemapNavigationServer::emitRosemapField(
    const std::shared_ptr<const RosemapSnapshot>& snapshot) {
  ayanami::nav_msgs::OccupancyGrid grid;
  grid.header.frame_id = snapshot->frame;
  grid.info.resolution = snapshot->resolution;
  grid.info.width = static_cast<std::uint32_t>(snapshot->size_x);
  grid.info.height = static_cast<std::uint32_t>(snapshot->size_y);
  Eigen::Isometry3d origin_iso = Eigen::Isometry3d::Identity();
  origin_iso.translation() =
      Eigen::Vector3d(snapshot->origin_x, snapshot->origin_y, 0.0);
  grid.info.origin = ayanami::geometry_msgs::Pose::fromIsometry(origin_iso);
  grid.data.resize(snapshot->occupied.size());
  for (std::size_t i = 0; i < snapshot->occupied.size(); ++i) {
    grid.data[i] = snapshot->occupied[i] ? static_cast<std::int8_t>(100)
                                         : static_cast<std::int8_t>(0);
  }
  ayanami::interfaces::NavigationCallbacks::on_rosemap_field(grid);
}

bool RosemapNavigationServer::recentreWindow(double robot_x, double robot_y) {
  const double origin_x = occupancy->getOriginX();
  const double origin_y = occupancy->getOriginY();
  const double center_x = origin_x + size_x_meters * 0.5;
  const double center_y = origin_y + size_y_meters * 0.5;
  if (window_centered &&
      std::hypot(robot_x - center_x, robot_y - center_y) <=
          recentre_threshold) {
    return false;
  }
  const double new_origin_x = robot_x - size_x_meters * 0.5;
  const double new_origin_y = robot_y - size_y_meters * 0.5;
  occupancy->updateOrigin(new_origin_x, new_origin_y);
  window_centered = true;
  return true;
}

// ---- write commands ----

bool RosemapNavigationServer::executeResetCommand(
    const ResetFieldCommand& command) {
  if (!command.enabled) {
    return true;
  }
  if (!field_context || !occupancy) {
    spdlog::warn(
        "RosemapNavigationServer::executeResetCommand: field not configured");
    return false;
  }
  occupancy->resetMap();
  double robot_x = 0.0;
  double robot_y = 0.0;
  resolveRobotPose(robot_x, robot_y);
  rebuildAndPublish(robot_x, robot_y);
  return true;
}

}  // namespace rosemap
}  // namespace ayanami
