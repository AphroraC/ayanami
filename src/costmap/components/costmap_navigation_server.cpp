// CostmapNavigationServer main implementation.
//
// Lifecycle discipline (/): the base NavigationServerBase destructor first
// stops -> resets actions -> clearPlugins.
//
// Obstacle observation discipline: no global NavigationCallbacks input slot
// is registered anymore (output/observation layering); the host
// (CostmapNavigation::setPointcloud/setLaserScan) hands observations
// straight to handleObstacle*.
//
// InflationLayer::updateCosts only applies `cost > old_cost` overwrites, so
// it never de-inflates automatically once an obstacle disappears. Before
// each dynamic update, handleObstacle* first clearAreas the previous frame
// + current frame observation window (expanded by the inflation radius),
// clearing old obstacle marks and old inflation bands to FREE in the master
// grid, then markObstacles + updateMap lets the InflationLayer fully
// recompute within its recompute window, leaving no stale bands.

#include "ayanami/costmap/components/costmap_map_family_factory.hpp"
#include "ayanami/costmap/components/costmap_navigation_server.hpp"

#include <costmap_2d_layers/inflation_layer.hpp>
#include <costmap_2d_layers/simple_obstacle_layer.hpp>
#include <costmap_2d_layers/static_layer.hpp>
#include <spdlog/spdlog.h>

#include "ayanami/abstract/components/robot_state_controller.hpp"
#include "ayanami/costmap/components/costmap_commands.hpp"
#include "ayanami/costmap/core/costmap_command.hpp"
#include "ayanami/utility/config.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <functional>
#include <limits>
#include <mutex>
#include <shared_mutex>
#include <utility>

namespace ayanami {
namespace costmap {

namespace {
constexpr double k_default_resolution = 0.05;
constexpr double k_default_inflation_radius = 0.6;
constexpr double k_default_inscribed_radius = 0.5;
constexpr double k_default_cost_scaling_factor = 10.0;
constexpr int k_default_size_x = 0;
constexpr int k_default_size_y = 0;

// Dynamic transforms are sampled into TransformStore at tf_sync_rate
// (50 Hz). Obstacle stamping prefers querying the sensor transform with the
// scan's own timestamp (zero tolerance, no waiting):
// it places the point cloud at the "capture instant" pose -- when the robot
// rotates, a latest query would bake the omega x (now - T_scan) timing skew
// into the costmap (measured: at 1.2rad/s rotation, map-frame point clouds
// drifted 0.13-0.28m between adjacent frames, while scan-stamp queries only
// 0.01-0.04m). fast_tf does not extrapolate:
// when the scan stamp is ahead of synchronized samples (faster-than-realtime
// simulation) or outside the pruning window, the query fails; then it falls
// back to latest (historical behavior) and counts the fallback, exposing the
// fallback rate via throttled logs. The robot pose (rolling centering /
// clear center) still uses latest -- window centering should follow the
// current pose.
// Query timeout policy: fast_tf's timeout parameter is a blocking wait
// budget (cv wait until a new sample or timeout, starving other TF readers/
// writers on the same global lock meanwhile) -- all core-side queries use
// timeout=0 (non-blocking) and take their own fallback paths when missing
// (latest fallback / last pose).
constexpr double k_tf_lookup_timeout_s = 0.0;

// Writer-side reader-starvation warning threshold: readers hold the shared
// lock for <10 ms per call; a writer acquisition that waits longer than this
// means some reader is overstaying its lock and map updates are delayed.
constexpr double k_writer_wait_warn_s = 0.1;

// TF chain missing warn throttling: for the same chain (local/global), at
// most one "no transform" per k_tf_warn_interval_s, avoiding flooding at
// 5.5Hz laser per frame. Returns true when this call should warn.
constexpr double k_tf_warn_interval_s = 5.0;
constexpr double k_robot_diag_interval_s = 60.0;
bool tfWarnDue(std::atomic<std::chrono::steady_clock::time_point>& last) {
  const auto now = std::chrono::steady_clock::now();
  auto prev = last.load();
  if (now - prev < std::chrono::duration<double>(k_tf_warn_interval_s)) {
    return false;
  }
  return last.compare_exchange_strong(prev, now);
}

// Robot pose/rolling diagnostics throttling (one per 60s, for debugging; hints at a missing TF chain on failure).
bool robotDiagDue(std::atomic<std::chrono::steady_clock::time_point>& last) {
  const auto now = std::chrono::steady_clock::now();
  auto prev = last.load();
  if (now - prev < std::chrono::duration<double>(k_robot_diag_interval_s)) {
    return false;
  }
  return last.compare_exchange_strong(prev, now);
}

// Command port implementation: hands typed write commands to the sole
// writer (this server) for execution. Recovery plugins only hold the
// CostmapCommandPort interface and never touch the mutable wrapper/layer.
class CommandPortImpl : public CostmapCommandPort {
 public:
  explicit CommandPortImpl(
      std::function<bool(const ClearCostmapCommand&)> handler)
      : handler(std::move(handler)) {}
  bool submit(const ClearCostmapCommand& command) override {
    return handler ? handler(command) : false;
  }

 private:
  std::function<bool(const ClearCostmapCommand&)> handler;
};
}  // namespace

CostmapNavigationServer::CostmapNavigationServer() {
  command_port = std::make_shared<CommandPortImpl>(
      [this](const ClearCostmapCommand& command) {
        return executeClearCommand(command);
      });
}

CostmapNavigationServer::~CostmapNavigationServer() = default;

void CostmapNavigationServer::configureCoreMaps(
    const std::string& config_path, ayanami::map::MapFamilyFactory::Ptr factory) {
  // The context's sole creation point = the map-family factory (falls back
  // to the costmap factory by default; core and host only know the
  // MapFamilyFactory abstract surface, the CostmapMapContext family surface
  // exists only inside this server).
  map_family_factory = factory ? std::move(factory)
                               : std::make_shared<CostmapMapFamilyFactory>();
  // Double configureCoreMaps guard. When a built context already exists,
  // re-entry is ignored: rebuilding would replace costmap_context and leave
  // the wrapper references of already-injected plugins dangling (after the
  // old context is replaced by the new object, plugins still point at the
  // old wrapper).
  if (costmap_context &&
      (costmap_context->globalCostmap() || costmap_context->localCostmap())) {
    spdlog::warn(
        "configureCoreMaps called twice, ignoring; existing plugins would be "
        "orphaned by context replace");
    return;
  }

  GlobalConfig::instance(config_path, /*override_path=*/true);
  // The costmap params were merged into the config_navigation file (the original costmap.json was removed).
  Config nav_cfg(GlobalConfig::getConfigPath("config_navigation"));
  const Config& cfg = nav_cfg;

  // Frame model (nav2 convention): the global costmap lives in the
  // global_costmap_frame (typically "map"), the local rolling costmap in the
  // local_costmap_frame (typically "odom") so dynamic marks stay immune to
  // localization jumps. map_frame is still honored as a fallback alias for
  // the global frame (legacy configs).
  const std::string global_frame = cfg.param<std::string>(
      "costmap", "global_costmap_frame",
      cfg.param<std::string>(
          "costmap", "global_frame",
          cfg.param<std::string>("costmap", "map_frame", "map")));
  const std::string local_frame = cfg.param<std::string>(
      "costmap", "local_costmap_frame",
      cfg.param<std::string>("costmap", "local_frame", "odom"));
  // robot_frame single source of truth = config_navigation's
  // navigation::robot_frame.
  const std::string robot_frame =
      nav_cfg.param<std::string>("navigation", "robot_frame", "base_link");
  const double resolution =
      cfg.param<double>("costmap", "resolution", k_default_resolution);

  // Consistency validation against config_ros.json::robot_frame.
  // Skipped when config_ros was not loaded (GlobalConfig has no config_ros
  // mapping).
  const std::string ros_config =
      GlobalConfig::getConfigPath("config_ros");
  Config ros_cfg(ros_config);
  const std::string ros_frame =
      ros_cfg.param<std::string>("ayanami_ros", "robot_frame", std::string{});
  if (!ros_frame.empty()) {
    if (robot_frame != ros_frame) {
      spdlog::error("robot_frame mismatch: navigation '" + robot_frame +
                    "' vs ayanami_ros '" + ros_frame + "'");
    }
    assert(robot_frame == ros_frame);
  }

  // ---- global: StaticLayer + SimpleObstacleLayer + InflationLayer ----
  auto global_lm = std::make_shared<costmap_2d::LayeredCostmap>(
      global_frame,
      cfg.param<bool>("costmap_global", "rolling_window", false),
      cfg.param<bool>("costmap_global", "track_unknown", false));
  auto static_layer = std::make_shared<costmap_2d_layers::StaticLayer>();
  global_lm->addLayer(static_layer);
  const std::string static_map =
      cfg.param<std::string>("costmap_global", "static_map", "");
  if (!static_map.empty()) {
    try {
      // A relative static_map (assets/nav.pgm) resolves relative to the
      // package root (the parent of config/), no longer depending on the
      // node cwd (~/.ros under roslaunch).
      std::string map_path = static_map;
      if (map_path.front() != '/' && map_path.find("://") == std::string::npos) {
        map_path = std::filesystem::path(config_path).parent_path().string() +
                   "/" + static_map;
      }
      // Ayanami map metadata is JSON-only and loaded through the common Config
      // API. Middleware adapters keep their deployment metadata outside this package.
      std::filesystem::path metadata_path(map_path);
      metadata_path.replace_extension(".json");
      Config map_cfg(metadata_path.string());
      const auto map_resolution = map_cfg.param<double>("map", "resolution");
      const auto map_origin = map_cfg.param<Eigen::Vector3d>("map", "origin");
      if (!map_resolution || !map_origin) {
        throw std::runtime_error("invalid JSON map metadata: " +
                                 metadata_path.string());
      }
      static_layer->setMapPath(map_path, *map_resolution, map_origin->x(),
                               map_origin->y());
    } catch (const std::exception& e) {
      spdlog::error("CostmapNavigationServer::configureCoreMaps: {}", e.what());
    }
  }
  if (!static_layer->hasMap()) {
    const int fallback_size_x =
        cfg.param<int>("costmap_global", "size_x", k_default_size_x);
    const int fallback_size_y =
        cfg.param<int>("costmap_global", "size_y", k_default_size_y);
    if (fallback_size_x > 0 && fallback_size_y > 0) {
      global_lm->resizeMap(static_cast<unsigned int>(fallback_size_x),
                           static_cast<unsigned int>(fallback_size_y),
                           resolution, 0.0, 0.0);
    } else {
      throw std::runtime_error(
          "static global map failed to load and no fallback size is configured");
    }
  }
  {
    const auto* global_master = global_lm->getCostmap();
    if (!global_master || global_master->getSizeInCellsX() == 0 ||
        global_master->getSizeInCellsY() == 0) {
      throw std::runtime_error("static map loaded but global master geometry is empty");
    }
    spdlog::info(
        "global costmap geometry from static map: {}x{} resolution={} "
        "origin=({}, {})",
        global_master->getSizeInCellsX(), global_master->getSizeInCellsY(),
        global_master->getResolution(), global_master->getOriginX(),
        global_master->getOriginY());
  }

  auto global_obstacle =
      std::make_shared<costmap_2d_layers::SimpleObstacleLayer>();
  global_obstacle->setCombinationMethod(
      costmap_2d_layers::SimpleObstacleLayer::CombinationMethod::MAXIMUM);
  global_obstacle->setClearToFree(false);
  global_lm->addLayer(global_obstacle);

  auto global_inflation = std::make_shared<costmap_2d_layers::InflationLayer>();
  const double global_inflation_radius = cfg.param<double>(
      "costmap_global", "inflation_radius", k_default_inflation_radius);
  global_inflation->setInflationRadius(global_inflation_radius);
  global_inflation->setInscribedRadius(cfg.param<double>(
      "costmap_global", "inscribed_radius", k_default_inscribed_radius));
  global_inflation->setCostScalingFactor(cfg.param<double>(
      "costmap_global", "cost_scaling_factor", k_default_cost_scaling_factor));
  global_lm->addLayer(global_inflation);

  // ---- local: SimpleObstacleLayer + InflationLayer (rolling, LOCAL frame) ----
  auto local_lm = std::make_shared<costmap_2d::LayeredCostmap>(
      local_frame,
      cfg.param<bool>("costmap_local", "rolling_window", true),
      cfg.param<bool>("costmap_local", "track_unknown", false));
  const int local_size_x =
      cfg.param<int>("costmap_local", "size_x", k_default_size_x);
  const int local_size_y =
      cfg.param<int>("costmap_local", "size_y", k_default_size_y);
  if (local_size_x > 0 && local_size_y > 0) {
    local_lm->resizeMap(static_cast<unsigned int>(local_size_x),
                        static_cast<unsigned int>(local_size_y), resolution,
                        0.0, 0.0);
  }

  auto obstacle_layer = std::make_shared<costmap_2d_layers::SimpleObstacleLayer>();
  obstacle_layer->setCombinationMethod(
      costmap_2d_layers::SimpleObstacleLayer::CombinationMethod::OVERWRITE);
  local_lm->addLayer(obstacle_layer);

  auto local_inflation = std::make_shared<costmap_2d_layers::InflationLayer>();
  const double local_inflation_radius = cfg.param<double>(
      "costmap_local", "inflation_radius", k_default_inflation_radius);
  local_inflation->setInflationRadius(local_inflation_radius);
  local_inflation->setInscribedRadius(cfg.param<double>(
      "costmap_local", "inscribed_radius", k_default_inscribed_radius));
  local_inflation->setCostScalingFactor(cfg.param<double>(
      "costmap_local", "cost_scaling_factor", k_default_cost_scaling_factor));
  local_lm->addLayer(local_inflation);

  auto local_wrapper =
      std::make_shared<costmap_2d::CostmapWrapper>(local_lm, local_frame);
  // Materialize the local map once at configure: layers start non-current,
  // and until the first updateMap the execution-layer safetyCheck would
  // gate the controller/recovery loops on a not-yet-current map.
  local_wrapper->updateMap(0.0, 0.0, 0.0);

  // Robot footprint sources (nav_demo parameter semantics):
  // - costmap::footprint polygon (nav2 semantics, highest priority)
  // - costmap::robot_radius > 0 -> generate a 16-gon circle approximation
  //   (ROS1 robot_radius semantics)
  // - both absent -> not set (the inscribed_radius parameter serves as the
  //   inflation layer fallback)
  costmap_2d::Footprint footprint;
  const auto footprint_values = cfg.param<std::vector<Eigen::Vector2d>>(
      "costmap", "footprint", {});
  footprint.reserve(footprint_values.size());
  for (const auto& point : footprint_values) {
    footprint.push_back(costmap_2d::Point{point.x(), point.y(), 0.0});
  }
  if (footprint.empty()) {
    const double robot_radius =
        cfg.param<double>("costmap", "robot_radius", 0.0);
    if (robot_radius > 0.0) {
      constexpr int k_circle_segments = 16;
      footprint.reserve(k_circle_segments);
      for (int i = 0; i < k_circle_segments; ++i) {
        const double angle = 2.0 * M_PI * static_cast<double>(i) /
                             static_cast<double>(k_circle_segments);
        footprint.push_back(costmap_2d::Point{
            robot_radius * std::cos(angle), robot_radius * std::sin(angle),
            0.0});
      }
    }
  }
  if (!footprint.empty()) {
    // Nav2 semantics: a configured footprint is the source of truth for the
    // inscribed radius. The explicit radius above is only a fallback when no
    // polygon is configured.
    global_lm->setFootprint(footprint);
    local_lm->setFootprint(footprint);
  }

  auto global_wrapper =
      std::make_shared<costmap_2d::CostmapWrapper>(global_lm, global_frame);
  // Materialize once after footprint propagation, avoiding a second full-map
  // reinflation during startup.
  global_wrapper->updateMap(0.0, 0.0, 0.0);

  global_obstacle_layer = global_obstacle.get();
  local_obstacle_layer = obstacle_layer.get();
  this->global_inflation_radius = global_inflation_radius;
  this->local_inflation_radius = local_inflation_radius;
  global_resolution = resolution;
  local_resolution = resolution;
  local_raytrace_range = cfg.param<double>(
      "costmap_local", "raytrace_range", local_raytrace_range);
 // Decoupled rolling-window update frequency (periodic rolling in spinOnce, independent of the sensor stream)
  const double local_update_hz =
      cfg.param<double>("costmap_local", "update_frequency", 10.0);
  const double global_update_hz =
      cfg.param<double>("costmap_global", "update_frequency", 5.0);
  local_roll_period = local_update_hz > 0.0
                          ? std::chrono::duration<double>(1.0 / local_update_hz)
                          : std::chrono::duration<double>(0.0);
  global_roll_period = global_update_hz > 0.0
                           ? std::chrono::duration<double>(1.0 / global_update_hz)
                           : std::chrono::duration<double>(0.0);

  costmap_context =
      std::dynamic_pointer_cast<CostmapMapContext>(map_family_factory->createContext());
  if (!costmap_context) {
    throw std::runtime_error(
        "map family factory '" + map_family_factory->familyName() +
        "' did not create a CostmapMapContext");
  }
  costmap_context->setGlobalCostmap(global_wrapper);
  costmap_context->setLocalCostmap(local_wrapper);
  costmap_context->setFrames(global_frame, local_frame, robot_frame);
  // Writer-side reader-starvation observability (see CostmapWrapper).
  global_wrapper->setWriterWaitWarn(k_writer_wait_warn_s);
  local_wrapper->setWriterWaitWarn(k_writer_wait_warn_s);
  // The footprint is cached as an immutable copy: snapshot building reads
  // only this cache (never read the live LayeredCostmap's footprint outside
  // the write lock).
  costmap_context->setFootprint(footprint);
  // The map write command port is exposed via the context
  // (AbstractMapContext::commandPort):
  // family plugins such as recovery fetch it from the map context at
  // configure time instead of side-path injection.
  costmap_context->setCommandPort(command_port);

  // One-time configuration of the global RobotStateController: frame names
  // + velocity timeout are cached only here; the hot getRobotState never
  // reads Config (extension).
  const double velocity_timeout =
      nav_cfg.param<double>("navigation", "velocity_timeout", 0.25);
  ayanami::components::RobotStateController::instance().initialize(
      global_frame, local_frame, robot_frame, velocity_timeout);

  prev_obstacles_global.clear();
  prev_obstacles_local.clear();
  has_prev_obstacles_global = false;
  has_prev_obstacles = false;
  has_last_global_robot = false;
  has_last_local_robot = false;
}

// ---- Virtual initialize*Plugin hooks: costmap family injection ----

bool CostmapNavigationServer::initializePlannerPlugin(
    const std::string& name, const ayanami::Config& module_config,
    ayanami::abstract::AbstractPlanner::Ptr ptr) {
  if (!costmap_context) {
    spdlog::error(
        "CostmapNavigationServer::initializePlannerPlugin('{}'): costmap "
        "context not configured (call configureCoreMaps first)",
        name);
    return false;
  }
  auto cast = std::dynamic_pointer_cast<CostmapPlanner>(ptr);
  if (!cast) {
    spdlog::error(
        "CostmapNavigationServer::initializePlannerPlugin('{}'): plugin is "
        "not a CostmapPlanner (costmap family interface mismatch)",
        name);
    return false;
  }
  cast->configure(ayanami::PluginContext{name, module_config},
                  costmap_context->getSharedConstPtr());
  return true;
}

bool CostmapNavigationServer::initializeSmootherPlugin(
    const std::string& name, const ayanami::Config& module_config,
    ayanami::abstract::AbstractSmoother::Ptr ptr) {
  if (!costmap_context) {
    spdlog::error(
        "CostmapNavigationServer::initializeSmootherPlugin('{}'): costmap "
        "context not configured (call configureCoreMaps first)",
        name);
    return false;
  }
  auto cast = std::dynamic_pointer_cast<CostmapSmoother>(ptr);
  if (!cast) {
    spdlog::error(
        "CostmapNavigationServer::initializeSmootherPlugin('{}'): plugin is "
        "not a CostmapSmoother (costmap family interface mismatch)",
        name);
    return false;
  }
  cast->configure(ayanami::PluginContext{name, module_config},
                  costmap_context->getSharedConstPtr());
  return true;
}

bool CostmapNavigationServer::initializeControllerPlugin(
    const std::string& name, const ayanami::Config& module_config,
    ayanami::abstract::AbstractController::Ptr ptr) {
  if (!costmap_context) {
    spdlog::error(
        "CostmapNavigationServer::initializeControllerPlugin('{}'): costmap "
        "context not configured (call configureCoreMaps first)",
        name);
    return false;
  }
  // Strict costmap family constraint: only CostmapController is accepted;
  // plain AbstractControllers are no longer allowed (costmap_pid now derives
  // from CostmapController).
  auto cast = std::dynamic_pointer_cast<CostmapController>(ptr);
  if (!cast) {
    spdlog::error(
        "CostmapNavigationServer::initializeControllerPlugin('{}'): plugin is "
        "not a CostmapController (costmap family interface mismatch)",
        name);
    return false;
  }
  cast->configure(ayanami::PluginContext{name, module_config},
                  costmap_context->getSharedConstPtr());
  return true;
}

bool CostmapNavigationServer::initializeRecoveryPlugin(
    const std::string& name, const ayanami::Config& module_config,
    ayanami::abstract::AbstractRecovery::Ptr ptr) {
  if (!costmap_context) {
    spdlog::error(
        "CostmapNavigationServer::initializeRecoveryPlugin('{}'): costmap "
        "context not configured (call configureCoreMaps first)",
        name);
    return false;
  }
  auto cast = std::dynamic_pointer_cast<CostmapRecovery>(ptr);
  if (!cast) {
    spdlog::error(
        "CostmapNavigationServer::initializeRecoveryPlugin('{}'): plugin is "
        "not a CostmapRecovery (costmap family interface mismatch)",
        name);
    return false;
  }
  cast->configure(ayanami::PluginContext{name, module_config},
                  costmap_context->getSharedConstPtr());
  // Stash the module config so createActions can inject per-plugin
  // execution settings (tick cycle period).
  recovery_configs.erase(name);
  recovery_configs.emplace(name, module_config);
  // Map write command port: recovery fetches it via map->commandPort
  // inside configure (CostmapCommandPort), executed by this server (the
  // sole writer).
  return true;
}

// ---- createActions: production path = Costmap*Action ----

void CostmapNavigationServer::createActions() {
  planner_actions.clear();
  smoother_actions.clear();
  controller_actions.clear();
  recovery_actions.clear();

  // planner/smoother/controller: one Costmap*Action per loaded plugin
  // (Costmap*Execution carrying the ctx; nav2 multi-plugin semantics:
  // requests select by goal id, empty id = default = first configured).
  for (const auto& name : planner_plugins.getLoadedNames()) {
    auto planner = planner_plugins.getPlugin(name);
    if (!planner) {
      continue;
    }
    planner_actions[name] = std::make_unique<CostmapPlannerAction>(
        planner, costmap_context);
  }
  if (!planner_actions.empty() &&
      planner_actions.find(default_planner) == planner_actions.end()) {
    spdlog::error("CostmapNavigationServer::createActions: default planner "
                  "'{}' not loaded",
                  default_planner);
  }

  for (const auto& name : smoother_plugins.getLoadedNames()) {
    auto smoother = smoother_plugins.getPlugin(name);
    if (!smoother) {
      continue;
    }
    smoother_actions[name] = std::make_unique<CostmapSmootherAction>(
        smoother, costmap_context);
  }
  if (!smoother_actions.empty() &&
      smoother_actions.find(default_smoother) == smoother_actions.end()) {
    spdlog::error("CostmapNavigationServer::createActions: default smoother "
                  "'{}' not loaded",
                  default_smoother);
  }

  // controller: CostmapControllerExecution + checker/velocity-limit/real
  // pose-source assembly.
  for (const auto& name : controller_plugins.getLoadedNames()) {
    auto controller = controller_plugins.getPlugin(name);
    if (!controller) {
      continue;
    }
    controller_actions[name] = std::make_unique<CostmapControllerAction>(
        controller, costmap_context,
        [this](ayanami::components::ControllerExecution::Ptr exec) {
          configureControllerExecution(exec);
        });
  }
  if (!controller_actions.empty() &&
      controller_actions.find(default_controller) == controller_actions.end()) {
    spdlog::error("CostmapNavigationServer::createActions: default controller "
                  "'{}' not loaded",
                  default_controller);
  }

  // recovery: CostmapRecoveryExecution + velocity-limit assembly +
  // per-plugin tick cycle period (recovery.sim_cycle_dt, seconds).
  for (const auto& name : recovery_plugins.getLoadedNames()) {
    auto recovery = recovery_plugins.getPlugin(name);
    if (!recovery) {
      continue;
    }
    double cycle_period_s = 0.01;
    const auto config_iter = recovery_configs.find(name);
    if (config_iter != recovery_configs.end()) {
      cycle_period_s = config_iter->second.param<double>(
          "recovery", "sim_cycle_dt", cycle_period_s);
    }
    auto action = std::make_unique<CostmapRecoveryAction>(
        recovery, costmap_context,
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

// ---- Obstacle observation map stamping ----

void CostmapNavigationServer::handleObstaclePointCloud(
    const ayanami::sensor_msgs::PointCloud2& cloud) {
  if (!costmap_context || !costmap_context->localCostmap()) {
    spdlog::warn(
        "CostmapNavigationServer::handleObstaclePointCloud: costmap not "
        "configured (call configureCoreMaps first)");
    return;
  }
  std::vector<Eigen::Vector3d> points;
  points.reserve(cloud.cloud.points.size());
  for (const auto& p : cloud.cloud.points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y)) {
      continue;
    }
    points.emplace_back(static_cast<double>(p.x), static_cast<double>(p.y),
                        0.0);
  }
  markObstaclesInFrame(cloud.header.frame_id, cloud.header.stamp, points);
}

void CostmapNavigationServer::handleObstacleLaserScan(
    const ayanami::sensor_msgs::LaserScan& scan) {
  if (!costmap_context || !costmap_context->localCostmap()) {
    spdlog::warn(
        "CostmapNavigationServer::handleObstacleLaserScan: costmap not "
        "configured (call configureCoreMaps first)");
    return;
  }
  if (scan.ranges.empty()) {
    return;
  }
  std::vector<Eigen::Vector3d> points;
  std::vector<Eigen::Vector3d> clearing_points;
  points.reserve(scan.ranges.size());
  clearing_points.reserve(scan.ranges.size());
  for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
    const double range = static_cast<double>(scan.ranges[i]);
    const double angle = scan.angle_min + i * scan.angle_increment;
    const double clear_range =
        std::isfinite(range)
            ? std::clamp(range, 0.0, local_raytrace_range)
            : local_raytrace_range;
    clearing_points.emplace_back(clear_range * std::cos(angle),
                                 clear_range * std::sin(angle), 0.0);
    if (range >= scan.range_min && range <= scan.range_max) {
      points.emplace_back(range * std::cos(angle), range * std::sin(angle),
                          0.0);
    }
  }
  markObstaclesInFrame(scan.header.frame_id, scan.header.stamp, points,
                       clearing_points);
}

void CostmapNavigationServer::markObstaclesInFrame(
    const std::string& frame_id, const ayanami::Timestamp& stamp,
    const std::vector<Eigen::Vector3d>& points,
    const std::vector<Eigen::Vector3d>& clearing_points) {
  if (!costmap_context || !costmap_context->localCostmap() ||
      !local_obstacle_layer) {
    spdlog::warn(
        "CostmapNavigationServer::markObstaclesInFrame: costmap not "
        "configured (call configureCoreMaps first)");
    return;
  }
  auto& robot_state_controller =
      ayanami::components::RobotStateController::instance();
  const std::string& global_frame = costmap_context->getGlobalFrame();
  const std::string& local_frame = costmap_context->getLocalFrame();
  const bool have_stamp = !stamp.isLatest();

  // Per-map stamping: each costmap is stamped in its own frame (global =
  // global_costmap_frame, local = local_costmap_frame, typically map/odom).
  // The sensor transform is queried per frame -- the scan's own timestamp
  // first (zero tolerance: fall back to latest immediately on query failure,
  // never blocking the callback thread; fast_tf does not extrapolate, so a
  // scan stamp ahead of the sync chain or outside the pruning window falls
  // back to latest and is counted).
  auto stampOneMap =
      [&](const std::string& map_frame,
          costmap_2d::CostmapWrapper& wrapper,
          costmap_2d_layers::SimpleObstacleLayer& layer,
          double inflation_radius, double resolution,
          bool& has_last, double& last_x, double& last_y, double& last_yaw,
          std::vector<Eigen::Vector2d>& prev, bool& has_prev) {
        if (points.empty() && clearing_points.empty()) {
          return;
        }
        Eigen::Isometry3d T_frame_sensor = Eigen::Isometry3d::Identity();
        bool stamp_ok = false;
        if (have_stamp &&
            robot_state_controller.lookupTransform(map_frame, frame_id, stamp,
                                                   T_frame_sensor, 0.0)) {
          stamp_ok = true;
          ++stamp_query_ok;
        }
        if (!stamp_ok &&
            !robot_state_controller.lookupTransform(
                map_frame, frame_id, ayanami::Timestamp::latest(),
                T_frame_sensor, k_tf_lookup_timeout_s)) {
          if (have_stamp) {
            ++stamp_query_fallback;
          }
          if (tfWarnDue(map_tf_warn_at)) {
            spdlog::warn(
                "CostmapNavigationServer::markObstaclesInFrame: no transform "
                "{} <- {} (stamp={:f}s); obstacle stamping paused. Check the "
                "TF producers (gazebo diff-drive odom->base_footprint, "
                "robot_state_publisher) and whether a leftover simulation "
                "instance publishes duplicate TF",
                map_frame, frame_id, stamp.toSeconds());
          }
          return;
        }

        // Points in this map's frame + observation bounding box.
        std::vector<Eigen::Vector2d> world;
        world.reserve(points.size());
        double min_x = std::numeric_limits<double>::max();
        double min_y = std::numeric_limits<double>::max();
        double max_x = std::numeric_limits<double>::lowest();
        double max_y = std::numeric_limits<double>::lowest();
        for (const auto& p : points) {
          const Eigen::Vector3d w = T_frame_sensor * p;
          world.emplace_back(w.x(), w.y());
          min_x = std::min(min_x, w.x());
          min_y = std::min(min_y, w.y());
          max_x = std::max(max_x, w.x());
          max_y = std::max(max_y, w.y());
        }

        // Resolve the robot pose (this map's frame) before taking the write
        // lock. On query failure (transient TF loss) roll the window with the
        // last successful pose -- never fall back to (0,0,0), otherwise the
        // rolling window gets anchored back to the frame origin and, once the
        // robot walks out of the window, dwb has no path.
        double robot_x = 0.0;
        double robot_y = 0.0;
        double robot_yaw = 0.0;
        Eigen::Isometry3d T_robot = Eigen::Isometry3d::Identity();
        if (robot_state_controller.lookupTransform(
                map_frame, costmap_context->getRobotFrame(),
                ayanami::Timestamp::latest(), T_robot,
                k_tf_lookup_timeout_s)) {
          robot_x = T_robot.translation().x();
          robot_y = T_robot.translation().y();
          robot_yaw =
              std::atan2(T_robot.linear()(1, 0), T_robot.linear()(0, 0));
          has_last = true;
          last_x = robot_x;
          last_y = robot_y;
          last_yaw = robot_yaw;
          if (robotDiagDue(robot_diag_at)) {
            spdlog::debug(
                "markObstacles: robot pose OK {}<-(robot_frame={}) "
                "trans=({:.3f},{:.3f})",
                map_frame, costmap_context->getRobotFrame(), robot_x, robot_y);
          }
        } else {
          if (robotDiagDue(robot_diag_at)) {
            spdlog::warn(
                "markObstacles: robot pose FAIL {}<-(robot_frame={}); "
                "has_last={} (rolling windows keep the last pose)",
                map_frame, costmap_context->getRobotFrame(), has_last);
          }
          if (has_last) {
            robot_x = last_x;
            robot_y = last_y;
            robot_yaw = last_yaw;
          }
        }

        // Stamp this map: raytrace freespace, clear the previous + current
        // observation window (removing old marks and old inflation bands;
        // the InflationLayer then fully recomputes within its recompute
        // window), mark the current obstacles and recompute the master grid.
        // Each map owns only its own write lock (taken sequentially, never
        // nested).
        auto write_lock = wrapper.lockForWrite();
        if (!clearing_points.empty()) {
          const Eigen::Vector3d sensor_origin = T_frame_sensor.translation();
          const auto* grid = wrapper.get().getCostmap();
          for (const auto& point : clearing_points) {
            const Eigen::Vector3d endpoint = T_frame_sensor * point;
            layer.raytraceFreespace(*grid, sensor_origin.x(),
                                    sensor_origin.y(), endpoint.x(),
                                    endpoint.y());
          }
        } else if (has_prev) {
          double clear_min_x = min_x;
          double clear_min_y = min_y;
          double clear_max_x = max_x;
          double clear_max_y = max_y;
          for (const auto& old : prev) {
            clear_min_x = std::min(clear_min_x, old.x());
            clear_min_y = std::min(clear_min_y, old.y());
            clear_max_x = std::max(clear_max_x, old.x());
            clear_max_y = std::max(clear_max_y, old.y());
          }
          // The expansion margin is the inflation radius + one cell: the band
          // can reach up to ceil(radius/res) cells, and world->cell rounding
          // would exclude cells lying exactly on the radius boundary.
          const double margin = inflation_radius + std::max(resolution, 0.0);
          layer.clearArea(clear_min_x - margin, clear_min_y - margin,
                          clear_max_x + margin, clear_max_y + margin);
        }
        for (const auto& w : world) {
          layer.markObstacles(w.x(), w.y());
        }
        prev = world;
        has_prev = true;
        wrapper.updateMapUnlocked(robot_x, robot_y, robot_yaw);
      };

  // Local first, then global (matching the reader lock order local -> global).
  stampOneMap(local_frame, *costmap_context->localCostmap(),
              *local_obstacle_layer, local_inflation_radius, local_resolution,
              has_last_local_robot, last_local_robot_x, last_local_robot_y,
              last_local_robot_yaw, prev_obstacles_local, has_prev_obstacles);
  if (costmap_context->globalCostmap() && global_obstacle_layer) {
    stampOneMap(global_frame, *costmap_context->globalCostmap(),
                *global_obstacle_layer, global_inflation_radius,
                global_resolution, has_last_global_robot, last_global_robot_x,
                last_global_robot_y, last_global_robot_yaw,
                prev_obstacles_global, has_prev_obstacles_global);
  }

  // Scan-stamp query fallback rate observation (a fallback means that
  // frame's sensor transform degraded to latest-time semantics, which bakes
  // an omega x dt stamping skew into the costmap while rotating): a
  // persistently high fallback rate indicates the scan stamp deviates from
  // the sync chain beyond the fast_tf pruning window (check RTF / TF sync
  // health).
  if (stamp_query_fallback.load() > 0) {
    const auto diag_now = std::chrono::steady_clock::now();
    if (diag_now - last_stamp_diag_at >= std::chrono::seconds(10)) {
      last_stamp_diag_at = diag_now;
      spdlog::info("markObstacles: stamp-queries ok={} fallback={}",
                   stamp_query_ok.load(), stamp_query_fallback.load());
      stamp_query_ok.store(0);
      stamp_query_fallback.store(0);
    }
  }
}

// ---- Map change command execution (sole writer) ----

void CostmapNavigationServer::updateRollingWindows() {
  if (!costmap_context) {
    return;
  }
  auto& robot_state_controller =
      ayanami::components::RobotStateController::instance();
  const auto now = std::chrono::steady_clock::now();

  // Local map: when rolling, center the window on the latest pose in the
  // LOCAL costmap frame (local_costmap_frame, typically odom). Pose
  // resolution happens before taking the lock; on query failure, reuse the
  // last successful pose (never fall back to the origin).
  if (local_roll_period.count() > 0.0 && costmap_context->localCostmap() &&
      costmap_context->localCostmap()->isRollingWindow() &&
      now - last_local_roll >= local_roll_period) {
    last_local_roll = now;
    const std::string& local_frame = costmap_context->getLocalFrame();
    Eigen::Isometry3d t_robot = Eigen::Isometry3d::Identity();
    if (robot_state_controller.lookupTransform(
            local_frame, costmap_context->getRobotFrame(),
            ayanami::Timestamp::latest(), t_robot, k_tf_lookup_timeout_s)) {
      has_last_local_robot = true;
      last_local_robot_x = t_robot.translation().x();
      last_local_robot_y = t_robot.translation().y();
      last_local_robot_yaw =
          std::atan2(t_robot.linear()(1, 0), t_robot.linear()(0, 0));
    }
    if (has_last_local_robot) {
      auto lock = costmap_context->localCostmap()->lockForWrite();
      costmap_context->localCostmap()->updateMapUnlocked(
          last_local_robot_x, last_local_robot_y, last_local_robot_yaw);
    }
  }

  // Global map: only needed for rolling configs (production uses a static
  // map, skipped). Centered on the latest pose in the global costmap frame.
  if (global_roll_period.count() > 0.0 && costmap_context->globalCostmap() &&
      costmap_context->globalCostmap()->isRollingWindow() &&
      now - last_global_roll >= global_roll_period) {
    last_global_roll = now;
    const std::string& global_frame = costmap_context->getGlobalFrame();
    Eigen::Isometry3d t_robot = Eigen::Isometry3d::Identity();
    if (robot_state_controller.lookupTransform(
            global_frame, costmap_context->getRobotFrame(),
            ayanami::Timestamp::latest(), t_robot, k_tf_lookup_timeout_s)) {
      const double gx = t_robot.translation().x();
      const double gy = t_robot.translation().y();
      const double gyaw =
          std::atan2(t_robot.linear()(1, 0), t_robot.linear()(0, 0));
      auto lock = costmap_context->globalCostmap()->lockForWrite();
      costmap_context->globalCostmap()->updateMapUnlocked(gx, gy, gyaw);
    }
  }
}

// ---- Map change command execution (sole writer) ----

bool CostmapNavigationServer::executeClearCommand(
    const ClearCostmapCommand& command) {
  if (!command.enabled) {
    return true;
  }
  if (!costmap_context) {
    spdlog::warn("CostmapNavigationServer::executeClearCommand: costmap not "
                 "configured");
    return false;
  }
  auto& robot_state_controller =
      ayanami::components::RobotStateController::instance();
  const std::string& global_frame = costmap_context->getGlobalFrame();
  const std::string& local_frame = costmap_context->getLocalFrame();

  const bool clear_global =
      command.affected_maps == "global" || command.affected_maps == "both";
  const bool clear_local =
      command.affected_maps == "local" || command.affected_maps == "both";
  const double radius =
      command.clear_radius > 0.0 ? command.clear_radius : 0.0;

  auto clearOne = [&](const costmap_2d::CostmapWrapper::Ptr& wrapper,
                       const std::string& frame) -> bool {
    if (!wrapper || frame.empty()) {
      return false;
    }
    Eigen::Isometry3d robot_pose = Eigen::Isometry3d::Identity();
    if (!robot_state_controller.lookupTransform(
            frame, costmap_context->getRobotFrame(),
            ayanami::Timestamp::latest(), robot_pose)) {
      return false;
    }
    // Sole writer: clear the dynamic layer + recompute + updateMap. The
    // mutable layers/master grid are never exposed to recovery plugins.
    return clearCostmapArea(wrapper, robot_pose, radius);
  };

  bool ok = true;
  if (clear_global) {
    ok = clearOne(costmap_context->globalCostmap(), global_frame) && ok;
  }
  if (clear_local) {
    ok = clearOne(costmap_context->localCostmap(), local_frame) && ok;
  }
  return ok;
}

}  // namespace costmap
}  // namespace ayanami
