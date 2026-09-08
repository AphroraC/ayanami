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

  const std::string global_frame =
      cfg.param<std::string>("costmap", "global_frame", "map");
  const std::string local_frame =
      cfg.param<std::string>("costmap", "local_frame", "odom");
  // robot_frame single source of truth = config_navigation's
  // navigation::robot_frame.
  const std::string robot_frame =
      nav_cfg.param<std::string>("navigation", "robot_frame", "base_link");
  const double resolution =
      cfg.param<double>("costmap", "resolution", k_default_resolution);

  // §3.2: consistency validation against config_ros.json::robot_frame.
  // Skipped when config_ros was not loaded (GlobalConfig has no config_ros
  // mapping).
  const std::string ros_config =
      GlobalConfig::getConfigPath("config_ros");
  Config ros_cfg(ros_config);
  const std::string ros_frame =
      ros_cfg.param<std::string>("ayanami_ros", "robot_frame", std::string{});
  if (!ros_frame.empty()) {
    if (robot_frame != ros_frame) {
      spdlog::error("robot_frame mismatch: navigation '{}' vs ayanami_ros '{}'",
                    robot_frame, ros_frame);
    }
    assert(robot_frame == ros_frame);
  }
  // local frame single source of truth = costmap.local_frame;
  // config_ros.odom_frame is only the ROS TF sync-edge configuration; the
  // two must agree semantically (warn on mismatch instead of silently
  // diverging).
  const std::string ros_odom_frame =
      ros_cfg.param<std::string>("ayanami_ros", "odom_frame", std::string{});
  if (!ros_odom_frame.empty() && local_frame != ros_odom_frame) {
    spdlog::error(
        "local_frame mismatch: costmap.local_frame '{}' vs ayanami_ros "
        "odom_frame '{}'; 局部地图与 TF 同步将分叉",
        local_frame, ros_odom_frame);
    assert(local_frame == ros_odom_frame);
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

  // ---- local: SimpleObstacleLayer + InflationLayer (rolling) ----
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
        "not a CostmapPlanner (costmap 族接口 mismatch)",
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
        "not a CostmapSmoother (costmap 族接口 mismatch)",
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
        "not a CostmapController (costmap 族接口 mismatch)",
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
        "not a CostmapRecovery (costmap 族接口 mismatch)",
        name);
    return false;
  }
  cast->configure(ayanami::PluginContext{name, module_config},
                  costmap_context->getSharedConstPtr());
  // Map write command port: recovery fetches it via map->commandPort
  // inside configure (CostmapCommandPort), executed by this server (the
  // sole writer).
  return true;
}

// ---- createActions: production path = Costmap*Action ----

void CostmapNavigationServer::createActions() {
  planner_action.reset();
  smoother_action.reset();
  controller_action.reset();
  recovery_actions.clear();

  // planner/smoother: Costmap*Action (Costmap*Execution carrying the ctx).
  if (auto planner = planner_plugins.getPlugin(default_planner)) {
    planner_action = std::make_unique<CostmapPlannerAction>(
        planner, costmap_context);
  }
  if (!planner_plugins.hasPlugin(default_planner)) {
    spdlog::error("CostmapNavigationServer::createActions: default planner "
                  "'{}' not loaded",
                  default_planner);
  }

  if (auto smoother = smoother_plugins.getPlugin(default_smoother)) {
    smoother_action = std::make_unique<CostmapSmootherAction>(
        smoother, costmap_context);
  }
  if (!smoother_plugins.hasPlugin(default_smoother)) {
    spdlog::error("CostmapNavigationServer::createActions: default smoother "
                  "'{}' not loaded",
                  default_smoother);
  }

  // controller: CostmapControllerExecution + checker/velocity-limit/real pose-source assembly.
  if (auto controller = controller_plugins.getPlugin(default_controller)) {
    auto action = std::make_unique<CostmapControllerAction>(
        controller, costmap_context,
        [this](ayanami::components::ControllerExecution::Ptr exec) {
          configureControllerExecution(exec);
        });
    controller_action = std::move(action);
  }
  if (!controller_plugins.hasPlugin(default_controller)) {
    spdlog::error("CostmapNavigationServer::createActions: default controller "
                  "'{}' not loaded",
                  default_controller);
  }

  // recovery: CostmapRecoveryExecution + velocity-limit assembly.
  for (const auto& name : recovery_plugins.getLoadedNames()) {
    auto recovery = recovery_plugins.getPlugin(name);
    if (!recovery) {
      continue;
    }
    auto action = std::make_unique<CostmapRecoveryAction>(
        recovery, costmap_context,
        [this](ayanami::components::RecoveryExecution::Ptr exec) {
          configureRecoveryExecution(exec);
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
  if (cloud.point_step == 0) {
    spdlog::warn("handleObstaclePointCloud: invalid point_step=0");
    return;
  }
  int offset_x = -1;
  int offset_y = -1;
  for (const auto& field : cloud.fields) {
    if (field.name == "x") {
      offset_x = static_cast<int>(field.offset);
    } else if (field.name == "y") {
      offset_y = static_cast<int>(field.offset);
    }
  }
  if (offset_x < 0 || offset_y < 0) {
    spdlog::warn("handleObstaclePointCloud: missing x/y fields");
    return;
  }

  const std::size_t count = static_cast<std::size_t>(cloud.height) *
                            static_cast<std::size_t>(cloud.width);
  std::vector<Eigen::Vector3d> points;
  points.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    const std::size_t base = i * static_cast<std::size_t>(cloud.point_step);
    const std::size_t max_field =
        static_cast<std::size_t>(std::max(offset_x, offset_y));
    if (base + max_field + sizeof(float) > cloud.data.size()) {
      continue;
    }
    float x = 0.0f;
    float y = 0.0f;
    std::memcpy(&x, &cloud.data[base + static_cast<std::size_t>(offset_x)],
                sizeof(float));
    std::memcpy(&y, &cloud.data[base + static_cast<std::size_t>(offset_y)],
                sizeof(float));
    points.emplace_back(static_cast<double>(x), static_cast<double>(y), 0.0);
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
  const std::string& local_frame =
      costmap_context->localCostmap()->getGlobalFrame();

  Eigen::Isometry3d T_local = Eigen::Isometry3d::Identity();
 // The sensor transform is preferably queried at the scan capture instant
 // (zero tolerance: fall back immediately on query failure, never blocking
 // the callback thread); on failure (scan stamp ahead of synchronized
 // samples / outside the pruning window), fall back to latest.
  const bool have_stamp = !stamp.isLatest();
  bool local_stamp_ok = false;
  if (have_stamp &&
      robot_state_controller.lookupTransform(local_frame, frame_id, stamp,
                                             T_local, 0.0)) {
    local_stamp_ok = true;
    ++stamp_query_ok;
  }
  if ((!points.empty() || !clearing_points.empty()) && !local_stamp_ok &&
      !robot_state_controller.lookupTransform(
          local_frame, frame_id, ayanami::Timestamp::latest(), T_local,
          k_tf_lookup_timeout_s)) {
    if (have_stamp) {
      ++stamp_query_fallback;
    }
    if (tfWarnDue(local_tf_warn_at)) {
      spdlog::warn("CostmapNavigationServer::markObstaclesInFrame: no "
                   "transform {} <- {} (stamp={:f}s); 障碍落图暂停。检查 TF "
                   "生产者 (gazebo 差速 odom->base_footprint、robot_state_"
                   "publisher) 是否存活, 以及是否存在残留的多个仿真实例同时"
                   "发布同名 TF",
                   local_frame, frame_id, stamp.toSeconds());
    }
    return;
  }

  std::vector<Eigen::Vector2d> world;
  world.reserve(points.size());
  double min_x = std::numeric_limits<double>::max();
  double min_y = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double max_y = std::numeric_limits<double>::lowest();
  for (const auto& p : points) {
    const Eigen::Vector3d w = T_local * p;
    world.emplace_back(w.x(), w.y());
    min_x = std::min(min_x, w.x());
    min_y = std::min(min_y, w.y());
    max_x = std::max(max_x, w.x());
    max_y = std::max(max_y, w.y());
  }

  // Resolve robot poses before taking either map lock. Each map update below
  // owns only its own mutex, so obstacle ingestion never nests local->global.
  // On query failure (transient TF loss) roll the window with the last
  // successful pose -- never fall back to (0,0,0), otherwise the rolling
  // window gets anchored back to the world origin and, once the robot walks
  // out of the window, dwb has no path.
  double robot_x = 0.0;
  double robot_y = 0.0;
  double robot_yaw = 0.0;
  Eigen::Isometry3d T_robot = Eigen::Isometry3d::Identity();
  if (robot_state_controller.lookupTransform(
          local_frame, costmap_context->getRobotFrame(),
          ayanami::Timestamp::latest(), T_robot, k_tf_lookup_timeout_s)) {
    robot_x = T_robot.translation().x();
    robot_y = T_robot.translation().y();
    robot_yaw = std::atan2(T_robot.linear()(1, 0), T_robot.linear()(0, 0));
    has_last_local_robot = true;
    last_local_robot_x = robot_x;
    last_local_robot_y = robot_y;
    last_local_robot_yaw = robot_yaw;
    if (robotDiagDue(local_robot_diag_at)) {
      spdlog::debug("markObstacles: robot pose OK {}<-(robot_frame={}) "
                    "trans=({:.3f},{:.3f})",
                   local_frame, costmap_context->getRobotFrame(), robot_x,
                   robot_y);
    }
  } else {
    if (robotDiagDue(local_robot_diag_at)) {
      spdlog::warn("markObstacles: robot pose FAIL {}<-(robot_frame={}); "
                   "has_last={} (滚动窗口将保持上次位姿)",
                   local_frame, costmap_context->getRobotFrame(),
                   has_last_local_robot);
    }
    if (has_last_local_robot) {
      robot_x = last_local_robot_x;
      robot_y = last_local_robot_y;
      robot_yaw = last_local_robot_yaw;
    }
  }

  {
    std::unique_lock<std::shared_mutex> local_lock(
        costmap_context->localCostmap()->mutex());

    if (!clearing_points.empty()) {
      const Eigen::Vector3d sensor_origin = T_local.translation();
      const auto* local_grid =
          costmap_context->localCostmap()->get().getCostmap();
      for (const auto& point : clearing_points) {
        const Eigen::Vector3d endpoint = T_local * point;
        local_obstacle_layer->raytraceFreespace(
            *local_grid, sensor_origin.x(), sensor_origin.y(), endpoint.x(),
            endpoint.y());
      }
    }

    // Clear the previous + current frame observation window, removing old
    // obstacle marks and writing old inflation bands back to FREE. The
    // expansion margin is the inflation radius + one cell: the band can
    // reach up to ceil(radius/res) cells, and world->cell rounding would
    // exclude cells lying exactly on the radius boundary, so one extra cell
    // is added to guarantee boundary-band cells are cleared too. Afterwards
    // the InflationLayer fully recomputes from lethal sources within its
    // recompute window, leaving no stale band once an obstacle moves
    // away.
    if (has_prev_obstacles && clearing_points.empty()) {
      double clear_min_x = min_x;
      double clear_min_y = min_y;
      double clear_max_x = max_x;
      double clear_max_y = max_y;
      for (const auto& old : prev_obstacles_local) {
        clear_min_x = std::min(clear_min_x, old.x());
        clear_min_y = std::min(clear_min_y, old.y());
        clear_max_x = std::max(clear_max_x, old.x());
        clear_max_y = std::max(clear_max_y, old.y());
      }
      const double margin =
          local_inflation_radius + std::max(local_resolution, 0.0);
      local_obstacle_layer->clearArea(clear_min_x - margin,
                                       clear_min_y - margin,
                                       clear_max_x + margin,
                                       clear_max_y + margin);
    }

    for (const auto& w : world) {
      local_obstacle_layer->markObstacles(w.x(), w.y());
    }

    prev_obstacles_local = world;
    has_prev_obstacles = true;

    costmap_context->localCostmap()->updateMapUnlocked(robot_x, robot_y, robot_yaw);
    if (robotDiagDue(local_robot_diag_at)) {
      const auto* probe_grid =
          costmap_context->localCostmap()->get().getCostmap();
      spdlog::debug(
          "roll updateMap robot=({:.3f},{:.3f}) origin=({:.3f},{:.3f}) "
          "rolling={}",
          robot_x, robot_y, probe_grid->getOriginX(), probe_grid->getOriginY(),
          costmap_context->localCostmap()->isRollingWindow() ? 1 : 0);
      spdlog::debug("roll updateMap robot=({:.3f},{:.3f}) rolling={}", robot_x,
                    robot_y,
                    costmap_context->localCostmap()->isRollingWindow() ? 1 : 0);
    }
  }

  // (M2): the global rolling costmap also needs periodic updateMap to drive
  // its rolling window to follow the robot (previously only local was
  // updated, so the global window never translated and the start pose could
  // fall outside the window, failing worldToMap). The window center uses the
  // robot pose in the global frame (map).
  if (costmap_context->globalCostmap() && global_obstacle_layer) {
    Eigen::Isometry3d T_global_sensor = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d T_global_robot = Eigen::Isometry3d::Identity();
    bool global_stamp_ok = false;
    if (have_stamp &&
        robot_state_controller.lookupTransform(
            costmap_context->getGlobalFrame(), frame_id, stamp, T_global_sensor,
            0.0)) {
      global_stamp_ok = true;
      ++stamp_query_ok;
    }
    const bool have_sensor_transform =
        (points.empty() && clearing_points.empty()) || global_stamp_ok ||
        robot_state_controller.lookupTransform(
            costmap_context->getGlobalFrame(), frame_id,
            ayanami::Timestamp::latest(), T_global_sensor,
            k_tf_lookup_timeout_s);
    if (!global_stamp_ok && have_stamp && have_sensor_transform &&
        !(points.empty() && clearing_points.empty())) {
      ++stamp_query_fallback;
    }
    if (!have_sensor_transform ||
        !robot_state_controller.lookupTransform(
            costmap_context->getGlobalFrame(), costmap_context->getRobotFrame(),
            ayanami::Timestamp::latest(), T_global_robot,
            k_tf_lookup_timeout_s)) {
      if (tfWarnDue(global_tf_warn_at)) {
        spdlog::warn(
            "CostmapNavigationServer::markObstaclesInFrame: no global "
            "transform {} <- {}/{} (stamp={:f}s); 全局障碍落图暂停。检查 "
            "AMCL map->odom 与 gazebo 差速 odom->base_footprint 是否存活",
            costmap_context->getGlobalFrame(), frame_id,
            costmap_context->getRobotFrame(), stamp.toSeconds());
      }
      return;
    }

    std::vector<Eigen::Vector2d> global_points;
    global_points.reserve(points.size());
    double global_min_x = std::numeric_limits<double>::max();
    double global_min_y = std::numeric_limits<double>::max();
    double global_max_x = std::numeric_limits<double>::lowest();
    double global_max_y = std::numeric_limits<double>::lowest();
    for (const auto& point : points) {
      const Eigen::Vector3d transformed = T_global_sensor * point;
      global_points.emplace_back(transformed.x(), transformed.y());
      global_min_x = std::min(global_min_x, transformed.x());
      global_min_y = std::min(global_min_y, transformed.y());
      global_max_x = std::max(global_max_x, transformed.x());
      global_max_y = std::max(global_max_y, transformed.y());
    }

    const double gx = T_global_robot.translation().x();
    const double gy = T_global_robot.translation().y();
    const double gyaw = std::atan2(T_global_robot.linear()(1, 0),
                                   T_global_robot.linear()(0, 0));
    std::unique_lock<std::shared_mutex> global_lock(
        costmap_context->globalCostmap()->mutex());

    if (!clearing_points.empty()) {
      const Eigen::Vector3d sensor_origin = T_global_sensor.translation();
      const auto* global_grid =
          costmap_context->globalCostmap()->get().getCostmap();
      for (const auto& point : clearing_points) {
        const Eigen::Vector3d endpoint = T_global_sensor * point;
        global_obstacle_layer->raytraceFreespace(
            *global_grid, sensor_origin.x(), sensor_origin.y(), endpoint.x(),
            endpoint.y());
      }
    }

    if (has_prev_obstacles_global) {
      for (const auto& old : prev_obstacles_global) {
        global_min_x = std::min(global_min_x, old.x());
        global_min_y = std::min(global_min_y, old.y());
        global_max_x = std::max(global_max_x, old.x());
        global_max_y = std::max(global_max_y, old.y());
      }
      const double margin =
          global_inflation_radius + std::max(global_resolution, 0.0);
      if (clearing_points.empty()) {
        global_obstacle_layer->clearArea(
            global_min_x - margin, global_min_y - margin,
            global_max_x + margin, global_max_y + margin);
      }
    }
    for (const auto& point : global_points) {
      global_obstacle_layer->markObstacles(point.x(), point.y());
    }
    prev_obstacles_global = std::move(global_points);
    has_prev_obstacles_global = true;
    costmap_context->globalCostmap()->updateMapUnlocked(gx, gy, gyaw);
  }

 // Scan-stamp query fallback rate observation (a fallback means that
 // frame's sensor transform degraded to latest time semantics, which carries
 // an omega x dt stamping skew when rotating): a persistently high fallback
 // rate indicates the scan stamp deviates from the sync chain beyond the
 // fast_tf pruning window (check RTF / TF sync health).
  if (stamp_query_fallback.load() > 0) {
    const auto diag_now = std::chrono::steady_clock::now();
    if (diag_now - last_stamp_diag_at >= std::chrono::seconds(10)) {
      last_stamp_diag_at = diag_now;
      spdlog::info(
          "markObstacles: stamp-queries ok={} fallback={}",
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

 // Local map: when rolling, center the window on the latest pose. Pose
 // resolution happens before taking the lock; on query failure, reuse the
 // last successful pose from markObstacles (never fall back to the origin).
  if (local_roll_period.count() > 0.0 && costmap_context->localCostmap() &&
      costmap_context->localCostmap()->isRollingWindow() &&
      now - last_local_roll >= local_roll_period) {
    last_local_roll = now;
    const std::string& local_frame =
        costmap_context->localCostmap()->getGlobalFrame();
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
      std::unique_lock<std::shared_mutex> lock(
          costmap_context->localCostmap()->mutex());
      costmap_context->localCostmap()->updateMapUnlocked(
          last_local_robot_x, last_local_robot_y, last_local_robot_yaw);
    }
  }

 // Global map: only needed for rolling configs (production uses a static map, skipped).
  if (global_roll_period.count() > 0.0 && costmap_context->globalCostmap() &&
      costmap_context->globalCostmap()->isRollingWindow() &&
      now - last_global_roll >= global_roll_period) {
    last_global_roll = now;
    Eigen::Isometry3d t_robot = Eigen::Isometry3d::Identity();
    if (robot_state_controller.lookupTransform(
            costmap_context->getGlobalFrame(),
            costmap_context->getRobotFrame(), ayanami::Timestamp::latest(),
            t_robot, k_tf_lookup_timeout_s)) {
      const double gx = t_robot.translation().x();
      const double gy = t_robot.translation().y();
      const double gyaw =
          std::atan2(t_robot.linear()(1, 0), t_robot.linear()(0, 0));
      std::unique_lock<std::shared_mutex> lock(
          costmap_context->globalCostmap()->mutex());
      costmap_context->globalCostmap()->updateMapUnlocked(gx, gy, gyaw);
    }
  }
}

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
    ok = clearOne(costmap_context->globalCostmap(),
                   costmap_context->getGlobalFrame()) && ok;
  }
  if (clear_local) {
    const std::string local_frame =
        costmap_context->localCostmap()
            ? costmap_context->localCostmap()->getGlobalFrame()
            : std::string{};
    ok = clearOne(costmap_context->localCostmap(), local_frame) && ok;
  }
  return ok;
}

}  // namespace costmap
}  // namespace ayanami
