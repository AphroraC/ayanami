// CostmapNavigationServer 主实现 (spec §5.4)。
//
// 生命周期纪律 (spec §4.5 / Ruling #3): 基类 NavigationServerBase 析构先
// stop() → reset actions → clearPlugins。
//
// 障碍观测纪律: 不再注册全局 NavigationCallbacks 输入槽 (输出/观测分层);
// 宿主 (CostmapNavigation::setPointcloud/setLaserScan) 直递 handleObstacle*。
//
// Ruling #7 (load-bearing): InflationLayer::updateCosts 只做 `cost > old_cost`
// 覆盖, 障碍消失后不会自动 de-inflate。handleObstacle* 每次动态更新前先
// clearArea 上一帧 + 本帧观察窗口 (外扩 inflation radius), 把旧障碍标记与旧
// 膨胀带在 master grid 中清为 FREE, 再 markObstacles + updateMap 让
// InflationLayer 在 recompute window 内全量重算, 无 stale 带残留。

#include "ayanami/costmap/components/costmap_navigation_server.hpp"

#include <costmap_2d_layers/inflation_layer.hpp>
#include <costmap_2d_layers/simple_obstacle_layer.hpp>
#include <costmap_2d_layers/static_layer.hpp>
#include <spdlog/spdlog.h>

#include "ayanami/abstract/components/robot_state_controller.hpp"
#include "ayanami/costmap/components/costmap_commands.hpp"
#include "ayanami/costmap/core/costmap_command.hpp"
#include "ayanami/interfaces/transform_store.hpp"
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
// (50 Hz) with their own stamps, while obstacle lookups use the sensor stamp.
// A zero tolerance lookup almost never matches a sampled stamp, so obstacles
// get dropped with "no transform". Use bounded tolerances: local odometry is
// published at ~100 Hz, AMCL map->odom at ~20 Hz with transform_tolerance 0.5.
constexpr double k_local_tf_tolerance_s = 0.2;
constexpr double k_global_tf_tolerance_s = 0.5;

// TF 链路缺失告警节流: 同一链路 (local/global) 的 "no transform" 每
// k_tf_warn_interval_s 最多输出一条, 避免 5.5Hz 激光每帧刷屏。返回 true 表示
// 本次应输出。
constexpr double k_tf_warn_interval_s = 5.0;
bool tfWarnDue(std::atomic<std::chrono::steady_clock::time_point>& last) {
  const auto now = std::chrono::steady_clock::now();
  auto prev = last.load();
  if (now - prev < std::chrono::duration<double>(k_tf_warn_interval_s)) {
    return false;
  }
  return last.compare_exchange_strong(prev, now);
}

// 命令端口实现: 把类型化写命令转交唯一写者 (本 server) 执行。恢复插件只
// 持 CostmapCommandPort 接口, 不接触可变 wrapper/layer。
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

void CostmapNavigationServer::configureCoreMaps(const std::string& config_path) {
  // 3.3 minor carry: 二次 configureCoreMaps 守护。已有已构 context 时忽略重入:
  // 重建会替换 costmap_context, 使已注入插件的 wrapper 引用悬挂 (旧 context 被
  // 新对象替换后, 插件仍指向旧 wrapper)。
  if (costmap_context &&
      (costmap_context->globalCostmap() || costmap_context->localCostmap())) {
    spdlog::warn(
        "configureCoreMaps called twice, ignoring; existing plugins would be "
        "orphaned by context replace");
    return;
  }

  GlobalConfig::instance(config_path, /*override_path=*/true);
  // costmap 参数已并入 config_navigation 文件 (原 costmap.json 移除)。
  Config nav_cfg(GlobalConfig::getConfigPath("config_navigation"));
  const Config& cfg = nav_cfg;

  const std::string global_frame =
      cfg.param<std::string>("costmap", "global_frame", "map");
  const std::string local_frame =
      cfg.param<std::string>("costmap", "local_frame", "odom");
  // 3.2 minor carry (M2-P6): robot_frame 单一来源 = costmap_navigation.json
  // 的 navigation::robot_frame。
  const std::string robot_frame =
      nav_cfg.param<std::string>("navigation", "robot_frame", "base_link");
  const double resolution =
      cfg.param<double>("costmap", "resolution", k_default_resolution);

  // 3.2 (T10 前瞻, 本 task 核心 scope): 与 config_ros.json::robot_frame 一致性
  // 校验。config_ros 未载 (GlobalConfig 无 config_ros 映射) 时跳过。
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
      // Task 12 stage C: 相对 static_map (assets/nav.pgm) 相对 package root
      // (config/ 的上一级) 解析, 不再依赖节点 cwd (roslaunch 下是 ~/.ros)。
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

  // 车体 footprint 来源 (nav_demo 参数语义):
  //   - costmap::footprint 多边形 (nav2 语义, 优先级最高)
  //   - costmap::robot_radius > 0 → 生成 16 边形近似圆 (ROS1 robot_radius 语义)
  //   - 两者都缺 → 不设置 (inscribed_radius 参数作为膨胀层 fallback)
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

  costmap_context = std::make_shared<CostmapContext>();
  costmap_context->setGlobalCostmap(global_wrapper);
  costmap_context->setLocalCostmap(local_wrapper);
  costmap_context->setFrames(global_frame, robot_frame);
  // footprint 缓存为不可变副本: 快照构建只读此缓存 (写锁外绝不读 live
  // LayeredCostmap 的 footprint)。
  costmap_context->setFootprint(footprint);

  // 3.2 minor carry: RobotStateProvider 也用同一 robot_frame (单一来源)。
  robot_state_provider.setFrames(global_frame, local_frame, robot_frame);

  // 全局 RobotStateController 一次性配置: 帧名 + 速度超时只在此缓存,
  // hot getRobotState() 永不读 Config (spec §5.3 扩展)。
  const double velocity_timeout =
      nav_cfg.param<double>("navigation", "velocity_timeout", 0.25);
  ayanami::components::RobotStateController::instance().initialize(
      global_frame, local_frame, robot_frame, velocity_timeout);

  prev_obstacles_global.clear();
  prev_obstacles_local.clear();
  has_prev_obstacles_global = false;
  has_prev_obstacles = false;
}

// ---- 虚 initialize*Plugin: costmap 族注入 ----

bool CostmapNavigationServer::initializePlannerPlugin(
    const std::string& name, ayanami::abstract::AbstractPlanner::Ptr ptr) {
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
  cast->initialize(name, costmap_context->getSharedConstPtr());
  cast->configure();
  return true;
}

bool CostmapNavigationServer::initializeSmootherPlugin(
    const std::string& name, ayanami::abstract::AbstractSmoother::Ptr ptr) {
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
  cast->initialize(name, costmap_context->getSharedConstPtr());
  cast->configure();
  return true;
}

bool CostmapNavigationServer::initializeControllerPlugin(
    const std::string& name,
    ayanami::abstract::AbstractController::Ptr ptr) {
  if (!costmap_context) {
    spdlog::error(
        "CostmapNavigationServer::initializeControllerPlugin('{}'): costmap "
        "context not configured (call configureCoreMaps first)",
        name);
    return false;
  }
  // 严格 costmap 族约束: 只接受 CostmapController, 不再放行普通
  // AbstractController (costmap_pid 已改继承 CostmapController)。
  auto cast = std::dynamic_pointer_cast<CostmapController>(ptr);
  if (!cast) {
    spdlog::error(
        "CostmapNavigationServer::initializeControllerPlugin('{}'): plugin is "
        "not a CostmapController (costmap 族接口 mismatch)",
        name);
    return false;
  }
  cast->initialize(name, costmap_context->getSharedConstPtr());
  cast->configure();
  return true;
}

bool CostmapNavigationServer::initializeRecoveryPlugin(
    const std::string& name, ayanami::abstract::AbstractRecovery::Ptr ptr) {
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
  cast->initialize(name, costmap_context->getSharedConstPtr());
  cast->configure();
  // 地图变更仅经命令端口: 恢复插件拿到 port 后, 需要改图时提交 typed
  // 命令, 由本 server (唯一写者) 变更层并发布新 epoch。
  cast->setCommandPort(command_port);
  return true;
}

// ---- createActions: 生产路径 = Costmap*Action ----

void CostmapNavigationServer::createActions() {
  planner_actions.clear();
  smoother_actions.clear();
  controller_actions.clear();
  recovery_actions.clear();

  // planner/smoother: Costmap*Action (带 ctx 的 Costmap*Execution)。
  for (const auto& name : planner_plugins.getLoadedNames()) {
    auto planner = planner_plugins.getPlugin(name);
    if (!planner) {
      continue;
    }
    planner_actions[name] = std::make_unique<CostmapPlannerAction>(
        planner, costmap_context);
  }
  if (!planner_plugins.hasPlugin(default_planner)) {
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
  if (!smoother_plugins.hasPlugin(default_smoother)) {
    spdlog::error("CostmapNavigationServer::createActions: default smoother "
                  "'{}' not loaded",
                  default_smoother);
  }

  // controller: CostmapControllerExecution + checker/速度上限/真实位姿源装配。
  for (const auto& name : controller_plugins.getLoadedNames()) {
    auto controller = controller_plugins.getPlugin(name);
    if (!controller) {
      continue;
    }
    auto action = std::make_unique<CostmapControllerAction>(
        controller, costmap_context,
        [this](ayanami::components::ControllerExecution::Ptr exec) {
          configureControllerExecution(exec);
          // 控制器真实机器人位姿/速度源: 全局 RobotStateController 单例 ——
          // 位姿取最新 TF (TransformStore map←robot_frame), 速度取最新 odom
          // (过期→零速度)。替换原硬编码零速度 (速度不再恒零, pid/dwb 可消费
          // 真实线速度)。
          auto& rsc = ayanami::components::RobotStateController::instance();
          exec->setRobotStateProvider(
              [&rsc](ayanami::geometry_msgs::PoseStamped& pose,
                     ayanami::geometry_msgs::TwistStamped& velocity) {
                return rsc.getRobotState(pose, velocity);
              });
        });
    controller_actions[name] = std::move(action);
  }
  if (!controller_plugins.hasPlugin(default_controller)) {
    spdlog::error("CostmapNavigationServer::createActions: default controller "
                  "'{}' not loaded",
                  default_controller);
  }

  // recovery: CostmapRecoveryExecution + 速度上限装配。
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
  if (!recovery_plugins.hasPlugin(default_recovery)) {
    spdlog::error("CostmapNavigationServer::createActions: default recovery "
                  "'{}' not loaded",
                  default_recovery);
  }
}

// ---- 障碍观测落图 ----

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
  if ((!points.empty() || !clearing_points.empty()) &&
      !robot_state_controller.lookupTransform(local_frame, frame_id, stamp,
                                              T_local,
                                              k_local_tf_tolerance_s)) {
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
  // 查询失败 (TF 短暂缺失) 时使用最后一次成功位姿滚动窗口 —— 绝不回退
  // (0,0,0), 否则滚动窗口被锚回世界原点, 机器人走出窗口后 dwb 无路可走。
  double robot_x = 0.0;
  double robot_y = 0.0;
  double robot_yaw = 0.0;
  Eigen::Isometry3d T_robot = Eigen::Isometry3d::Identity();
  if (robot_state_controller.lookupTransform(
          local_frame, costmap_context->getRobotFrame(), stamp, T_robot,
          k_local_tf_tolerance_s)) {
    robot_x = T_robot.translation().x();
    robot_y = T_robot.translation().y();
    robot_yaw = std::atan2(T_robot.linear()(1, 0), T_robot.linear()(0, 0));
    has_last_local_robot = true;
    last_local_robot_x = robot_x;
    last_local_robot_y = robot_y;
    last_local_robot_yaw = robot_yaw;
  } else if (has_last_local_robot) {
    robot_x = last_local_robot_x;
    robot_y = last_local_robot_y;
    robot_yaw = last_local_robot_yaw;
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

    // Ruling #7: 清上一帧 + 本帧观察窗口, 移除旧障碍标记并把旧膨胀带写回
    // FREE。外扩 margin 取 inflation radius + 一个 cell: 膨胀带最远可达
    // ceil(radius/res) 个 cell, 世界→cell 取整会把正好压在 radius 边界的
    // cell 排除, 故多扩一个 cell 保证边界带 cell 一并清掉。之后
    // InflationLayer 在 recompute window 内从 lethal 源全量重算, 障碍移走后
    // 无 stale 带。
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
  }

  // Task 12 stage C (M2 Ruling #M2-4): global rolling costmap 也要周期性
  // updateMap 驱动滚动窗口跟随机器人 (此前只有 local 更新, global 窗口永不
  // 平移, 导致 start pose 落在窗口外 worldToMap 失败)。窗口中心用 global
  // frame (map) 下的机器人位姿。
  if (costmap_context->globalCostmap() && global_obstacle_layer) {
    Eigen::Isometry3d T_global_sensor = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d T_global_robot = Eigen::Isometry3d::Identity();
    const bool have_sensor_transform =
        (points.empty() && clearing_points.empty()) ||
        robot_state_controller.lookupTransform(
            costmap_context->getGlobalFrame(), frame_id, stamp,
            T_global_sensor, k_global_tf_tolerance_s);
    if (!have_sensor_transform ||
        !robot_state_controller.lookupTransform(
            costmap_context->getGlobalFrame(), costmap_context->getRobotFrame(),
            stamp, T_global_robot, k_global_tf_tolerance_s)) {
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
}

// ---- 地图变更命令执行 (唯一写者) ----

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
    // 唯一写者: 清空动态层 + 重算 + updateMap。任何时刻不把
    // 可变的层/主网格暴露给恢复插件。
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
