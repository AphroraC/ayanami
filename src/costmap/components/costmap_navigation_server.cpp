// CostmapNavigationServer 主实现 (spec §5.4)。
//
// 生命周期纪律 (spec §4.5 / Ruling #3): 派生析构先移除 obstacle 回调
// (回调捕获 this), 再交给基类析构 stop() → reset actions → clearPlugins。
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

#include "ayanami/interfaces/callbacks.hpp"
#include "ayanami/interfaces/transform_store.hpp"
#include "ayanami/utility/config.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <limits>
#include <mutex>
#include <utility>

namespace ayanami {
namespace costmap {

namespace {
constexpr double kDefaultResolution = 0.05;
constexpr double kDefaultInflationRadius = 0.6;
constexpr double kDefaultInscribedRadius = 0.5;
constexpr double kDefaultCostScalingFactor = 10.0;
constexpr int kDefaultSizeX = 0;
constexpr int kDefaultSizeY = 0;
}  // namespace

CostmapNavigationServer::CostmapNavigationServer() {
  pointcloud_cb_id_ =
      ayanami::interfaces::NavigationCallbacks::on_obstacle_pointcloud.add(
          [this](const ayanami::sensor_msgs::PointCloud2& cloud) {
            handleObstaclePointCloud(cloud);
          });
  laserscan_cb_id_ =
      ayanami::interfaces::NavigationCallbacks::on_obstacle_laserscan.add(
          [this](const ayanami::sensor_msgs::LaserScan& scan) {
            handleObstacleLaserScan(scan);
          });
}

CostmapNavigationServer::~CostmapNavigationServer() {
  if (pointcloud_cb_id_ >= 0) {
    ayanami::interfaces::NavigationCallbacks::on_obstacle_pointcloud.remove(
        pointcloud_cb_id_);
  }
  if (laserscan_cb_id_ >= 0) {
    ayanami::interfaces::NavigationCallbacks::on_obstacle_laserscan.remove(
        laserscan_cb_id_);
  }
}

void CostmapNavigationServer::configureCoreMaps(const std::string& config_path) {
  // 3.3 minor carry: 二次 configureCoreMaps 守护。已有已构 context 时忽略重入:
  // 重建会替换 costmap_context_, 使已注入插件的 wrapper 引用悬挂 (旧 context 被
  // 新对象替换后, 插件仍指向旧 wrapper)。
  if (costmap_context_ &&
      (costmap_context_->global_costmap_ || costmap_context_->local_costmap_)) {
    spdlog::warn(
        "configureCoreMaps called twice, ignoring; existing plugins would be "
        "orphaned by context replace");
    return;
  }

  GlobalConfig::instance(config_path, /*override_path=*/true);
  // costmap 参数已并入 config_navigation 文件 (原 costmap.json 移除)。
  Config nav_cfg(GlobalConfig::get_config_path("config_navigation"));
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
      cfg.param<double>("costmap", "resolution", kDefaultResolution);

  // 3.2 (T10 前瞻, 本 task 核心 scope): 与 config_ros.json::robot_frame 一致性
  // 校验。config_ros 未载 (GlobalConfig 无 config_ros 映射) 时跳过。
  const std::string ros_config =
      GlobalConfig::get_config_path("config_ros");
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
        cfg.param<int>("costmap_global", "size_x", kDefaultSizeX);
    const int fallback_size_y =
        cfg.param<int>("costmap_global", "size_y", kDefaultSizeY);
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
      "costmap_global", "inflation_radius", kDefaultInflationRadius);
  global_inflation->setInflationRadius(global_inflation_radius);
  global_inflation->setInscribedRadius(cfg.param<double>(
      "costmap_global", "inscribed_radius", kDefaultInscribedRadius));
  global_inflation->setCostScalingFactor(cfg.param<double>(
      "costmap_global", "cost_scaling_factor", kDefaultCostScalingFactor));
  global_lm->addLayer(global_inflation);

  // ---- local: SimpleObstacleLayer + InflationLayer (rolling) ----
  auto local_lm = std::make_shared<costmap_2d::LayeredCostmap>(
      local_frame,
      cfg.param<bool>("costmap_local", "rolling_window", true),
      cfg.param<bool>("costmap_local", "track_unknown", false));
  const int local_size_x =
      cfg.param<int>("costmap_local", "size_x", kDefaultSizeX);
  const int local_size_y =
      cfg.param<int>("costmap_local", "size_y", kDefaultSizeY);
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
      "costmap_local", "inflation_radius", kDefaultInflationRadius);
  local_inflation->setInflationRadius(local_inflation_radius);
  local_inflation->setInscribedRadius(cfg.param<double>(
      "costmap_local", "inscribed_radius", kDefaultInscribedRadius));
  local_inflation->setCostScalingFactor(cfg.param<double>(
      "costmap_local", "cost_scaling_factor", kDefaultCostScalingFactor));
  local_lm->addLayer(local_inflation);

  auto local_wrapper =
      std::make_shared<costmap_2d::CostmapWrapper>(local_lm, local_frame);

  const auto footprint_values = cfg.param<std::vector<Eigen::Vector2d>>(
      "costmap", "footprint", {});
  costmap_2d::Footprint footprint;
  footprint.reserve(footprint_values.size());
  for (const auto& point : footprint_values) {
    footprint.push_back(costmap_2d::Point{point.x(), point.y(), 0.0});
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

  global_obstacle_layer_ = global_obstacle.get();
  local_obstacle_layer_ = obstacle_layer.get();
  global_inflation_radius_ = global_inflation_radius;
  local_inflation_radius_ = local_inflation_radius;
  global_resolution_ = resolution;
  local_resolution_ = resolution;
  local_raytrace_range_ = cfg.param<double>(
      "costmap_local", "raytrace_range", local_raytrace_range_);

  costmap_context_ = std::make_shared<CostmapContext>();
  costmap_context_->global_costmap_ = global_wrapper;
  costmap_context_->local_costmap_ = local_wrapper;
  costmap_context_->global_frame_ = global_frame;
  costmap_context_->robot_frame_ = robot_frame;
  costmap_context_->transform_store_ =
      &ayanami::interfaces::TransformStore::instance();
  map_context_ = costmap_context_;  // 注入 AbstractMapContext 层

  // 3.2 minor carry: RobotStateProvider 也用同一 robot_frame (单一来源)。
  robot_state_provider_.setFrames(global_frame, local_frame, robot_frame);

  planner_name_to_costmap_.clear();
  controller_name_to_costmap_.clear();
  prev_obstacles_global_.clear();
  prev_obstacles_local_.clear();
  has_prev_obstacles_global_ = false;
  has_prev_obstacles_ = false;
}

// ---- 虚 initialize*Plugin: costmap 族注入 ----

bool CostmapNavigationServer::initializePlannerPlugin(
    const std::string& name, ayanami::abstract::AbstractPlanner::Ptr ptr) {
  if (!costmap_context_) {
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
  cast->initialize(name, costmap_context_);
  planner_name_to_costmap_[name] = costmap_context_->global_costmap_;
  return true;
}

bool CostmapNavigationServer::initializeSmootherPlugin(
    const std::string& name, ayanami::abstract::AbstractSmoother::Ptr ptr) {
  if (!costmap_context_) {
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
  cast->initialize(name, costmap_context_);
  return true;
}

bool CostmapNavigationServer::initializeControllerPlugin(
    const std::string& name,
    ayanami::abstract::AbstractController::Ptr ptr) {
  if (!costmap_context_) {
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
  cast->initialize(name, costmap_context_);
  controller_name_to_costmap_[name] = costmap_context_->local_costmap_;
  return true;
}

bool CostmapNavigationServer::initializeRecoveryPlugin(
    const std::string& name, ayanami::abstract::AbstractRecovery::Ptr ptr) {
  if (!costmap_context_) {
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
  cast->initialize(name, costmap_context_);
  return true;
}

// ---- 虚 new*Execution 工厂: 返回带地图的 Costmap 版 ----

std::unique_ptr<ayanami::components::PlannerExecution>
CostmapNavigationServer::newPlannerExecution(
    const std::string& name, ayanami::abstract::AbstractPlanner::Ptr ptr) {
  costmap_2d::CostmapWrapper::Ptr wrapper;
  auto iter = planner_name_to_costmap_.find(name);
  if (iter != planner_name_to_costmap_.end()) {
    wrapper = iter->second;
  } else if (costmap_context_) {
    wrapper = costmap_context_->global_costmap_;
  }
  return std::make_unique<CostmapPlannerExecution>(name, std::move(ptr),
                                                   std::move(wrapper));
}

std::unique_ptr<ayanami::components::ControllerExecution>
CostmapNavigationServer::newControllerExecution(
    const std::string& name,
    ayanami::abstract::AbstractController::Ptr ptr) {
  costmap_2d::CostmapWrapper::Ptr wrapper;
  auto iter = controller_name_to_costmap_.find(name);
  if (iter != controller_name_to_costmap_.end()) {
    wrapper = iter->second;
  } else if (costmap_context_) {
    wrapper = costmap_context_->local_costmap_;
  }
  auto exec = std::make_unique<CostmapControllerExecution>(
      name, std::move(ptr), std::move(wrapper));
  // Task 12 stage C: 控制器真实机器人位姿源 —— 默认 identity pose 会让 pid
  // 从地图原点跟踪路径 (机器人实际在 (0.07,-0.14)), 且使 goal_checker 误判。
  // 经 CostmapContext 查 TransformStore map←robot_frame (与 planner fallback
  // 同一来源); 速度暂缺, 返回零速度 (pid 以位姿为主)。
  if (costmap_context_) {
    auto ctx = costmap_context_;
    exec->setRobotStateProvider(
        [ctx](ayanami::geometry_msgs::PoseStamped& pose,
              ayanami::geometry_msgs::TwistStamped& velocity) {
          if (!ctx->getRobotPose(pose)) {
            return false;
          }
          velocity = ayanami::geometry_msgs::TwistStamped{};
          return true;
        });
  }
  return exec;
}

std::unique_ptr<ayanami::components::RecoveryExecution>
CostmapNavigationServer::newRecoveryExecution(
    const std::string& name, ayanami::abstract::AbstractRecovery::Ptr ptr) {
  costmap_2d::CostmapWrapper::Ptr global;
  costmap_2d::CostmapWrapper::Ptr local;
  if (costmap_context_) {
    global = costmap_context_->global_costmap_;
    local = costmap_context_->local_costmap_;
  }
  return std::make_unique<CostmapRecoveryExecution>(
      name, std::move(ptr), std::move(global), std::move(local));
}

// ---- 障碍观测落图 ----

void CostmapNavigationServer::handleObstaclePointCloud(
    const ayanami::sensor_msgs::PointCloud2& cloud) {
  if (!costmap_context_ || !costmap_context_->local_costmap_) {
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
  if (!costmap_context_ || !costmap_context_->local_costmap_) {
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
            ? std::clamp(range, 0.0, local_raytrace_range_)
            : local_raytrace_range_;
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
  if (!costmap_context_ || !costmap_context_->local_costmap_ ||
      !local_obstacle_layer_) {
    spdlog::warn(
        "CostmapNavigationServer::markObstaclesInFrame: costmap not "
        "configured (call configureCoreMaps first)");
    return;
  }
  auto* store = costmap_context_->transform_store_;
  if (!store) {
    spdlog::warn("CostmapNavigationServer::markObstaclesInFrame: no "
                 "transform_store injected");
    return;
  }
  const std::string& local_frame =
      costmap_context_->local_costmap_->getGlobalFrame();

  Eigen::Isometry3d T_local = Eigen::Isometry3d::Identity();
  if ((!points.empty() || !clearing_points.empty()) &&
      !store->lookupTransform(local_frame, frame_id, stamp, T_local)) {
    spdlog::warn("CostmapNavigationServer::markObstaclesInFrame: no transform "
                 "{} <- {} (stamp={:f}s)",
                 local_frame, frame_id, stamp.toSeconds());
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
  double robot_x = 0.0;
  double robot_y = 0.0;
  double robot_yaw = 0.0;
  Eigen::Isometry3d T_robot = Eigen::Isometry3d::Identity();
  if (store->lookupTransform(local_frame, costmap_context_->robot_frame_, stamp,
                             T_robot)) {
    robot_x = T_robot.translation().x();
    robot_y = T_robot.translation().y();
    robot_yaw = std::atan2(T_robot.linear()(1, 0), T_robot.linear()(0, 0));
  }

  {
    std::lock_guard<std::recursive_mutex> local_lock(
        costmap_context_->local_costmap_->mutex());

    if (!clearing_points.empty()) {
      const Eigen::Vector3d sensor_origin = T_local.translation();
      const auto* local_grid =
          costmap_context_->local_costmap_->get().getCostmap();
      for (const auto& point : clearing_points) {
        const Eigen::Vector3d endpoint = T_local * point;
        local_obstacle_layer_->raytraceFreespace(
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
    if (has_prev_obstacles_ && clearing_points.empty()) {
      double clear_min_x = min_x;
      double clear_min_y = min_y;
      double clear_max_x = max_x;
      double clear_max_y = max_y;
      for (const auto& old : prev_obstacles_local_) {
        clear_min_x = std::min(clear_min_x, old.x());
        clear_min_y = std::min(clear_min_y, old.y());
        clear_max_x = std::max(clear_max_x, old.x());
        clear_max_y = std::max(clear_max_y, old.y());
      }
      const double margin =
          local_inflation_radius_ + std::max(local_resolution_, 0.0);
      local_obstacle_layer_->clearArea(clear_min_x - margin,
                                       clear_min_y - margin,
                                       clear_max_x + margin,
                                       clear_max_y + margin);
    }

    for (const auto& w : world) {
      local_obstacle_layer_->markObstacles(w.x(), w.y());
    }

    prev_obstacles_local_ = world;
    has_prev_obstacles_ = true;

    // updateMap re-enters the wrapper's recursive mutex.
    costmap_context_->local_costmap_->updateMap(robot_x, robot_y, robot_yaw);
  }

  // Task 12 stage C (M2 Ruling #M2-4): global rolling costmap 也要周期性
  // updateMap 驱动滚动窗口跟随机器人 (此前只有 local 更新, global 窗口永不
  // 平移, 导致 start pose 落在窗口外 worldToMap 失败)。窗口中心用 global
  // frame (map) 下的机器人位姿。
  if (costmap_context_->global_costmap_ && global_obstacle_layer_) {
    Eigen::Isometry3d T_global_sensor = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d T_global_robot = Eigen::Isometry3d::Identity();
    const bool have_sensor_transform =
        (points.empty() && clearing_points.empty()) ||
        store->lookupTransform(costmap_context_->global_frame_, frame_id,
                               stamp, T_global_sensor);
    if (!have_sensor_transform ||
        !store->lookupTransform(costmap_context_->global_frame_,
                                costmap_context_->robot_frame_, stamp,
                                T_global_robot)) {
      spdlog::warn(
          "CostmapNavigationServer::markObstaclesInFrame: no global "
          "transform {} <- {}/{} (stamp={:f}s)",
          costmap_context_->global_frame_, frame_id,
          costmap_context_->robot_frame_, stamp.toSeconds());
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
    std::lock_guard<std::recursive_mutex> global_lock(
        costmap_context_->global_costmap_->mutex());

    if (!clearing_points.empty()) {
      const Eigen::Vector3d sensor_origin = T_global_sensor.translation();
      const auto* global_grid =
          costmap_context_->global_costmap_->get().getCostmap();
      for (const auto& point : clearing_points) {
        const Eigen::Vector3d endpoint = T_global_sensor * point;
        global_obstacle_layer_->raytraceFreespace(
            *global_grid, sensor_origin.x(), sensor_origin.y(), endpoint.x(),
            endpoint.y());
      }
    }

    if (has_prev_obstacles_global_) {
      for (const auto& old : prev_obstacles_global_) {
        global_min_x = std::min(global_min_x, old.x());
        global_min_y = std::min(global_min_y, old.y());
        global_max_x = std::max(global_max_x, old.x());
        global_max_y = std::max(global_max_y, old.y());
      }
      const double margin =
          global_inflation_radius_ + std::max(global_resolution_, 0.0);
      if (clearing_points.empty()) {
        global_obstacle_layer_->clearArea(
            global_min_x - margin, global_min_y - margin,
            global_max_x + margin, global_max_y + margin);
      }
    }
    for (const auto& point : global_points) {
      global_obstacle_layer_->markObstacles(point.x(), point.y());
    }
    prev_obstacles_global_ = std::move(global_points);
    has_prev_obstacles_global_ = true;
    costmap_context_->global_costmap_->updateMap(gx, gy, gyaw);
  }
}

}  // namespace costmap
}  // namespace ayanami
