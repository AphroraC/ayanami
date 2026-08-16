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
  Config cfg(GlobalConfig::get_config_path("config_costmap"));

  const std::string global_frame =
      cfg.param<std::string>("costmap", "global_frame", "map");
  const std::string local_frame =
      cfg.param<std::string>("costmap", "local_frame", "odom");
  // 3.2 minor carry (M2-P6): robot_frame 单一来源 = costmap_navigation.json
  // 的 navigation::robot_frame; costmap.json 不再承载该字段。
  Config nav_cfg(GlobalConfig::get_config_path("config_navigation"));
  const std::string robot_frame =
      nav_cfg.param<std::string>("navigation", "robot_frame", "base_link");
  const double resolution =
      cfg.param<double>("costmap", "resolution", kDefaultResolution);

  // 3.2 (T10 前瞻, 本 task 核心 scope): 与 config_ros.json::robot_frame 一致性
  // 校验。config_ros 未载 (GlobalConfig 无 ayanami_ros 段) 时跳过。
  const std::string ros_frame = GlobalConfig::instance()->param<std::string>(
      "ayanami_ros", "robot_frame", std::string{});
  if (!ros_frame.empty()) {
    if (robot_frame != ros_frame) {
      spdlog::error("robot_frame mismatch: navigation '{}' vs ayanami_ros '{}'",
                    robot_frame, ros_frame);
    }
    assert(robot_frame == ros_frame);
  }

  // ---- global: StaticLayer + InflationLayer ----
  auto global_lm = std::make_shared<costmap_2d::LayeredCostmap>(
      global_frame,
      cfg.param<bool>("costmap_global", "rolling_window", false),
      cfg.param<bool>("costmap_global", "track_unknown", false));
  const int global_size_x =
      cfg.param<int>("costmap_global", "size_x", kDefaultSizeX);
  const int global_size_y =
      cfg.param<int>("costmap_global", "size_y", kDefaultSizeY);
  if (global_size_x > 0 && global_size_y > 0) {
    global_lm->resizeMap(static_cast<unsigned int>(global_size_x),
                         static_cast<unsigned int>(global_size_y), resolution,
                         0.0, 0.0);
  }

  auto static_layer = std::make_shared<costmap_2d_layers::StaticLayer>();
  const std::string static_map =
      cfg.param<std::string>("costmap_global", "static_map", "");
  if (!static_map.empty()) {
    try {
      static_layer->setMapPath(static_map, resolution, 0.0, 0.0);
    } catch (const std::exception& e) {
      spdlog::error("CostmapNavigationServer::configureCoreMaps: {}", e.what());
    }
  }
  global_lm->addLayer(static_layer);

  auto global_inflation = std::make_shared<costmap_2d_layers::InflationLayer>();
  global_inflation->setInflationRadius(cfg.param<double>(
      "costmap_global", "inflation_radius", kDefaultInflationRadius));
  global_inflation->setInscribedRadius(cfg.param<double>(
      "costmap_global", "inscribed_radius", kDefaultInscribedRadius));
  global_inflation->setCostScalingFactor(cfg.param<double>(
      "costmap_global", "cost_scaling_factor", kDefaultCostScalingFactor));
  global_lm->addLayer(global_inflation);

  auto global_wrapper =
      std::make_shared<costmap_2d::CostmapWrapper>(global_lm, global_frame);

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

  obstacle_layer_ = obstacle_layer.get();
  local_inflation_radius_ = local_inflation_radius;
  local_resolution_ = resolution;

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
  prev_obstacles_local_.clear();
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
  return std::make_unique<CostmapControllerExecution>(name, std::move(ptr),
                                                      std::move(wrapper));
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
  // R1 (reviewer I1): 并发传感器回调串行化。局部 wrapper 的 recursive_mutex
  // 保护 obstacle_layer_ 的 marks_/clear_regions_、prev_obstacles_local_ 与
  // local master grid 的访问 (同一帧内只写 local costmap, global 不涉及)。
  // recursive_mutex 允许同线程重入 (updateMap 内部再锁不重锁死)。
  std::lock_guard<std::recursive_mutex> lock(
      costmap_context_->local_costmap_->mutex());

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
  markObstaclesInFrame(cloud.header.frame_id, points);
}

void CostmapNavigationServer::handleObstacleLaserScan(
    const ayanami::sensor_msgs::LaserScan& scan) {
  if (!costmap_context_ || !costmap_context_->local_costmap_) {
    spdlog::warn(
        "CostmapNavigationServer::handleObstacleLaserScan: costmap not "
        "configured (call configureCoreMaps first)");
    return;
  }
  // R1 (reviewer I1): 与 handleObstaclePointCloud 同一把 local wrapper 锁。
  std::lock_guard<std::recursive_mutex> lock(
      costmap_context_->local_costmap_->mutex());

  if (scan.ranges.empty()) {
    return;
  }
  std::vector<Eigen::Vector3d> points;
  points.reserve(scan.ranges.size());
  for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
    const double range = static_cast<double>(scan.ranges[i]);
    // 过滤 NaN/Inf 与出界 range
    if (!(range >= scan.range_min && range <= scan.range_max)) {
      continue;
    }
    const double angle = scan.angle_min + i * scan.angle_increment;
    points.emplace_back(range * std::cos(angle), range * std::sin(angle), 0.0);
  }
  markObstaclesInFrame(scan.header.frame_id, points);
}

void CostmapNavigationServer::markObstaclesInFrame(
    const std::string& frame_id, const std::vector<Eigen::Vector3d>& points) {
  if (points.empty()) {
    return;
  }
  if (!costmap_context_ || !costmap_context_->local_costmap_ ||
      !obstacle_layer_) {
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
  const auto stamp = ayanami::interfaces::TransformStore::fromSeconds(0.0);

  Eigen::Isometry3d T_local = Eigen::Isometry3d::Identity();
  if (!store->lookupTransform(local_frame, frame_id, stamp, T_local)) {
    spdlog::warn("CostmapNavigationServer::markObstaclesInFrame: no transform "
                 "{} <- {}",
                 local_frame, frame_id);
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

    // Ruling #7: 清上一帧 + 本帧观察窗口, 移除旧障碍标记并把旧膨胀带写回
    // FREE。外扩 margin 取 inflation radius + 一个 cell: 膨胀带最远可达
    // ceil(radius/res) 个 cell, 世界→cell 取整会把正好压在 radius 边界的
    // cell 排除, 故多扩一个 cell 保证边界带 cell 一并清掉。之后
    // InflationLayer 在 recompute window 内从 lethal 源全量重算, 障碍移走后
    // 无 stale 带。
    if (has_prev_obstacles_) {
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
      obstacle_layer_->clearArea(clear_min_x - margin, clear_min_y - margin,
                                 clear_max_x + margin, clear_max_y + margin);
    }

  for (const auto& w : world) {
    obstacle_layer_->markObstacles(w.x(), w.y());
  }

  prev_obstacles_local_ = std::move(world);
  has_prev_obstacles_ = true;

  // 触发 local updateMap (rolling origin 平移 + 各层 updateBounds/updateCosts)。
  double robot_x = 0.0;
  double robot_y = 0.0;
  double robot_yaw = 0.0;
  Eigen::Isometry3d T_robot = Eigen::Isometry3d::Identity();
  if (store->lookupTransform(local_frame, costmap_context_->robot_frame_, stamp,
                             T_robot)) {
    robot_x = T_robot.translation().x();
    robot_y = T_robot.translation().y();
    robot_yaw =
        std::atan2(T_robot.linear()(1, 0), T_robot.linear()(0, 0));
  }
  costmap_context_->local_costmap_->updateMap(robot_x, robot_y, robot_yaw);
}

}  // namespace costmap
}  // namespace ayanami
