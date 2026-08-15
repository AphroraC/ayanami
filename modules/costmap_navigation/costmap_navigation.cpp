// CostmapNavigation 宿主插件实现 (spec §5.6)。
//
// 抢占纪律 (spec §6.1): pilot_.run() 是常驻阻塞 tick 循环 (kill_switch_ 退出),
// 故 navigation_thread_ 只建一次并复用; 新 goal 抢占不重建树不重启线程, 只
// 先发零速度 (Global Constraint P9) + pilot_.halt() + controller cancelAll +
// beginNavigation (pilot 内部 haltTree + session_id_++ + pending_goal + 唤醒)。
//
// Ruling #5 (load-bearing): pilot wiring 显式三连 configure → loadFromFile →
// setNavigationServer; 树只建一次 (16 板项校验通过), 后续 goal 只 applyPendingGoal。
//
// Ruling #3 析构链: ~CostmapNavigation → stop() join 导航线程 → navigation_server_
// 成员析构 (CostmapNavigationServer 移除 obstacle 回调 → 基类 NavigationServerBase
// 先 clearActions 后 clearPlugins); BT 树 (pilot_ 成员, 声明在 server 之后) 先析构,
// 故 BT 节点不再持有过期插件指针。

#include "ayanami/modules/costmap_navigation/costmap_navigation.hpp"

#include <spdlog/spdlog.h>

#include <filesystem>
#include <utility>

#include "ayanami/interfaces/callbacks.hpp"
#include "ayanami/utility/config.hpp"

namespace ayanami {
namespace modules {

namespace {
constexpr const char* kDefaultTreeId =
    "NavigateWithReplanningAndSmoothing";
}  // namespace

CostmapNavigation::CostmapNavigation() = default;

CostmapNavigation::~CostmapNavigation() { stop(); }

void CostmapNavigation::initialize(const std::string& config_path) {
  GlobalConfig::instance(config_path, /*override_path=*/true);

  navigation_server_.configureCoreMaps(config_path);
  navigation_server_.loadPlugins(config_path);  // 触发 4 虚 initialize*Plugin
  navigation_server_.createActions();

  pilot_.configure(config_path);

  Config nav_cfg(GlobalConfig::get_config_path("config_navigation"));
  const std::string xml_rel = nav_cfg.param<std::string>(
      "navigation", "behavior_tree_xml", "BT/navigation.xml");
  const std::string tree_id = nav_cfg.param<std::string>(
      "navigation", "behavior_tree_id", kDefaultTreeId);
  const std::string xml_path =
      (std::filesystem::path(config_path) / xml_rel).string();

  // Ruling #5: 显式 wiring (pilot configure 只读配置, 不自动 load/set)。
  pilot_.loadFromFile(xml_path, tree_id);
  pilot_.setNavigationServer(&navigation_server_);

  visualization_rate_ = nav_cfg.param<double>(
      "navigation", "costmap_visualization_rate", 1.0);
}

void CostmapNavigation::start() {
  navigation_server_.start();
  last_publish_ = std::chrono::steady_clock::now();
  publishMapsForVisualization();
}

void CostmapNavigation::stop() {
  if (navigation_thread_ && navigation_thread_->joinable()) {
    pilot_.halt();
    pilot_.stop();  // kill_switch_ + 唤醒 run → run() 退出
    auto t = std::move(navigation_thread_);
    navigation_thread_.reset();
    t->join();
  }
  navigation_server_.stop();
}

void CostmapNavigation::spinOnce() {
  const auto now = std::chrono::steady_clock::now();
  const double hz = visualization_rate_ > 0.0 ? visualization_rate_ : 1.0;
  if (now - last_publish_ >= std::chrono::duration<double>(1.0 / hz)) {
    last_publish_ = now;
    publishMapsForVisualization();
  }
}

void CostmapNavigation::setGoal(const ayanami::geometry_msgs::PoseStamped& goal) {
  std::lock_guard<std::mutex> lk(goal_mutex_);
  if (navigation_thread_ && navigation_thread_->joinable()) {
    // 抢占 (spec §6.1): 先发零速度 (Global Constraint P9, 全字段清零)。
    geometry_msgs::TwistStamped stop = geometry_msgs::TwistStamped{};
    stop.twist = geometry_msgs::Twist{};
    ayanami::interfaces::NavigationCallbacks::on_velocity_command(stop);
    pilot_.halt();
    if (navigation_server_.controllerAction()) {
      navigation_server_.controllerAction()->cancelAll();
    }
  }
  current_goal_ = goal;
  state_ = RUNNING;
  // pilot 内部 haltTree + session_id_++ + 写 pending_goal_ + emitWakeUpSignal。
  pilot_.beginNavigation(goal);
  if (!navigation_thread_) {
    navigation_thread_ = std::make_unique<std::thread>([this] { pilot_.run(); });
  }
}

void CostmapNavigation::publishMapsForVisualization() {
  auto publish = [this](const std::string& name,
                        const costmap_2d::CostmapWrapper::Ptr& wrapper) {
    if (!wrapper) {
      return;
    }
    // R1: 读 costmap master grid 前持 wrapper 的 recursive_mutex (与
    // CostmapPlannerExecution::run 同款模式, costmap_planner_execution.hpp)。
    // ROS host 接入后 handleObstaclePointCloud 并发写 SimpleObstacleLayer,
    // 若 viz 线程无锁读 cellloop → data race。recursive_mutex 允许同线程
    // 重入, 锁内不再嵌套加锁。
    std::lock_guard<std::recursive_mutex> lock(wrapper->mutex());
    costmap_2d::Costmap2D* cm = wrapper->get().getCostmap();
    if (!cm) {
      return;
    }
    ayanami::nav_msgs::OccupancyGrid grid;
    grid.header.frame_id = wrapper->getGlobalFrame();
    grid.info.resolution = cm->getResolution();
    grid.info.width = cm->getSizeInCellsX();
    grid.info.height = cm->getSizeInCellsY();
    grid.info.origin = ayanami::geometry_msgs::Pose::Identity();
    grid.info.origin.translation() =
        Eigen::Vector3d(cm->getOriginX(), cm->getOriginY(), 0.0);
    grid.data.resize(static_cast<std::size_t>(grid.info.width) *
                     static_cast<std::size_t>(grid.info.height));
    for (unsigned int y = 0; y < grid.info.height; ++y) {
      for (unsigned int x = 0; x < grid.info.width; ++x) {
        const auto cost = cm->getCost(x, y);
        std::size_t idx = static_cast<std::size_t>(y) * grid.info.width + x;
        if (cost == costmap_2d::NO_INFORMATION) {
          grid.data[idx] = -1;
        } else if (costmap_2d::isLethal(cost)) {
          grid.data[idx] = 100;
        } else {
          grid.data[idx] = static_cast<int8_t>(
              (static_cast<int>(cost) * 100) / 252);
        }
      }
    }
    if (map_callback_) {
      map_callback_(name, grid);
    }
    ayanami::interfaces::NavigationCallbacks::on_costmap_visualization(name,
                                                                       grid);
  };
  publish("global_costmap", navigation_server_.globalCostmap());
  publish("local_costmap", navigation_server_.localCostmap());
}

}  // namespace modules
}  // namespace ayanami
