// CostmapNavigation 宿主插件实现 (spec §5.6)。
//
// 抢占纪律 (spec §6.1): bt_pilot.run() 是常驻阻塞 tick 循环 (kill_switch 退出),
// 故 navigation_thread 只建一次并复用。新 goal 由 BehaviorTreePilot 的 tick
// 线程更新 goal revision；controller 继续旧路径，planner 原位切换目标，新路径
// 完成后 controller 再原位更新。宿主 setGoal 只做非阻塞 latest-wins 投递。
//
// Ruling #5 (load-bearing): bt_pilot wiring 显式三连 configure → loadFromFile →
// setNavigationRuntime; 树只建一次 (17 板项校验通过), 后续 goal 只 applyPendingGoal。
//
// Ruling #3 析构链: ~CostmapNavigation → stop() join 导航线程 → navigation_server
// 成员析构 (CostmapNavigationServer 基类 NavigationServerBase 先 clearActions 后
// clearPlugins); BT 树 (bt_pilot 成员, 声明在 server 之后) 先析构, 故 BT 节点不再
// 持有过期插件指针。

#include "costmap_navigation.hpp"

#include <spdlog/spdlog.h>

#include <filesystem>
#include <shared_mutex>
#include <utility>

#include "ayanami/interfaces/callbacks.hpp"
#include "ayanami/utility/config.hpp"
#include "ayanami/utility/logging.hpp"

namespace ayanami {
namespace modules {

namespace {
constexpr const char* k_default_tree_id =
    "NavigateWithReplanningAndSmoothing";
}  // namespace

CostmapNavigation::CostmapNavigation()
    : navigation_server(std::make_shared<costmap::CostmapNavigationServer>()) {}

CostmapNavigation::~CostmapNavigation() { stop(); }

void CostmapNavigation::initialize(const std::string& config_path) {
  GlobalConfig::instance(config_path, /*override_path=*/true);
  ayanami::configureLogging();

  navigation_server->configureCoreMaps(config_path);
  // loadPlugins 内部已触发 4 虚 initialize*Plugin + createActions() (review 修复:
  // 不再显式二次调用 createActions, 避免 action map 重建)。
  navigation_server->loadPlugins(config_path);

  bt_pilot.configure(config_path);

  Config nav_cfg(GlobalConfig::getConfigPath("config_navigation"));
  // 生产单一来源 (review 修复): 默认导航树由 BehaviorTreeExtension 内嵌注册,
  // 不再从文件读取 (config/BT/navigation.xml 已删)。xml 空 → 空路径走扩展
  // 默认树; 测试仍可显式给 behavior_tree_xml 提供自定义 XML。
  const std::string xml_rel = nav_cfg.param<std::string>(
      "navigation", "behavior_tree_xml", "");
  const std::string tree_id = nav_cfg.param<std::string>(
      "navigation", "behavior_tree_id", k_default_tree_id);
  const std::string xml_path = xml_rel.empty()
                                   ? std::string{}
                                   : (std::filesystem::path(config_path) /
                                      xml_rel).string();

  // Ruling #5: 显式 wiring (bt_pilot configure 只读配置, 不自动 load/set)。
  bt_pilot.loadFromFile(xml_path, tree_id);
  bt_pilot.setNavigationRuntime(navigation_server);

  visualization_rate = nav_cfg.param<double>(
      "navigation", "costmap_visualization_rate", 1.0);
}

void CostmapNavigation::start() {
  navigation_server->start();
  last_publish = std::chrono::steady_clock::now();
  publishMapsForVisualization();
}

void CostmapNavigation::stop() {
  if (navigation_thread && navigation_thread->joinable()) {
    bt_pilot.halt();
    bt_pilot.stop();  // kill_switch + 唤醒 run → run() 退出
    auto t = std::move(navigation_thread);
    navigation_thread.reset();
    t->join();
  }
  if (navigation_server) navigation_server->stop();
}

void CostmapNavigation::spinOnce() {
  const auto now = std::chrono::steady_clock::now();
  const double hz = visualization_rate > 0.0 ? visualization_rate : 1.0;
  if (now - last_publish >= std::chrono::duration<double>(1.0 / hz)) {
    last_publish = now;
    publishMapsForVisualization();
  }
}

void CostmapNavigation::setGoal(const ayanami::geometry_msgs::PoseStamped& goal) {
  std::lock_guard<std::mutex> lk(goal_mutex);
  current_goal = goal;
  state = RUNNING;
  // 非阻塞邮箱投递 (规范: 宿主 setGoal 也 latest-wins 邮箱化):
  // bt_pilot 内部 pending_goal 最新覆盖; halt 旧树 / controller cancelAll /
  // 零速度 / 会话推进全部在 bt_pilot tick 线程 applyPendingGoal 完成, 不在
  // 调用线程 halt BT 树 (绝不跨线程 haltTree)。
  bt_pilot.beginNavigation(goal);
  if (!navigation_thread || !navigation_thread->joinable()) {
    navigation_thread = std::make_unique<std::thread>([this] { bt_pilot.run(); });
  }
}

void CostmapNavigation::setPointcloud(
    const ayanami::sensor_msgs::PointCloud2& cloud) {
  // 障碍观测输入直递 CostmapNavigationServer 公共落图管线 (不再走全局旁路槽)。
  navigation_server->handleObstaclePointCloud(cloud);
}

void CostmapNavigation::setLaserScan(
    const ayanami::sensor_msgs::LaserScan& scan) {
  navigation_server->handleObstacleLaserScan(scan);
}

void CostmapNavigation::publishMapsForVisualization() {
  auto publish = [this](const std::string& name,
                        const costmap_2d::CostmapWrapper::Ptr& wrapper) {
    if (!wrapper) {
      return;
    }
    // 读者纪律: 持 shared 锁直读 master grid (零拷贝), 单次拷贝进
    // OccupancyGrid 消息用于发布 (1Hz 可视化, 写者等锁即可)。
    std::shared_lock<std::shared_mutex> lock(wrapper->mutex());
    const costmap_2d::Costmap2D* cm = wrapper->get().getCostmap();
    if (!cm) {
      return;
    }
    ayanami::nav_msgs::OccupancyGrid grid;
    grid.header.frame_id = wrapper->getGlobalFrame();
    grid.info.resolution = cm->getResolution();
    grid.info.width = cm->getSizeInCellsX();
    grid.info.height = cm->getSizeInCellsY();
    Eigen::Isometry3d origin_iso = Eigen::Isometry3d::Identity();
    origin_iso.translation() =
        Eigen::Vector3d(cm->getOriginX(), cm->getOriginY(), 0.0);
    grid.info.origin = ayanami::geometry_msgs::Pose::fromIsometry(origin_iso);
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
    ayanami::interfaces::NavigationCallbacks::on_costmap_visualization(name,
                                                                       grid);
  };
  publish("global_costmap", navigation_server->globalCostmap());
  publish("local_costmap", navigation_server->localCostmap());
}

}  // namespace modules
}  // namespace ayanami
