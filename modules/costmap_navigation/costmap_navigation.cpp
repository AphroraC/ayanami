// CostmapNavigation host plugin implementation.
//
// Preemption discipline: bt_pilot.run is a resident blocking tick loop (exits via kill_switch),
// so navigation_thread is created once and reused. A new goal is applied by the BehaviorTreePilot tick
// thread updating the goal revision; the controller keeps following the old path while the planner switches the goal in place, and once the new path
// is ready the controller updates in place. The host setGoal only does a non-blocking latest-wins delivery.
//
// bt_pilot wiring is the explicit three-step configure -> loadFromFile ->
// setNavigationRuntime; the tree is built once (passes the 17-blackboard-item validation) and later goals only applyPendingGoal.
//
// Destruction chain: ~CostmapNavigation -> stop joins the navigation thread -> navigation_server
// member destruction (the CostmapNavigationServer base NavigationServerBase calls clearActions first, then
// clearPlugins); the BT tree (bt_pilot member, declared after the server) is destroyed first, so BT nodes no longer
// hold stale plugin pointers.

#include "costmap_navigation.hpp"

#include "ayanami/costmap/components/costmap_map_family_factory.hpp"
#include "ayanami/costmap/core/costmap_command.hpp"

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

  // The map family factory is constructed by the host (the only creation point of the context); the core assembly chain only knows the abstract factory interface.
  navigation_server->configureCoreMaps(
      config_path, std::make_shared<costmap::CostmapMapFamilyFactory>());
  // loadPlugins already triggers the 4 virtual initialize*Plugin + createActions internally
  // (createActions is not explicitly called a second time, avoiding a rebuild of the action map).
  navigation_server->loadPlugins(config_path);

  bt_pilot.configure(config_path);

  Config nav_cfg(GlobalConfig::getConfigPath("config_navigation"));
  // Production single source: the default navigation tree is registered inline by the BehaviorTreeExtension,
  // no longer read from a file (config/BT/navigation.xml was deleted). Empty xml -> empty path uses the extension
  // default tree; tests can still supply custom XML via behavior_tree_xml explicitly.
  const std::string xml_rel = nav_cfg.param<std::string>(
      "navigation", "behavior_tree_xml", "");
  const std::string tree_id = nav_cfg.param<std::string>(
      "navigation", "behavior_tree_id", k_default_tree_id);
  const std::string xml_path = xml_rel.empty()
                                   ? std::string{}
                                   : (std::filesystem::path(config_path) /
                                      xml_rel).string();

  // Explicit wiring (bt_pilot configure only reads config; it does not auto-load/set).
  bt_pilot.loadFromFile(xml_path, tree_id);
  bt_pilot.setNavigationRuntime(navigation_server);

  visualization_rate = nav_cfg.param<double>(
      "navigation", "costmap_visualization_rate", 1.0);
}

void CostmapNavigation::start() {
  navigation_server->start();
  last_publish = std::chrono::steady_clock::now();
  // Global map (static + inflation) is published ONCE: downstream topics
  // are latched and its content changes only on configure / clear commands.
  // The local rolling map is republished periodically from spinOnce().
  publishGlobalMapForVisualization();
  publishLocalMapForVisualization();
}

void CostmapNavigation::stop() {
  if (navigation_thread && navigation_thread->joinable()) {
    bt_pilot.halt();
    bt_pilot.stop();  // kill_switch + wake run -> run exits
    auto t = std::move(navigation_thread);
    navigation_thread.reset();
    t->join();
  }
  if (navigation_server) navigation_server->stop();
}

void CostmapNavigation::spinOnce() {
 // Decoupled rolling-window updates: periodic rolling independent of the sensor stream (the window still follows the
 // robot when scans stop), with the rate set by the costmap_local/global.update_frequency config.
  navigation_server->updateRollingWindows();

  const auto now = std::chrono::steady_clock::now();
  const double hz = visualization_rate > 0.0 ? visualization_rate : 1.0;
  if (now - last_publish >= std::chrono::duration<double>(1.0 / hz)) {
    last_publish = now;
    publishLocalMapForVisualization();
  }
}

void CostmapNavigation::setGoal(const ayanami::geometry_msgs::PoseStamped& goal) {
  std::lock_guard<std::mutex> lk(goal_mutex);
  current_goal = goal;
  state = RUNNING;
  // Non-blocking mailbox delivery (spec: the host setGoal is also mailbox-ized latest-wins):
  // inside bt_pilot, pending_goal keeps the newest; halting the old tree / controller cancelAll /
  // zero velocity / session advancement all happen in the bt_pilot tick thread's applyPendingGoal, never on the
  // calling thread (never haltTree across threads).
  bt_pilot.beginNavigation(goal);
  if (!navigation_thread || !navigation_thread->joinable()) {
    navigation_thread = std::make_unique<std::thread>([this] { bt_pilot.run(); });
  }
}

void CostmapNavigation::setPointcloud(
    const ayanami::sensor_msgs::PointCloud2& cloud) {
  // Obstacle observation input goes straight to the CostmapNavigationServer public map-drop pipeline (no longer via the global side-channel slot).
  navigation_server->handleObstaclePointCloud(cloud);
}

void CostmapNavigation::setLaserScan(
    const ayanami::sensor_msgs::LaserScan& scan) {
  navigation_server->handleObstacleLaserScan(scan);
}

void CostmapNavigation::onLocalizationJump() {
  // Localization correction jump: clear the whole dynamic layer of the global map (clear_radius<=0 = entire dynamic layer);
  // the static layer bounds are re-marked and then restored by updateMap. Executed via the single-writer command port.
  if (!navigation_server) {
    return;
  }
  costmap::ClearCostmapCommand command;
  // Both maps are stamped in the shared map frame, so a correction jump
  // misaligns the dynamic markers of both -- clear both dynamic layers.
  command.affected_maps = "both";
  command.clear_radius = 0.0;  // <=0: clear the entire dynamic layer
  command.enabled = true;
  if (navigation_server->executeClearCommand(command)) {
    spdlog::info(
        "onLocalizationJump: cleared dynamic costmap after "
        "localization correction jump");
  } else {
    spdlog::warn(
        "onLocalizationJump: dynamic costmap clear failed (TF or "
        "map unavailable)");
  }
}

void CostmapNavigation::publishMapForVisualization(
    const std::string& name, const costmap_2d::CostmapWrapper::Ptr& wrapper) {
  if (!wrapper) {
    return;
  }
  // Reader discipline: shared-lock direct master grid read (zero copy), one
  // copy into the core OccupancyGrid for the side-channel slot (downstream
  // ROS topics are latched; the map writer waits for the lock).
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
        grid.data[idx] =
            static_cast<int8_t>((static_cast<int>(cost) * 100) / 252);
      }
    }
  }
  ayanami::interfaces::NavigationCallbacks::on_costmap_visualization(name,
                                                                     grid);
}

void CostmapNavigation::publishGlobalMapForVisualization() {
  publishMapForVisualization("global_costmap",
                             navigation_server->globalCostmap());
}

void CostmapNavigation::publishLocalMapForVisualization() {
  publishMapForVisualization("local_costmap",
                             navigation_server->localCostmap());
}

}  // namespace modules
}  // namespace ayanami
