// CostmapNavigation host module implementation.
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
// clearModules); the BT tree (bt_pilot member, declared after the server) is destroyed first, so BT nodes no longer
// hold stale module pointers.

#include "costmap_navigation.hpp"

#include "ayanami/costmap/components/costmap_map_family_factory.hpp"
#include <ayanami/utility/bootstrap.hpp>
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
  // bootstrap() owns both the config root and logging for the process.
  ayanami::bootstrap(config_path);

  // The map family factory is constructed by the host (the only creation point of the context); the core assembly chain only knows the abstract factory interface.
  navigation_server->configureCoreMaps(
      config_path, std::make_shared<costmap::CostmapMapFamilyFactory>());
  // loadModules already triggers the 4 virtual initialize*Module + createActions internally
  // (createActions is not explicitly called a second time, avoiding a rebuild of the action map).
  navigation_server->loadModules(config_path);

  bt_pilot.configure(config_path);

  Config nav_cfg(GlobalConfig::getConfigPath("config_navigation"));
  // The tree XML is a config asset (navigation.behavior_tree_xml), resolved
  // relative to the config directory. The base directory must be the one
  // GlobalConfig actually resolved to -- passing the package root instead of
  // the config/ directory makes GlobalConfig normalize to "<root>/config",
  // so joining the raw argument here would look for BT/navigation.xml one
  // level too high.
  const std::string xml_rel = nav_cfg.param<std::string>(
      "navigation", "behavior_tree_xml", "");
  const std::string tree_id = nav_cfg.param<std::string>(
      "navigation", "behavior_tree_id", k_default_tree_id);
  const std::string config_dir =
      GlobalConfig::instance()->param<std::string>("global", "config_path", ".");
  const std::string xml_path = xml_rel.empty()
                                   ? std::string{}
                                   : (std::filesystem::path(config_dir) /
                                      xml_rel).string();

  // Explicit wiring (bt_pilot configure only reads config; it does not auto-load/set).
  bt_pilot.loadFromFile(xml_path, tree_id);
  bt_pilot.setNavigationRuntime(navigation_server);

}

void CostmapNavigation::start() { navigation_server->start(); }

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

}  // namespace modules
}  // namespace ayanami
