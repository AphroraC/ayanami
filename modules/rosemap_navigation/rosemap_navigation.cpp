// RosemapNavigation host module implementation.
//
// The rosemap-family twin of costmap_navigation: same preemption
// discipline (resident bt_pilot.run loop, latest-wins goal mailbox, no
// cross-thread haltTree), rosemap field stack behind it.

#include "rosemap_navigation.hpp"

#include <ayanami/utility/bootstrap.hpp>
#include <ayanami/utility/config.hpp>
#include "ayanami/interfaces/callbacks.hpp"
#include "ayanami/rosemap/components/rosemap_map_family_factory.hpp"
#include "ayanami/utility/logging.hpp"

#include <spdlog/spdlog.h>

#include <filesystem>
#include <utility>

namespace ayanami {
namespace modules {

namespace {
constexpr const char* k_default_tree_id =
    "NavigateWithReplanningAndSmoothing";
}  // namespace

RosemapNavigation::RosemapNavigation()
    : navigation_server(
          std::make_shared<rosemap::RosemapNavigationServer>()) {}

RosemapNavigation::~RosemapNavigation() { stop(); }

void RosemapNavigation::initialize(const std::string& config_path) {
  // bootstrap() owns both the config root and logging for the process.
  ayanami::bootstrap(config_path);

  // The map family factory is constructed by the host (the only creation
  // point of the context); the core assembly chain only knows the abstract
  // factory interface.
  navigation_server->configureCoreMaps(
      config_path, std::make_shared<rosemap::RosemapMapFamilyFactory>());
  // loadModules already triggers the 4 virtual initialize*Module +
  // createActions internally.
  navigation_server->loadModules(config_path);

  bt_pilot.configure(config_path);

  Config nav_cfg(GlobalConfig::getConfigPath("config_navigation"));
  // The tree XML is a config asset (navigation.behavior_tree_xml), resolved
  // relative to the config directory (see costmap_navigation: the base
  // directory must be the one GlobalConfig actually resolved to).
  const std::string xml_rel = nav_cfg.param<std::string>(
      "navigation", "behavior_tree_xml", "");
  const std::string tree_id = nav_cfg.param<std::string>(
      "navigation", "behavior_tree_id", k_default_tree_id);
  const std::string config_dir =
      GlobalConfig::instance()->param<std::string>("global", "config_path", ".");
  const std::string xml_path = xml_rel.empty()
                                   ? std::string{}
                                   : (std::filesystem::path(config_dir) /
                                      xml_rel)
                                         .string();

  // Explicit wiring (bt_pilot configure only reads config; it does not
  // auto-load/set).
  bt_pilot.loadFromFile(xml_path, tree_id);
  bt_pilot.setNavigationRuntime(navigation_server);
}

void RosemapNavigation::start() { navigation_server->start(); }

void RosemapNavigation::stop() {
  if (navigation_thread && navigation_thread->joinable()) {
    bt_pilot.halt();
    bt_pilot.stop();  // kill_switch + wake run -> run exits
    auto t = std::move(navigation_thread);
    navigation_thread.reset();
    t->join();
  }
  if (navigation_server) navigation_server->stop();
}

void RosemapNavigation::spinOnce() {
  // Nothing to poll: the server emits on_rosemap_field from its writer
  // thread on every published snapshot (sequence-gated unconditional).
}

void RosemapNavigation::setGoal(
    const ayanami::geometry_msgs::PoseStamped& goal) {
  std::lock_guard<std::mutex> lk(goal_mutex);
  current_goal = goal;
  // Non-blocking mailbox delivery (spec: the host setGoal is also
  // mailbox-ized latest-wins): inside bt_pilot, pending_goal keeps the
  // newest; halting the old tree / controller cancelAll / zero velocity /
  // session advancement all happen in the bt_pilot tick thread's
  // applyPendingGoal, never on the calling thread (never haltTree across
  // threads).
  bt_pilot.beginNavigation(goal);
  if (!navigation_thread || !navigation_thread->joinable()) {
    navigation_thread = std::make_unique<std::thread>([this] { bt_pilot.run(); });
  }
}

void RosemapNavigation::setPointcloud(
    const ayanami::sensor_msgs::PointCloud2& cloud) {
  // Obstacle observation input goes straight to the server's public
  // map-drop pipeline (no global side-channel slot).
  navigation_server->handleObstaclePointCloud(cloud);
}

void RosemapNavigation::setLaserScan(
    const ayanami::sensor_msgs::LaserScan& scan) {
  navigation_server->handleObstacleLaserScan(scan);
}

}  // namespace modules
}  // namespace ayanami
