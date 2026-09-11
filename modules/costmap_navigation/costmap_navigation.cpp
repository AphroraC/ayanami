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

  global_visualization_rate = nav_cfg.param<double>(
      "navigation", "costmap_visualization_rate", 1.0);
  // The local map follows the robot: it gets its own (faster) publish
  // cadence; without the key it inherits the global value.
  local_visualization_rate = nav_cfg.param<double>(
      "costmap_local", "visualization_rate", global_visualization_rate);
}

void CostmapNavigation::start() {
  navigation_server->start();
  last_global_publish = std::chrono::steady_clock::now();
  last_local_publish = std::chrono::steady_clock::now();
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
  const double global_hz =
      global_visualization_rate > 0.0 ? global_visualization_rate : 1.0;
  if (now - last_global_publish >=
      std::chrono::duration<double>(1.0 / global_hz)) {
    last_global_publish = now;
    publishGlobalMapForVisualization();
  }
  const double local_hz =
      local_visualization_rate > 0.0 ? local_visualization_rate : 1.0;
  if (now - last_local_publish >=
      std::chrono::duration<double>(1.0 / local_hz)) {
    last_local_publish = now;
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

void CostmapNavigation::publishMapForVisualization(
    const std::string& name, const costmap_2d::CostmapWrapper::Ptr& wrapper) {
  if (!wrapper) {
    return;
  }
  // Reader discipline: shared-lock direct master grid read (zero copy), one
  // pass over the grid either emitting a full grid or diffing against the
  // last published state (incremental publishing, nav2 Costmap2DPublisher
  // semantics: full on first emission / geometry change, bounded-window
  // update on content change, nothing when unchanged).
  std::shared_lock<std::shared_mutex> lock(wrapper->mutex());
  const costmap_2d::Costmap2D* cm = wrapper->get().getCostmap();
  if (!cm) {
    return;
  }
  const uint32_t width = cm->getSizeInCellsX();
  const uint32_t height = cm->getSizeInCellsY();
  const double resolution = cm->getResolution();
  const double origin_x = cm->getOriginX();
  const double origin_y = cm->getOriginY();

  std::lock_guard<std::mutex> guard(saved_costmaps_mutex);
  SavedCostmap& saved = saved_costmaps[name];
  const bool geometry_changed = !saved.valid || saved.width != width ||
                                saved.height != height ||
                                saved.resolution != resolution ||
                                saved.origin_x != origin_x ||
                                saved.origin_y != origin_y;

  const auto to_occupancy = [](unsigned char cost) {
    if (cost == costmap_2d::NO_INFORMATION) {
      return static_cast<int8_t>(-1);
    }
    if (costmap_2d::isLethal(cost)) {
      return static_cast<int8_t>(100);
    }
    return static_cast<int8_t>((static_cast<int>(cost) * 100) / 252);
  };

  if (geometry_changed) {
    ayanami::nav_msgs::OccupancyGrid grid;
    grid.header.frame_id = wrapper->globalFrame();
    grid.info.resolution = resolution;
    grid.info.width = width;
    grid.info.height = height;
    Eigen::Isometry3d origin_iso = Eigen::Isometry3d::Identity();
    origin_iso.translation() = Eigen::Vector3d(origin_x, origin_y, 0.0);
    grid.info.origin = ayanami::geometry_msgs::Pose::fromIsometry(origin_iso);
    grid.data.resize(static_cast<std::size_t>(width) * height);
    saved.costs.resize(static_cast<std::size_t>(width) * height);
    for (uint32_t y = 0; y < height; ++y) {
      for (uint32_t x = 0; x < width; ++x) {
        const unsigned char cost = cm->getCost(x, y);
        const std::size_t idx =
            static_cast<std::size_t>(y) * width + x;
        saved.costs[idx] = cost;
        grid.data[idx] = to_occupancy(cost);
      }
    }
    saved.valid = true;
    saved.width = width;
    saved.height = height;
    saved.resolution = resolution;
    saved.origin_x = origin_x;
    saved.origin_y = origin_y;
    ayanami::interfaces::NavigationCallbacks::on_costmap_visualization(name,
                                                                       grid);
    return;
  }

  // Content-only change: find the bounding box of cells whose cost differs
  // from the last published state; identical maps emit nothing at all.
  uint32_t min_x = width;
  uint32_t min_y = height;
  uint32_t max_x = 0;
  uint32_t max_y = 0;
  bool changed = false;
  for (uint32_t y = 0; y < height; ++y) {
    for (uint32_t x = 0; x < width; ++x) {
      const unsigned char cost = cm->getCost(x, y);
      if (cost ==
          saved.costs[static_cast<std::size_t>(y) * width + x]) {
        continue;
      }
      changed = true;
      min_x = std::min(min_x, x);
      min_y = std::min(min_y, y);
      max_x = std::max(max_x, x);
      max_y = std::max(max_y, y);
    }
  }
  if (!changed) {
    return;
  }
  ayanami::nav_msgs::OccupancyGridUpdate update;
  update.x0 = min_x;
  update.y0 = min_y;
  update.width = max_x - min_x + 1;
  update.height = max_y - min_y + 1;
  update.data.reserve(static_cast<std::size_t>(update.width) *
                      update.height);
  for (uint32_t y = min_y; y <= max_y; ++y) {
    for (uint32_t x = min_x; x <= max_x; ++x) {
      const std::size_t idx = static_cast<std::size_t>(y) * width + x;
      const unsigned char cost = cm->getCost(x, y);
      saved.costs[idx] = cost;
      update.data.push_back(to_occupancy(cost));
    }
  }
  ayanami::interfaces::NavigationCallbacks::on_costmap_visualization_update(
      name, update);
}

void CostmapNavigation::emitRawCostmap(const std::string& name,
                                       const std::string& frame,
                                       const costmap_2d::Costmap2D& cm) {
  ayanami::nav_msgs::RawCostmap raw;
  raw.name = name;
  raw.frame_id = frame;
  raw.resolution = cm.getResolution();
  raw.origin_x = cm.getOriginX();
  raw.origin_y = cm.getOriginY();
  raw.width = cm.getSizeInCellsX();
  raw.height = cm.getSizeInCellsY();
  raw.revision = saved_costmaps[name].raw_revision;
  const auto* map = cm.getCharMap();
  raw.costs.assign(map, map + static_cast<std::size_t>(raw.width) * raw.height);
  ayanami::interfaces::NavigationCallbacks::on_costmap_raw(raw);
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
