#pragma once

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "ayanami/behavior_tree/pilot.hpp"
#include "ayanami/abstract/core/abstract_navigation.hpp"
#include "ayanami/costmap/components/costmap_map_context.hpp"
#include "ayanami/costmap/components/costmap_navigation_server.hpp"
#include "ayanami/messages/geometry_msgs.hpp"
#include "ayanami/messages/sensor_msgs.hpp"

namespace ayanami {
namespace modules {

/**
 * @brief AbstractNavigation host plugin: assembles the full costmap stack behind a dlopen entry point.
 *
 * Responsibilities (preemption discipline and more):
 * - Holds a CostmapNavigationServer (costmap family server) + a BehaviorTreePilot with a resident tree.
 * - initialize: configureCoreMaps -> loadPlugins (triggering the virtual initialize*Plugin injections of the
 *   costmap context, which internally includes createActions) -> bt_pilot wiring: configure ->
 *   loadFromFile -> setNavigationRuntime (explicit three-step sequence, not relying on bt_pilot auto-wiring).
 * - setGoal: single navigation thread reuse (bt_pilot.run is a resident loop; beginNavigation only rewrites the blackboard +
 *   wakes it); preemption first publishes zero velocity (Global Constraint P9) + halt + controller cancelAll.
 * - publishMapForVisualization: CostmapWrapper -> OccupancyGrid ->
 *   side-channel NavigationCallbacks::on_costmap_visualization.
 *
 * Lifecycle: on destruction, first stop joins the navigation thread, then bt_pilot stop, then server
 * stop (the server calls clearPlugins only once BT nodes no longer hold stale plugin pointers).
 */
class CostmapNavigation final : public ayanami::abstract::AbstractNavigation {
 public:
  using Ptr = std::shared_ptr<CostmapNavigation>;

  CostmapNavigation();
  ~CostmapNavigation() override;

  void initialize(const std::string& config_path) override;
  void start() override;
  void stop() override;
  void spinOnce() override;
  void setGoal(const ayanami::geometry_msgs::PoseStamped& goal) override;
  void setPointcloud(const ayanami::sensor_msgs::PointCloud2& cloud) override;
  void setLaserScan(const ayanami::sensor_msgs::LaserScan& scan) override;

  /// @brief Localization correction jump: clears the global costmap dynamic layers (marks made under the old
  /// correction are untrusted; the static layer is kept, and later scans rebuild it under the new correction).
  /// local is in the odom frame, unaffected by AMCL jumps, so it is not cleared.
  void onLocalizationJump() override;

  // ---- Test/host support accessors ----
  behavior_tree::BehaviorTreePilot& pilot() { return bt_pilot; }
   costmap::CostmapNavigationServer& navigationServer() {
    return *navigation_server;
   }

   /// @brief Expose the costmap family module as a const shared owner. The
   /// module's read APIs enforce the map locking contract for callers.
   costmap::CostmapMapContext::ConstPtr getSharedConstPtr() const {
     return navigation_server ? navigation_server->costmapContextConst()
                               : nullptr;
   }

 private:
  /// @brief Converts one costmap to OccupancyGrid and publishes to the
  /// side-channel slot (core message; rviz_viewer re-publishes as ROS).
  void publishMapForVisualization(const std::string& name,
                                  const costmap_2d::CostmapWrapper::Ptr&
                                      wrapper);
  /// @brief Global map: static + inflation content changes only on configure
  /// / clear commands -> published ONCE (latched downstream), not per tick.
  void publishGlobalMapForVisualization();
  /// @brief Local map: rolling window with per-cycle obstacle updates ->
  /// republished at costmap_visualization_rate.
  void publishLocalMapForVisualization();

  std::shared_ptr<costmap::CostmapNavigationServer> navigation_server;
  behavior_tree::BehaviorTreePilot bt_pilot;
  std::unique_ptr<std::thread> navigation_thread;
  std::mutex goal_mutex;
  ayanami::geometry_msgs::PoseStamped current_goal;
  double visualization_rate{1.0};  // costmap_visualization_rate (Hz)
  std::chrono::steady_clock::time_point last_publish{};
  enum NavState { IDLE, RUNNING, COMPLETED, FAILED } state{IDLE};
};

}  // namespace modules
}  // namespace ayanami
