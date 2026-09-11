#pragma once

#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "ayanami/behavior_tree/pilot.hpp"
#include "ayanami/abstract/core/abstract_navigation.hpp"
#include "ayanami/costmap/components/costmap_map_context.hpp"
#include "ayanami/costmap/components/costmap_navigation_server.hpp"
#include "ayanami/messages/geometry_msgs.hpp"
#include "ayanami/messages/sensor_msgs.hpp"

namespace ayanami {
namespace modules {

/**
 * @brief AbstractNavigation host module: assembles the full costmap stack behind a dlopen entry point.
 *
 * Responsibilities (preemption discipline and more):
 * - Holds a CostmapNavigationServer (costmap family server) + a BehaviorTreePilot with a resident tree.
 * - initialize: configureCoreMaps -> loadModules (triggering the virtual initialize*Module injections of the
 *   costmap context, which internally includes createActions) -> bt_pilot wiring: configure ->
 *   loadFromFile -> setNavigationRuntime (explicit three-step sequence, not relying on bt_pilot auto-wiring).
 * - setGoal: single navigation thread reuse (bt_pilot.run is a resident loop; beginNavigation only rewrites the blackboard +
 *   wakes it); preemption first publishes zero velocity (Global Constraint P9) + halt + controller cancelAll.
 * - publishMapForVisualization: CostmapWrapper -> OccupancyGrid ->
 *   side-channel NavigationCallbacks::on_costmap_visualization.
 *
 * Lifecycle: on destruction, first stop joins the navigation thread, then bt_pilot stop, then server
 * stop (the server calls clearModules only once BT nodes no longer hold stale module pointers).
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
  /// Incremental (nav2 Costmap2DPublisher semantics): the first emission and
  /// any geometry change (size/resolution/origin) emit a FULL grid; a pure
  /// content change emits only the changed-cell window on
  /// on_costmap_visualization_update; an unchanged map emits nothing.
  void publishMapForVisualization(const std::string& name,
                                  const costmap_2d::CostmapWrapper::Ptr&
                                      wrapper);
  /// @brief Global map: static + inflation content changes only on configure
  /// / clear commands -> published ONCE (latched downstream), not per tick.
  void publishGlobalMapForVisualization();

  /// @brief Lossless full-grid copy for in-framework consumers (the
  /// smoother's collision check): emitted under the same shared lock as
  /// the occupancy visualization, only when the content changed. The
  /// payload is a plain value copy -- subscribers never touch the live
  /// grid, so no lock ordering is introduced.
  void emitRawCostmap(const std::string& name, const std::string& frame,
                      const costmap_2d::Costmap2D& cm);
  /// @brief Local map: rolling window with per-cycle obstacle updates ->
  /// republished at costmap_visualization_rate.
  void publishLocalMapForVisualization();

  /// @brief Last-published state per map name (incremental diffing baseline).
  struct SavedCostmap {
    bool valid{false};
    uint32_t width{0};
    uint32_t height{0};
    double resolution{0.0};
    double origin_x{0.0};
    double origin_y{0.0};
    /// @brief Raw master-grid costs (not occupancy-converted) for diffing.
    std::vector<unsigned char> costs;
    /// @brief Monotonic per-map revision of the last raw emission (advances
    /// whenever the published content changes).
    uint64_t raw_revision{0};
  };
  std::map<std::string, SavedCostmap> saved_costmaps;
  /// @brief Guards saved_costmaps (publish happens on the start/spinOnce
  /// caller threads).
  std::mutex saved_costmaps_mutex;

  std::shared_ptr<costmap::CostmapNavigationServer> navigation_server;
  behavior_tree::BehaviorTreePilot bt_pilot;
  std::unique_ptr<std::thread> navigation_thread;
  std::mutex goal_mutex;
  ayanami::geometry_msgs::PoseStamped current_goal;
  /// @brief Per-map publish cadence (Hz). The LOCAL map follows the robot and
  /// must publish faster than the static global map: navigation.costmap_
  /// visualization_rate gates the global map; costmap_local.visualization_
  /// rate (fallback: the global value) gates the local one. The incremental
  /// publisher keeps an unchanged map silent, so the faster local cadence
  /// costs nothing while the robot stands still.
  double global_visualization_rate{1.0};
  double local_visualization_rate{1.0};
  std::chrono::steady_clock::time_point last_global_publish{};
  std::chrono::steady_clock::time_point last_local_publish{};
  enum NavState { IDLE, RUNNING, COMPLETED, FAILED } state{IDLE};
};

}  // namespace modules
}  // namespace ayanami
