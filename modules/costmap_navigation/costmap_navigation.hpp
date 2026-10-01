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
 * - Costmap visualization: the SERVER emits per-layer artifacts
 *   (on_costmap_global_{static,inflation} /
 *   on_costmap_local_{obstacle,inflation}) generation-gated from the writer
 *   thread;
 *   the host publishes no maps.
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
  std::shared_ptr<costmap::CostmapNavigationServer> navigation_server;
  behavior_tree::BehaviorTreePilot bt_pilot;
  std::unique_ptr<std::thread> navigation_thread;
  std::mutex goal_mutex;
  ayanami::geometry_msgs::PoseStamped current_goal;
  enum NavState { IDLE, RUNNING, COMPLETED, FAILED } state{IDLE};
};

}  // namespace modules
}  // namespace ayanami
