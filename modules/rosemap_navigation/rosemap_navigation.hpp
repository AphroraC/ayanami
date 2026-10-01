#pragma once

#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

#include "ayanami/abstract/core/abstract_navigation.hpp"
#include "ayanami/behavior_tree/pilot.hpp"
#include "ayanami/messages/geometry_msgs.hpp"
#include "ayanami/messages/sensor_msgs.hpp"
#include "ayanami/rosemap/core/rosemap_map_context.hpp"
#include "ayanami/rosemap/components/rosemap_navigation_server.hpp"

namespace ayanami {
namespace modules {

/**
 * @brief AbstractNavigation host module: assembles the full rosemap stack
 * behind a dlopen entry point (the rosemap-family twin of
 * costmap_navigation).
 *
 * Responsibilities:
 * - Holds a RosemapNavigationServer (rosemap family server) + a
 *   BehaviorTreePilot (one tree instance per navigation attempt).
 * - initialize: configureCoreMaps -> loadModules (triggering the virtual
 *   initialize*Module injections) -> bt_pilot wiring: configure ->
 *   loadFromFile -> setNavigationRuntime (explicit three-step sequence).
 * - setGoal: single navigation thread reuse (bt_pilot.run is a resident
 *   loop; beginNavigation only rewrites the blackboard + wakes it).
 * - Field visualization: the SERVER emits each published snapshot on
 *   NavigationCallbacks::on_rosemap_field from the writer thread
 *   (sequence-gated unconditional emission); the host publishes nothing.
 *
 * Lifecycle: on destruction, first stop joins the navigation thread, then
 * bt_pilot stop, then server stop (the server calls clearModules only once
 * BT nodes no longer hold stale module pointers).
 */
class RosemapNavigation final : public ayanami::abstract::AbstractNavigation {
 public:
  using Ptr = std::shared_ptr<RosemapNavigation>;

  RosemapNavigation();
  ~RosemapNavigation() override;

  void initialize(const std::string& config_path) override;
  void start() override;
  void stop() override;
  void spinOnce() override;
  void setGoal(const ayanami::geometry_msgs::PoseStamped& goal) override;
  void setPointcloud(const ayanami::sensor_msgs::PointCloud2& cloud) override;
  void setLaserScan(const ayanami::sensor_msgs::LaserScan& scan) override;

  // ---- Test/host support accessors ----
  behavior_tree::BehaviorTreePilot& pilot() { return bt_pilot; }
  rosemap::RosemapNavigationServer& navigationServer() {
    return *navigation_server;
  }

  /// @brief Expose the rosemap family module as a const shared owner. The
  /// module's read APIs enforce the snapshot discipline for callers.
  rosemap::RosemapMapContext::ConstPtr getSharedConstPtr() const {
    return navigation_server ? navigation_server->fieldContextConst() : nullptr;
  }

 private:
  std::shared_ptr<rosemap::RosemapNavigationServer> navigation_server;
  behavior_tree::BehaviorTreePilot bt_pilot;
  std::unique_ptr<std::thread> navigation_thread;
  std::mutex goal_mutex;
  ayanami::geometry_msgs::PoseStamped current_goal;
};

}  // namespace modules
}  // namespace ayanami
