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
 * @brief AbstractNavigation 宿主插件 (spec §5.6): 把 costmap 全套组装为 dlopen 入口。
 *
 * 职责 (spec §6.1 抢占纪律 + Ruling #5):
 * - 持 CostmapNavigationServer (costmap 家族 server) + BehaviorTreePilot 常驻树。
 * - initialize: configureCoreMaps → loadPlugins (触发虚 initialize*Plugin 注入
 *   costmap context, 内部含 createActions) → bt_pilot wiring: configure →
 *   loadFromFile → setNavigationRuntime (Ruling #5 显式三连, 不靠 bt_pilot 自动)。
 * - setGoal: 单导航线程复用 (bt_pilot.run 常驻循环, beginNavigation 只重写黑板 +
 *   唤醒); 抢占先发零速度 (Global Constraint P9) + halt + controller cancelAll。
 * - publishMapsForVisualization: global/local CostmapWrapper → OccupancyGrid →
 *   旁路 NavigationCallbacks::on_costmap_visualization。
 *
 * 生命周期 (Ruling #3): 析构先 stop() join 导航线程, 再 bt_pilot stop, 再 server
 * stop (BT 节点不再持有过期插件指针时 server 才 clearPlugins)。
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

  // ---- 测试/宿主支撑访问器 ----
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
  /// @brief 转 global/local costmap → OccupancyGrid, 发旁路槽。
  void publishMapsForVisualization();

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
