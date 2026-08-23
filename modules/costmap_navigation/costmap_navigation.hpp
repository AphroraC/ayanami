#pragma once

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "ayanami/behavior_tree/pilot.hpp"
#include "ayanami/abstract/core/abstract_navigation.hpp"
#include "ayanami/costmap/components/costmap_navigation_server.hpp"
#include "ayanami/messages/geometry_msgs.hpp"

namespace ayanami {
namespace modules {

/**
 * @brief AbstractNavigation 宿主插件 (spec §5.6): 把 costmap 全套组装为 dlopen 入口。
 *
 * 职责 (spec §6.1 抢占纪律 + Ruling #5):
 * - 持 CostmapNavigationServer (costmap 家族 server) + BehaviorTreePilot 常驻树。
 * - initialize: configureCoreMaps → loadPlugins (触发虚 initialize*Plugin 注入
 *   costmap context) → createActions → pilot wiring: configure → loadFromFile →
 *   setNavigationServer (Ruling #5 显式三连, 不靠 pilot 自动)。
 * - setGoal: 单导航线程复用 (pilot_.run 常驻循环, beginNavigation 只重写黑板 +
 *   唤醒); 抢占先发零速度 (Global Constraint P9) + halt + controller cancelAll。
 * - publishMapsForVisualization: global/local CostmapWrapper → OccupancyGrid →
 *   map_callback_ + NavigationCallbacks::on_costmap_visualization 旁路。
 *
 * 生命周期 (Ruling #3): 析构先 stop() join 导航线程, 再 pilot stop, 再 server
 * stop (BT 节点不再持有过期插件指针时 server 才 clearPlugins)。
 */
class CostmapNavigation : public ayanami::abstract::AbstractNavigation {
 public:
  using Ptr = std::shared_ptr<CostmapNavigation>;

  CostmapNavigation();
  ~CostmapNavigation() override;

  void initialize(const std::string& config_path) override;
  void start() override;
  void stop() override;
  void spinOnce() override;
  void setGoal(const ayanami::geometry_msgs::PoseStamped& goal) override;
  void setMapCallback(MapCallback cb) override { map_callback_ = std::move(cb); }

  // ---- 测试/宿主支撑访问器 ----
  behavior_tree::BehaviorTreePilot& pilot() { return pilot_; }
  costmap::CostmapNavigationServer& navigationServer() {
    return navigation_server_;
  }

 private:
  /// @brief 转 global/local costmap → OccupancyGrid, 调 map_callback_ + 旁路槽。
  void publishMapsForVisualization();

  costmap::CostmapNavigationServer navigation_server_;
  behavior_tree::BehaviorTreePilot pilot_;
  MapCallback map_callback_;
  std::unique_ptr<std::thread> navigation_thread_;
  std::mutex goal_mutex_;
  ayanami::geometry_msgs::PoseStamped current_goal_;
  double visualization_rate_{1.0};  // costmap_visualization_rate (Hz)
  std::chrono::steady_clock::time_point last_publish_{};
  enum NavState { IDLE, RUNNING, COMPLETED, FAILED } state_{IDLE};
};

}  // namespace modules
}  // namespace ayanami
