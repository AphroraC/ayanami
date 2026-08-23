#pragma once

#include "ayanami/messages/geometry_msgs.hpp"
#include "ayanami/messages/nav_msgs.hpp"

#include <functional>
#include <memory>
#include <string>

namespace ayanami {
namespace abstract {

/**
 * @brief 宿主导航插件入口 (spec §5.1)。
 *
 * ROS 侧 dlopen 宿主插件 (如 CostmapNavigation, spec §5.6), 只通过本接口
 * initialize/start/stop/spinOnce/setGoal + setMapCallback 转发输入;
 * 内部持有 NavigationServerBase + BehaviorTreePilot 全套。
 */
class AbstractNavigation {
public:
  using Ptr = std::shared_ptr<AbstractNavigation>;

  virtual ~AbstractNavigation() = default;

  /// @brief 用 config_path 加载配置并初始化全部组件/插件。
  virtual void initialize(const std::string& config_path) = 0;

  /// @brief 启动内部循环 (BT 常驻树 / 订阅)。
  virtual void start() = 0;

  /// @brief 停止内部循环。
  virtual void stop() = 0;

  /// @brief 非阻塞轮询一次 (集成进外部线程/定时器时用)。
  virtual void spinOnce() = 0;

  /// @brief 设置导航目标。
  virtual void setGoal(const ayanami::geometry_msgs::PoseStamped& goal) = 0;

  /// @brief 地图可视化/转发回调 (frame_id, 占据栅格)。
  using MapCallback =
      std::function<void(const std::string&, const ayanami::nav_msgs::OccupancyGrid&)>;

  /// @brief 注册地图回调。默认空实现 (不注册), 派生可选覆写。
  virtual void setMapCallback(MapCallback cb) { (void)cb; }
};

} // namespace abstract
} // namespace ayanami
