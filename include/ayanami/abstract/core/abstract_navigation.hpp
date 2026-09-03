#pragma once

#include "ayanami/messages/geometry_msgs.hpp"
#include "ayanami/messages/nav_msgs.hpp"
#include "ayanami/messages/sensor_msgs.hpp"
#include "ayanami/utility/extension_module.hpp"

#include <functional>
#include <memory>
#include <string>

namespace ayanami {
namespace abstract {

/**
 * @brief 宿主导航插件入口 (spec §5.1)。
 *
 * ROS 侧 dlopen 宿主插件 (如 CostmapNavigation, spec §5.6), 只通过本接口
 * initialize/start/stop/spinOnce/setGoal + setPointcloud/setLaserScan 转发
 * 输入; 内部持有 NavigationServerBase + BehaviorTreePilot 全套。
 */
class AbstractNavigation : public virtual ExtensionModule {
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

  /// @brief 障碍观测输入: 点云 (ROS 回调同步 TF + 转换后直递宿主)。
  /// 默认 no-op (测试 stub 可不管); 具体宿主覆写转发到障碍落图管线。
  virtual void setPointcloud(const ayanami::sensor_msgs::PointCloud2& cloud) {
    (void)cloud;
  }
  /// @brief 障碍观测输入: 激光扫描 (同上)。
  virtual void setLaserScan(const ayanami::sensor_msgs::LaserScan& scan) {
    (void)scan;
  }
};

} // namespace abstract
} // namespace ayanami
