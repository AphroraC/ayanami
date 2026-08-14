#pragma once

#include "ayanami/messages/geometry_msgs.hpp"

#include <memory>
#include <string>

namespace ayanami {
namespace abstract {

/**
 * @brief 地图形式无关的上下文基类 (spec §5.2, 关键)。
 *
 * NavigationServerBase 持 AbstractMapContext::Ptr, 可做通用 safetyCheck
 * (isCurrent()), 无需知道是 costmap 还是 gridmap。
 *
 * 派生示例: costmap::CostmapContext 包 global/local LayeredCostmap, 覆写
 * isCurrent/lock/unlock 为真实语义; gridmap 版沿用默认实现 (本轮只预留)。
 */
class AbstractMapContext {
public:
  using Ptr = std::shared_ptr<AbstractMapContext>;

  virtual ~AbstractMapContext() = default;

  /// @brief 全局坐标系 (map)。
  virtual const std::string& getGlobalFrame() const = 0;

  /// @brief 机器人坐标系 (base_link)。
  virtual const std::string& getRobotFrame() const = 0;

  /// @brief 地图是否新鲜。默认恒真, costmap 派生覆写。
  virtual bool isCurrent() const { return true; }

  /// @brief 写锁 (规划/控制加锁用)。默认空实现。
  virtual void lock() {}

  /// @brief 解锁。默认空实现。
  virtual void unlock() {}

  /**
   * @brief 获取机器人在 global_frame 的位姿。
   * @param[out] pose 输出的位姿。
   * @return true 成功; false 无可用的变换。
   * 由 TransformStore 提供, 这里只是转发。
   */
  virtual bool getRobotPose(ayanami::geometry_msgs::PoseStamped& pose) const = 0;
};

} // namespace abstract
} // namespace ayanami
