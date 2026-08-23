#pragma once

#include "ayanami/messages/geometry_msgs.hpp"

#include <memory>

namespace ayanami {
namespace abstract {

/**
 * @brief 进展检查插件抽象 (对标 MBF AbstractProgressChecker, spec §5.1)。
 *
 * 跟踪机器人是否在推进: 设定时间窗内应移动足够的距离/转角, 否则判为卡住
 * (无进展)。有状态: reset() 应在新 goal 或恢复动作前重设基线。
 */
class AbstractProgressChecker {
 public:
  using Ptr = std::shared_ptr<AbstractProgressChecker>;

  virtual ~AbstractProgressChecker() = default;

  /// @brief 重置内部状态 (新 goal / 恢复动作前)。
  virtual void reset() = 0;

  /// @brief 检查当前位姿是否有进展; false = 无进展 (卡住)。
  virtual bool check(
      const ayanami::geometry_msgs::PoseStamped& current_pose) = 0;
};

} // namespace abstract
} // namespace ayanami