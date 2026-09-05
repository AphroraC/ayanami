#pragma once

#include "ayanami/messages/geometry_msgs.hpp"
#include "ayanami/types.hpp"
#include "ayanami/utility/extension_module.hpp"

#include <memory>

namespace ayanami {
namespace abstract {

/**
 * @brief 位姿容差 (对标 geometry_msgs/PoseWithCovariance 等价语义)。
 *
 * position: xy 平面位置容差 (m); angle: 偏航角容差 (rad)。
 */
struct PoseTolerance {
  double position = 0.10;
  double angle = 0.20;
};

/**
 * @brief 速度容差 (体坐标, 判停边界)。
 *
 * linear: 线速度容差 (m/s); angular: 角速度容差 (rad/s)。
 */
struct TwistTolerance {
  double linear = 0.10;
  double angular = 0.10;
};

/**
 * @brief 目标检查总容差组合 (getTolerances 返回值)。
 */
struct GoalCheckerTolerances {
  PoseTolerance pose;
  TwistTolerance velocity;
};

/**
 * @brief 目标检查插件抽象 (对标 MBF AbstractGoalChecker, spec §5.1)。
 *
 * 判断 current_pose 是否已到达 goal_pose: 位置/转角差在容差内且速度低于静止
 * 阈值。有状态: reset() 应在收到新 goal / 重规划时调用。
 */
class AbstractGoalChecker : public virtual ExtensionModule {
 public:
  using Ptr = std::shared_ptr<AbstractGoalChecker>;

  virtual ~AbstractGoalChecker() = default;

  /// @brief 注入插件逻辑名 + 自身配置 (config_{goal_checker} 指向的模块
  /// json)。参数统一从这里读取, 插件不读全局配置索引。
  virtual void configure(const ayanami::PluginContext& context) {
    (void)context;
  }

  /// @brief 重置内部状态 (新 goal / 重规划)。
  virtual void reset() = 0;

  /// @brief 判断是否到达目标位姿。
  virtual bool isGoalReached(
      const ayanami::geometry_msgs::PoseStamped& current_pose,
      const ayanami::geometry_msgs::PoseStamped& goal_pose,
      const ayanami::geometry_msgs::TwistStamped& current_velocity) = 0;

  /// @brief 当前生效的位姿/速度容差。
  virtual GoalCheckerTolerances getTolerances() const = 0;
};

} // namespace abstract
} // namespace ayanami