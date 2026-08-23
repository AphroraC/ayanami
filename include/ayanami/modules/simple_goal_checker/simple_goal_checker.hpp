#pragma once

#include <atomic>
#include <memory>
#include <string>

#include <ayanami/abstract/core/abstract_goal_checker.hpp>

namespace ayanami {
namespace modules {

/**
 * @brief 简单目标检查模块 (对标 MBF SimpleGoalChecker)。
 *
 * 纯 C++ (零 ROS): xy 距离 < pose.position 且相对转角 < pose.angle,
 * 且线速度 < velocity.linear 且角速度 < velocity.angular → 到达。
 * 容差经 simple_goal_checker.json 的 goal_checker 段配置。
 */
class SimpleGoalChecker : public ayanami::abstract::AbstractGoalChecker {
 public:
  SimpleGoalChecker();

  void reset() override;

  bool isGoalReached(const ayanami::geometry_msgs::PoseStamped& current_pose,
                     const ayanami::geometry_msgs::PoseStamped& goal_pose,
                     const ayanami::geometry_msgs::TwistStamped& current_velocity) override;

  ayanami::abstract::GoalCheckerTolerances getTolerances() const override;

 private:
  void loadConfig();

  ayanami::abstract::GoalCheckerTolerances tolerances_;
  std::atomic<bool> initialized_{false};
};

}  // namespace modules
}  // namespace ayanami