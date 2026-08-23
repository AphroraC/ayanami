// SimpleGoalChecker — 简单目标检查模块。
//
// 纯 C++ (零 ROS): 位置 xy 差、相对转角差按 PoseTolerance 判; 静止速度阈值
// 按 TwistTolerance 判。容差经 simple_goal_checker.json 配置。

#include "simple_goal_checker.hpp"

#include <ayanami/utility/config.hpp>

#include <cmath>

namespace ayanami {
namespace modules {

SimpleGoalChecker::SimpleGoalChecker() { loadConfig(); }

void SimpleGoalChecker::loadConfig() {
  try {
    const ayanami::ComponentConfig holder("config_goal_checker");
    const auto& file = holder.get();
    tolerances_.pose.position =
        file.param<double>("goal_checker", "position_tolerance",
                           tolerances_.pose.position);
    tolerances_.pose.angle =
        file.param<double>("goal_checker", "angle_tolerance",
                           tolerances_.pose.angle);
    tolerances_.velocity.linear =
        file.param<double>("goal_checker", "linear_tolerance",
                           tolerances_.velocity.linear);
    tolerances_.velocity.angular =
        file.param<double>("goal_checker", "angular_tolerance",
                           tolerances_.velocity.angular);
  } catch (...) {
    // 配置不可读时保持默认值。
  }
}

void SimpleGoalChecker::reset() { initialized_.store(true); }

bool SimpleGoalChecker::isGoalReached(
    const ayanami::geometry_msgs::PoseStamped& current_pose,
    const ayanami::geometry_msgs::PoseStamped& goal_pose,
    const ayanami::geometry_msgs::TwistStamped& current_velocity) {
  initialized_.store(true);

  const Eigen::Vector3d delta =
      current_pose.pose.translation() - goal_pose.pose.translation();
  const double dist = delta.head<2>().norm();
  if (dist > tolerances_.pose.position) {
    return false;
  }

  // 相对转角 (Eigen::AngleAxisd 取旋转轴转角, 对 2D 偏航足够稳健)。
  const Eigen::AngleAxisd relative(
      goal_pose.pose.rotation().transpose() * current_pose.pose.rotation());
  const double angle_diff = relative.angle();
  if (angle_diff > tolerances_.pose.angle) {
    return false;
  }

  if (current_velocity.twist.linear.norm() > tolerances_.velocity.linear) {
    return false;
  }
  if (std::abs(current_velocity.twist.angular.z()) >
      tolerances_.velocity.angular) {
    return false;
  }
  return true;
}

ayanami::abstract::GoalCheckerTolerances SimpleGoalChecker::getTolerances()
    const {
  return tolerances_;
}

}  // namespace modules
}  // namespace ayanami
