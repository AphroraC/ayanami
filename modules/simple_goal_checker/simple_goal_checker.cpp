// SimpleGoalChecker — 简单目标检查模块。
//
// 纯 C++ (零 ROS): 位置 xy 差、相对转角差按 PoseTolerance 判; 静止速度阈值
// 按 TwistTolerance 判。容差经 simple_goal_checker.json 配置。

#include "simple_goal_checker.hpp"

#include <ayanami/utility/config.hpp>

#include <cmath>

namespace ayanami {
namespace modules {

SimpleGoalChecker::SimpleGoalChecker() {
  // 参数在构造时经 glim 式显式 Config 读取并拷贝进成员 (无 thread_local 注入,
  // 配置不可读时保持默认值): 主 config.json 的 global 单层索引
  // config_goal_checker → 本插件 json 文件。
  try {
    const Config file(GlobalConfig::getConfigPath("config_goal_checker"));
    param.position_tolerance =
        file.param<double>("goal_checker", "position_tolerance",
                           param.position_tolerance);
    param.angle_tolerance =
        file.param<double>("goal_checker", "angle_tolerance",
                           param.angle_tolerance);
    param.linear_tolerance =
        file.param<double>("goal_checker", "linear_tolerance",
                           param.linear_tolerance);
    param.angular_tolerance =
        file.param<double>("goal_checker", "angular_tolerance",
                           param.angular_tolerance);
  } catch (...) {
    // 配置不可读时保持默认值。
  }
}

void SimpleGoalChecker::reset() { initialized.store(true); }

bool SimpleGoalChecker::isGoalReached(
    const ayanami::geometry_msgs::PoseStamped& current_pose,
    const ayanami::geometry_msgs::PoseStamped& goal_pose,
    const ayanami::geometry_msgs::TwistStamped& current_velocity) {
  initialized.store(true);

  const Eigen::Vector3d delta =
      current_pose.pose.position - goal_pose.pose.position;
  const double dist = delta.head<2>().norm();
  if (dist > param.position_tolerance) {
    return false;
  }

  // 相对转角 (Eigen::AngleAxisd 取旋转轴转角, 对 2D 偏航足够稳健)。
  const Eigen::AngleAxisd relative(
      (goal_pose.pose.orientation.inverse() * current_pose.pose.orientation)
          .normalized());
  const double angle_diff = relative.angle();
  if (angle_diff > param.angle_tolerance) {
    return false;
  }

  if (current_velocity.twist.linear.norm() > param.linear_tolerance) {
    return false;
  }
  if (std::abs(current_velocity.twist.angular.z()) >
      param.angular_tolerance) {
    return false;
  }
  return true;
}

ayanami::abstract::GoalCheckerTolerances SimpleGoalChecker::getTolerances()
    const {
  ayanami::abstract::GoalCheckerTolerances tolerances;
  tolerances.pose.position = param.position_tolerance;
  tolerances.pose.angle = param.angle_tolerance;
  tolerances.velocity.linear = param.linear_tolerance;
  tolerances.velocity.angular = param.angular_tolerance;
  return tolerances;
}

}  // namespace modules
}  // namespace ayanami
