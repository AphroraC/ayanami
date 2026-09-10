// SimpleGoalChecker -- simple goal checker module.
//
// Pure C++ (zero ROS): position xy difference and relative rotation angle are judged by PoseTolerance; at-rest velocity thresholds
// are judged by TwistTolerance. Tolerances are configured via simple_goal_checker.json.

#include "simple_goal_checker.hpp"

#include <cmath>

namespace ayanami {
namespace modules {


void SimpleGoalChecker::configureParameters(const ayanami::ModuleContext& context) {
  // Parameter loading, separated from configuration: the loader calls this
  // right after a successful load, and every value lands in the module's own
  // SimpleGoalCheckerParam member (whose field initializers are the defaults for keys the
  // config omits).
  // Single source of parameters: this module's json (the file
  // config_goal_checker points to). Param section convention: the section
  // key = the module json "name" field, delivered as ModuleContext::name.
  const std::string& section = context.name;
  param.position_tolerance = context.config.param<double>(
      section, "position_tolerance", param.position_tolerance);
  param.angle_tolerance = context.config.param<double>(
      section, "angle_tolerance", param.angle_tolerance);
  param.linear_tolerance = context.config.param<double>(
      section, "linear_tolerance", param.linear_tolerance);
  param.angular_tolerance = context.config.param<double>(
      section, "angular_tolerance", param.angular_tolerance);
}

void SimpleGoalChecker::configure(const ayanami::ModuleContext& context) {
  // Direct-construction callers (tests, or a host that builds the object
  // itself) never go through the loader's configureParameters call, so bind
  // the parameters here as well. Re-reading the same keys from the same
  // config is idempotent.
  configureParameters(context);
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

  // Relative rotation angle (Eigen::AngleAxisd extracts the rotation axis angle; robust enough for 2D yaw).
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

ayanami::GoalCheckerTolerances SimpleGoalChecker::getTolerances()
    const {
  ayanami::GoalCheckerTolerances tolerances;
  tolerances.pose.position = param.position_tolerance;
  tolerances.pose.angle = param.angle_tolerance;
  tolerances.velocity.linear = param.linear_tolerance;
  tolerances.velocity.angular = param.angular_tolerance;
  return tolerances;
}

}  // namespace modules
}  // namespace ayanami
