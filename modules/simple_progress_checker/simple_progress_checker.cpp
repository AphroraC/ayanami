// SimpleProgressChecker -- simple progress checker module.
//
// Pure C++ (zero ROS): progress is declared only if, within the time_period window, the robot moves required_movement distance or
// rotates movement_angle_required; otherwise no progress (stuck).

#include "simple_progress_checker.hpp"

#include <cmath>

namespace ayanami {
namespace modules {

namespace {
constexpr double k_pi = 3.14159265358979323846;
}


void SimpleProgressChecker::configureParameters(const ayanami::ModuleContext& context) {
  // Parameter loading, separated from configuration: the loader calls this
  // right after a successful load, and every value lands in the module's own
  // SimpleProgressCheckerParam member (whose field initializers are the defaults for keys the
  // config omits).
  // Single source of parameters: this module's json (the file
  // config_progress_checker points to). Param section convention: the
  // section key = the module json "name" field, delivered as
  // ModuleContext::name.
  const std::string& section = context.name;
  param.required_movement = context.config.param<double>(
      section, "required_movement", param.required_movement);
  param.movement_angle_required = context.config.param<double>(
      section, "movement_angle_required", param.movement_angle_required);
  param.time_period = context.config.param<double>(
      section, "time_period", param.time_period);
}

void SimpleProgressChecker::configure(const ayanami::ModuleContext& context) {
  // Direct-construction callers (tests, or a host that builds the object
  // itself) never go through the loader's configureParameters call, so bind
  // the parameters here as well. Re-reading the same keys from the same
  // config is idempotent.
  configureParameters(context);
}


void SimpleProgressChecker::reset() {
  initialized = false;
  reset_pending.store(true);
}

namespace {

// Eigen quaternion -> 2D yaw angle [-pi, pi] (normalized to the yaw principal value).
double yawOf(const ayanami::geometry_msgs::Pose& pose) {
  const Eigen::Quaterniond& q = pose.orientation;
  double yaw = std::atan2(
      2.0 * (q.w() * q.z() + q.x() * q.y()),
      1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
  return yaw;
}

double normalizeAngle(double angle) {
  while (angle > k_pi) angle -= 2.0 * k_pi;
  while (angle < -k_pi) angle += 2.0 * k_pi;
  return angle;
}

}  // namespace

bool SimpleProgressChecker::check(
    const ayanami::geometry_msgs::PoseStamped& current_pose) {
  if (!initialized || reset_pending.load()) {
    base_pose = current_pose;
    base_time = std::chrono::steady_clock::now();
    initialized = true;
    reset_pending.store(false);
    return true;
  }

  const Eigen::Vector3d delta =
      current_pose.pose.position - base_pose.pose.position;
  const double moved = delta.head<2>().norm();
  const double angle_moved =
      std::abs(normalizeAngle(yawOf(current_pose.pose) - yawOf(base_pose.pose)));

  const double elapsed = std::chrono::duration<double>(
                             std::chrono::steady_clock::now() - base_time)
                             .count();

  if (moved >= param.required_movement ||
      angle_moved >= param.movement_angle_required) {
    base_pose = current_pose;
    base_time = std::chrono::steady_clock::now();
    return true;
  }

  if (elapsed > param.time_period) {
    return false;  // no progress (stuck)
  }
  return true;
}

}  // namespace modules
}  // namespace ayanami
