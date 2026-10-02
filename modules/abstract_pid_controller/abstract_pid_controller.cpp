// modules/abstract_pid_controller -- 3-axis PID local controller module
// (abstract family: pure path tracker, zero map dependency by construction;
// ported from costmap_pid_controller with the family shell removed).
// Rewritten with the semantics of the _legacy pid_controller, wrapping up the five historical control fixes:
// - computeVelocityCommands forcibly zeroes cmd_vel at entry (dirty memory bug)
// - diff mode: yaw control aims straight at the target (no sign flip); omni mode picks the nearest body-frame direction angle
// - repeated setPlan with the same goal does not reset PID state (plan_signature comparison)
// - linear is rate-limited separately by max_acc_x/max_decel_x; angular is limited by max_acc_theta
// - distanceDecay decelerates near the goal; once the position enters the param.goal_tolerance XY tolerance,
// linear=0 and only the angular PID aligns the goal heading; arrival is declared by the execution layer goal_checker
// (computeVelocityCommands no longer takes a goal_checker; the controller itself does not judge arrival).

#include "abstract_pid_controller.hpp"

#include <ayanami/messages/actions.hpp>
#include <ayanami/messages/failure.hpp>
#include <ayanami/utility/config.hpp>

#include <spdlog/spdlog.h>
#include <ayanami/utility/logging.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <functional>
#include <limits>
#include <map>

namespace ayanami {
namespace modules {

namespace {

constexpr double k_pi = 3.14159265358979323846;

double normalizeAngle(double angle) {
  while (angle > k_pi) {
    angle -= 2.0 * k_pi;
  }
  while (angle < -k_pi) {
    angle += 2.0 * k_pi;
  }
  return angle;
}

double shortestAngularDistance(double from, double to) {
  return normalizeAngle(to - from);
}

double yawOf(const geometry_msgs::PoseStamped& pose) {
  const Eigen::Quaterniond& q = pose.pose.orientation;
  return std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                    1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}

double distance2d(const geometry_msgs::PoseStamped& a,
                  const geometry_msgs::PoseStamped& b) {
  const Eigen::Vector3d& pa = a.pose.position;
  const Eigen::Vector3d& pb = b.pose.position;
  return std::hypot(pa.x() - pb.x(), pa.y() - pb.y());
}

// Coarse plan signature: only the final goal (x, y) * 1000 + path size, no per-pose
// hashing. After a re-smooth, a plan with the same goal and same size gets the same signature -> setPlan does not reset
// PID state (prevents PID reset stutter triggered by re-smoothing); different goals give different signatures (reset).
uint64_t computePlanSignature(const nav_msgs::Path& plan) {
  if (plan.poses.empty()) {
    return 0;
  }
  auto& goal = plan.poses.back();
  int64_t gx =
      static_cast<int64_t>(std::round(goal.pose.position.x() * 1000.0));
  int64_t gy =
      static_cast<int64_t>(std::round(goal.pose.position.y() * 1000.0));
  return (std::hash<int64_t>()(gx) ^ std::hash<int64_t>()(gy)) ^
         (static_cast<uint64_t>(plan.poses.size()) << 4);
}

}  // namespace

void PidController::PidAxis::reset() {
  integral = 0.0;
  last_error = 0.0;
  has_last_error = false;
}

double PidController::PidAxis::compute(double error, double dt) {
  dt = std::max(dt, 1e-6);
  integral += error * dt;
  if (i_max > 0.0) {
    integral = std::clamp(integral, -i_max, i_max);
  }
  const double derivative = has_last_error ? (error - last_error) / dt : 0.0;
  last_error = error;
  has_last_error = true;
  return kp * error + ki * integral + kd * derivative;
}

PidController::PidController() = default;

void PidController::setGoalTolerances(
    const ayanami::PoseTolerance& tolerance) {
  if (!goal_tolerances) {
    // Log the effective bound once per run, at the moment the contract first
    // arrives: a silently tightening clamp is exactly the kind of thing that
    // makes "why did it stop 5 cm short?" unexplainable from the logs.
    if (tolerance.position < param.goal_tolerance) {
      SPDLOG_LOGGER_INFO(
        ayanami::controllerLogger(),
        "PidController: honoring the goal checker's tighter XY "
        "tolerance ({:.3f} m) over goal_tolerance ({:.3f} m)",
        tolerance.position,
        param.goal_tolerance);
    }
    if (tolerance.angle > 1.5707963267948966) {
      SPDLOG_LOGGER_WARN(
        ayanami::controllerLogger(),
        "PidController: goal checker angle tolerance is {:.3f} rad "
        "(> pi/2); the heading contract is effectively vacuous, so arrival "
        "will be decided on position alone",
        tolerance.angle);
    }
  }
  goal_tolerances = tolerance;
}

double PidController::goalToleranceXy() const {
  if (!goal_tolerances) {
    return param.goal_tolerance;
  }
  return std::min(param.goal_tolerance, goal_tolerances->position);
}


void PidController::configureParameters(const ayanami::ModuleContext& context) {
  // Parameter loading, separated from configuration: the loader calls this
  // right after a successful load, and every value lands in the module's own
  // PidControllerParam member (whose field initializers are the defaults for keys the
  // config omits).
  // Parameters are read from the ModuleContext-injected own configuration (the module does not know the global config layout);
  // missing keys keep the default values.
  {
    const Config& file = context.config;
    // Param section convention: the section key = the module json "name"
    // field, delivered as ModuleContext::name.
    const std::string& section = context.name;
    param.max_vel_x = file.param<double>(section, "max_vel_x",
                                         param.max_vel_x);
    param.max_vel_theta =
        file.param<double>(section, "max_vel_theta",
                           param.max_vel_theta);
    param.max_acc_x = file.param<double>(section, "max_acc_x",
                                         param.max_acc_x);
    param.max_decel_x =
        file.param<double>(section, "max_decel_x",
                           param.max_decel_x);
    param.max_acc_theta =
        file.param<double>(section, "max_acc_theta",
                           param.max_acc_theta);
    param.kp_v = file.param<double>(section, "kp_v",
                                    param.kp_v);
    param.ki_v = file.param<double>(section, "ki_v",
                                    param.ki_v);
    param.kd_v = file.param<double>(section, "kd_v",
                                    param.kd_v);
    param.kp_w = file.param<double>(section, "kp_w",
                                    param.kp_w);
    param.ki_w = file.param<double>(section, "ki_w",
                                    param.ki_w);
    param.kd_w = file.param<double>(section, "kd_w",
                                    param.kd_w);
    param.dist_decay = file.param<double>(section,
                                          "dist_decay", param.dist_decay);
    param.goal_tolerance =
        file.param<double>(section, "goal_tolerance",
                           param.goal_tolerance);
    param.slide_window = file.param<double>(section,
                                            "slide_window", param.slide_window);
    param.default_dt = file.param<double>(section,
                                          "default_dt", param.default_dt);
    param.heading_deadband =
        file.param<double>(section, "heading_deadband",
                           param.heading_deadband);
    param.rotate_in_place_angle =
        file.param<double>(section, "rotate_in_place_angle",
                           param.rotate_in_place_angle);
    param.mode = file.param<std::string>(section, "mode",
                                         param.mode);
  }

  // The PID gains are derived state: applying them here keeps the module fully
  // usable once its parameters are loaded, without needing a map binding.

  pid_x.kp = param.kp_v;
  pid_x.ki = param.ki_v;
  pid_x.kd = param.kd_v;
  pid_y.kp = param.kp_v;
  pid_y.ki = param.ki_v;
  pid_y.kd = param.kd_v;
  pid_w.kp = param.kp_w;
  pid_w.ki = param.ki_w;
  pid_w.kd = param.kd_w;

  // Seed the runtime dt from the fallback before any execution push.
  cycle_period = param.default_dt;
}



bool PidController::setPlan(const nav_msgs::Path& plan) {
  if (plan.poses.empty()) {
    return false;
  }
  const uint64_t signature = computePlanSignature(plan);
  this->plan = plan;
  goal = plan.poses.back();
  canceled.store(false);
  if (signature != plan_signature) {
    // New goal (or first frame): reset tracking state so the first setPlan frame starts from 0 (no stale velocity carried over).
    plan_signature = signature;
    resetTrackingState();
    last_track_idx = 1;
  }
  return true;
}

ResultT<void> PidController::computeVelocityCommand(
    const geometry_msgs::PoseStamped& pose,
    const geometry_msgs::TwistStamped& velocity,
    geometry_msgs::TwistStamped& cmd) {
  // Historical dirty memory bug: forcibly zero at entry, then fill.
  geometry_msgs::TwistStamped cmd_vel = geometry_msgs::TwistStamped{};
  cmd_vel.twist = geometry_msgs::Twist{};
  cmd_vel.header = pose.header;

  if (plan.poses.empty()) {
    return ResultT<void>::failure(ErrorCode::NO_VALID_COMMAND,
                                      "PID controller has no plan");
  }
  if (canceled.load()) {
    resetTrackingState();
    return ResultT<void>::failure(ErrorCode::CANCELED,
                                      "PID controller canceled");
  }

  last_pose = pose;
  // Real inter-call dt from the execution cadence (setCyclePeriod push);
  // param.default_dt only seeds the member before injection.
  const double dt = std::max(cycle_period, 1e-6);

  // Arrival is entirely the execution layer goal_checker's responsibility; the PID itself uses param's
  // goal_tolerance (its own param) as the XY bound for stopping linear velocity. The injected arrival
  // contract is an upper bound: stopping farther out than the checker's XY tolerance would leave the
  // robot aligning its heading outside the acceptance region, where isGoalReached never fires.
  const double xy_tol = goalToleranceXy();

  // ---- Position already inside the goal XY tolerance: stop linear velocity, only the angular PID aligns the goal heading ----
  // (the goal_checker in the execution layer finally declares arrival)
  if (distance2d(last_pose, goal) < xy_tol) {
    const double yaw_err =
        shortestAngularDistance(yawOf(pose), yawOf(goal));
    const double target_theta =
        std::abs(yaw_err) < param.heading_deadband ? 0.0 : yaw_err;
    double angular_w = pid_w.compute(target_theta, dt);
    angular_w = std::clamp(angular_w, -param.max_vel_theta, param.max_vel_theta);
    angular_w = limitAcceleration(last_angular_vel, angular_w,
                                  param.max_acc_theta, param.max_acc_theta, dt);
    last_angular_vel = angular_w;
    last_linear_vel = 0.0;
    cmd_vel.twist.linear.x() = 0.0;
    cmd_vel.twist.linear.y() = 0.0;
    cmd_vel.twist.angular.z() = angular_w;
    cmd = cmd_vel;
  auto result = ResultT<void>::success();
    result.message = "PID controller reached goal position, aligning heading";
    return result;
  }

  // ---- Select the local tracking target (nearest point + lookahead window) ----
  const double robot_yaw = yawOf(pose);
  size_t nearest = 0;
  double min_dist = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < plan.poses.size(); ++i) {
    const double d = distance2d(pose, plan.poses[i]);
    if (d < min_dist) {
      min_dist = d;
      nearest = i;
    }
  }
  // Advance forward to the first point >= slide_window (or the last point), avoiding stop-and-stall where the target
  // distance collapses to ~0 when the lookahead window shrinks at dense path points.
  size_t target_idx = nearest;
  while (target_idx + 1 < plan.poses.size() &&
         distance2d(pose, plan.poses[target_idx]) < param.slide_window) {
    ++target_idx;
  }
  last_track_idx = target_idx;

  const Eigen::Vector3d rel =
      plan.poses[target_idx].pose.position - pose.pose.position;
  const double c = std::cos(robot_yaw);
  const double s = std::sin(robot_yaw);
  const double local_x = c * rel.x() + s * rel.y();   // forward
  const double local_y = -s * rel.x() + c * rel.y();  // leftward

  double target_theta = std::atan2(local_y, local_x);
  if (param.mode == "omni") {
    // Omnidirectional base: pick the nearest body-frame motion direction angle (0 / +-90 deg / 180 deg).
    std::map<double, double> candidates;
    for (const double reference : {0.0, k_pi / 2.0, k_pi, -k_pi / 2.0}) {
      const double diff = shortestAngularDistance(reference, target_theta);
      candidates.emplace(std::abs(diff), diff);
    }
    target_theta = candidates.begin()->second;
  }
  // Differential base: yaw control aims straight at the target (no sign flip).
  if (std::abs(target_theta) < param.heading_deadband) {
    target_theta = 0.0;
  }

  // ---- Three-axis PID ----
  double linear_x = 0.0;
  double linear_y = 0.0;
  if (param.mode == "omni") {
    linear_x = pid_x.compute(local_x, dt);
    linear_y = pid_y.compute(local_y, dt);
  } else {
    const double target_dist = std::hypot(local_x, local_y);
    const double heading_scale =
        std::abs(target_theta) > param.rotate_in_place_angle
            ? 0.0
            : std::max(0.0, std::cos(target_theta));
    linear_x = pid_x.compute(target_dist, dt) * heading_scale;
    linear_y = 0.0;
  }
  double angular_w = pid_w.compute(target_theta, dt);

  // Linear velocity upper bound.
  double cur_v = std::hypot(linear_x, linear_y);
  if (cur_v > param.max_vel_x) {
    const double k = param.max_vel_x / cur_v;
    linear_x *= k;
    linear_y *= k;
    cur_v = param.max_vel_x;
  }

  // Angular velocity: max_vel_theta clamp + max_acc_theta rate limit.
  angular_w = std::clamp(angular_w, -param.max_vel_theta, param.max_vel_theta);
  angular_w = limitAcceleration(last_angular_vel, angular_w,
                                param.max_acc_theta, param.max_acc_theta, dt);
  last_angular_vel = angular_w;

  // Linear velocity: distanceDecay deceleration near the goal + separate max_acc_x/max_decel_x rate limits.
  double desired_vel = 0.0;
  if (cur_v > 1e-9) {
    const double goal_dist = distance2d(pose, goal);
    const double decayed = distanceDecay(goal_dist, cur_v);
    desired_vel = limitAcceleration(last_linear_vel, decayed,
                                    param.max_acc_x, param.max_decel_x, dt);
    const double coeff = desired_vel / cur_v;
    linear_x *= coeff;
    linear_y *= coeff;
    last_linear_vel = desired_vel;
  } else {
    last_linear_vel = 0.0;
  }

  cmd_vel.twist.linear.x() = linear_x;
  cmd_vel.twist.linear.y() = linear_y;
  cmd_vel.twist.angular.z() = angular_w;

  cmd = cmd_vel;
  auto result = ResultT<void>::success();
  result.message = "PID controller generated velocity command";
  return result;
}

bool PidController::cancel() {
  canceled.store(true);
  return true;
}

void PidController::setCyclePeriod(double seconds) {
  if (seconds > 0.0) {
    cycle_period = seconds;
  }
}

void PidController::resetTrackingState() {
  pid_x.reset();
  pid_y.reset();
  pid_w.reset();
  last_linear_vel = 0.0;
  last_angular_vel = 0.0;
}

double PidController::limitAcceleration(double last_vel, double cmd_vel,
                                               double max_acc, double max_decel,
                                               double dt) const {
  // Acceleration uses max_acc; deceleration uses max_decel, limited separately.
  const double limit =
      cmd_vel >= last_vel ? std::max(0.0, max_acc) : std::max(0.0, max_decel);
  const double max_change = limit * std::max(dt, 0.0);
  return std::max(last_vel - max_change,
                  std::min(cmd_vel, last_vel + max_change));
}

double PidController::distanceDecay(double distance,
                                           double target_velocity) const {
  if (distance < param.dist_decay) {
    return (distance / std::max(param.dist_decay, 1e-9)) * target_velocity;
  }
  return target_velocity;
}

}  // namespace modules
}  // namespace ayanami
