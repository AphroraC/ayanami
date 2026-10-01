// modules/rosemap_escape_recovery -- ESDF-gradient escape recovery.
//
// Pure C++ (zero ROS): the navi_minco_bit escape concept (gradient uphill,
// plateau sweep fallback, exit-distance gate) rebuilt on the unified
// start/tick/finish recovery protocol. The module never publishes velocity
// itself -- tick() returns the suggested body-frame velocity and the
// RecoveryExecution is the sole publisher.

#include "rosemap_escape_recovery.hpp"

#include <ayanami/rosemap/core/rosemap_query.hpp>
#include <ayanami/utility/config.hpp>

#include <algorithm>
#include <cmath>


namespace ayanami {
namespace modules {

namespace {
constexpr double k_two_pi = 6.28318530717958647692;
}

void RosemapEscapeRecovery::configureParameters(
    const ayanami::ModuleContext& context) {
  const Config& file = context.config;
  const std::string& section = context.name;
  param.escape_speed =
      file.param<double>(section, "escape_speed", param.escape_speed);
  param.exit_distance =
      file.param<double>(section, "exit_distance", param.exit_distance);
  param.min_gradient =
      file.param<double>(section, "min_gradient", param.min_gradient);
  param.direction_count =
      file.param<int>(section, "direction_count", param.direction_count);
  param.sweep_radius =
      file.param<double>(section, "sweep_radius", param.sweep_radius);
  param.resample_interval = file.param<int>(section, "resample_interval",
                                            param.resample_interval);
  param.max_ticks = file.param<int>(section, "max_ticks", param.max_ticks);
}

void RosemapEscapeRecovery::configure(
    const ModuleContext& context,
    ayanami::rosemap::RosemapMapContext::ConstPtr map) {
  configureParameters(context);
  map_context = std::move(map);
}

void RosemapEscapeRecovery::setCyclePeriod(double seconds) {
  cycle_dt = std::max(seconds, 1e-6);
}

bool RosemapEscapeRecovery::pickDirection(
    const Eigen::Vector2d& position, Eigen::Vector2d& direction) const {
  if (!map_context) {
    return false;
  }
  const auto query = map_context->queryRosemap(position.x(), position.y());
  if (query.status == ayanami::rosemap::RosemapQueryStatus::kOk &&
      query.gradient.norm() >= param.min_gradient) {
    direction = query.gradient.normalized();
    return true;
  }
  // Plateau (or out-of-field): sweep direction_count rays and keep the
  // one whose sample point has the largest clearance (navi's 36-way
  // fallback).
  const int count = std::max(4, param.direction_count);
  double best_distance = -std::numeric_limits<double>::infinity();
  bool found = false;
  for (int i = 0; i < count; ++i) {
    const double angle = 2.0 * k_two_pi *
                         static_cast<double>(i) / static_cast<double>(count);
    const Eigen::Vector2d candidate(std::cos(angle), std::sin(angle));
    const Eigen::Vector2d sample = position + param.sweep_radius * candidate;
    const auto probe = map_context->queryRosemap(sample.x(), sample.y());
    if (probe.status != ayanami::rosemap::RosemapQueryStatus::kOk) {
      continue;
    }
    if (probe.distance > best_distance) {
      best_distance = probe.distance;
      direction = candidate;
      found = true;
    }
  }
  return found;
}

void RosemapEscapeRecovery::start() {
  canceled.store(false);
  ticks_remaining = param.max_ticks;
  ticks_to_resample = 0;
}

ResultT<void> RosemapEscapeRecovery::tick(RecoveryCycleData& cycle) {
  if (canceled.load()) {
    return ResultT<void>::failure(ErrorCode::CANCELED,
                                  "escape recovery canceled");
  }
  if (!map_context) {
    return ResultT<void>::failure(ErrorCode::MAP_UNAVAILABLE,
                                  "escape recovery has no field module");
  }
  if (ticks_remaining <= 0) {
    return ResultT<void>::failure(
        ErrorCode::RECOVERY_FAILED, "escape tick budget exhausted");
  }
  --ticks_remaining;

  geometry_msgs::PoseStamped pose;
  if (!map_context->robotPose(pose)) {
    return ResultT<void>::failure(ErrorCode::TRANSFORM_UNAVAILABLE,
                                  "escape recovery lost the robot pose");
  }
  const Eigen::Vector2d position = pose.pose.position.head<2>();

  // Exit gate: clearance above exit_distance counts as escaped.
  const auto here = map_context->queryRosemap(position.x(), position.y());
  if (here.status == ayanami::rosemap::RosemapQueryStatus::kOk &&
      here.distance >= param.exit_distance) {
    cycle.status = RecoveryStatus::SUCCESS;
    return ResultT<void>::success();
  }

  // Re-pick the direction periodically (tracks field evolution).
  if (ticks_to_resample <= 0) {
    Eigen::Vector2d direction = escape_direction;
    if (pickDirection(position, direction)) {
      escape_direction = direction;
    }
    ticks_to_resample = std::max(1, param.resample_interval);
  }
  --ticks_to_resample;

  cycle.status = RecoveryStatus::RUNNING;
  geometry_msgs::TwistStamped velocity;
  velocity.header.frame_id = map_context->robotFrame();
  // World-frame direction -> body frame (the Twist convention).
  const double robot_yaw =
      std::atan2(pose.pose.orientation.toRotationMatrix()(1, 0),
                 pose.pose.orientation.toRotationMatrix()(0, 0));
  const double cos_yaw = std::cos(robot_yaw);
  const double sin_yaw = std::sin(robot_yaw);
  const Eigen::Vector2d body_velocity(
      cos_yaw * escape_direction.x() + sin_yaw * escape_direction.y(),
      -sin_yaw * escape_direction.x() + cos_yaw * escape_direction.y());
  velocity.twist.linear.x() =
      param.escape_speed * body_velocity.x();
  velocity.twist.linear.y() =
      param.escape_speed * body_velocity.y();
  cycle.velocity = std::move(velocity);
  return ResultT<void>::success();
}

void RosemapEscapeRecovery::finish() {}

bool RosemapEscapeRecovery::cancel() {
  canceled.store(true);
  return true;
}

}  // namespace modules
}  // namespace ayanami
