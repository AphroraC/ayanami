// oscillation.cpp -- ported from dwb_critics/src/oscillation.cpp (rclcpp::Clock becomes
// std::chrono::steady_clock, identical semantics).
#include "oscillation.hpp"

#include <chrono>
#include <cmath>
#include <string>
#include <vector>

#include "../exceptions.hpp"

namespace dwb_critics {

OscillationCritic::CommandTrend::CommandTrend() { reset(); }

void OscillationCritic::CommandTrend::reset() {
  sign = Sign::ZERO;
  positive_only = false;
  negative_only = false;
}

bool OscillationCritic::CommandTrend::update(double velocity) {
  bool flag_set = false;
  if (velocity < 0.0) {
    if (sign == Sign::POSITIVE) {
      negative_only = true;
      flag_set = true;
    }
    sign = Sign::NEGATIVE;
  } else if (velocity > 0.0) {
    if (sign == Sign::NEGATIVE) {
      positive_only = true;
      flag_set = true;
    }
    sign = Sign::POSITIVE;
  }
  return flag_set;
}

bool OscillationCritic::CommandTrend::isOscillating(double velocity) {
  return (positive_only && velocity < 0.0) || (negative_only && velocity > 0.0);
}

bool OscillationCritic::CommandTrend::hasSignFlipped() {
  return positive_only || negative_only;
}

void OscillationCritic::onInit() {
  oscillation_reset_dist =
      config().param<double>(dwb_plugin_name,
                             name + ".oscillation_reset_dist", 0.05);
  oscillation_reset_dist_sq = oscillation_reset_dist * oscillation_reset_dist;
  oscillation_reset_angle =
      config().param<double>(dwb_plugin_name,
                             name + ".oscillation_reset_angle", 0.2);
  oscillation_reset_time =
      config().param<double>(dwb_plugin_name,
                             name + ".oscillation_reset_time", -1.0);

  /**
   * Historical Parameter Loading
   * If x_only_threshold is set, use that.
   * (the min_speed_xy / min_trans_vel legacy-compatibility branch has no counterpart in the json config model; omitted)
   */
  x_only_threshold =
      config().param<double>(dwb_plugin_name,
                             name + ".x_only_threshold", 0.05);

  reset();
}

bool OscillationCritic::prepare(const geometry_msgs::Pose2D& pose,
                                const nav_2d_msgs::Twist2D&,
                                const geometry_msgs::Pose2D&,
                                const nav_2d_msgs::Path2D&) {
  this->pose = pose;
  return true;
}

void OscillationCritic::debrief(const nav_2d_msgs::Twist2D& cmd_vel) {
  if (setOscillationFlags(cmd_vel)) {
    prev_stationary_pose = pose;
    prev_reset_time = std::chrono::steady_clock::now();
  }

  // if we've got restrictions... check if we can reset any oscillation flags
  if (x_trend.hasSignFlipped() || y_trend.hasSignFlipped() ||
      theta_trend.hasSignFlipped()) {
    // Reset flags if enough time or distance has passed
    if (resetAvailable()) {
      reset();
    }
  }
}

bool OscillationCritic::resetAvailable() {
  if (oscillation_reset_dist >= 0.0) {
    double x_diff = pose.x - prev_stationary_pose.x;
    double y_diff = pose.y - prev_stationary_pose.y;
    double sq_dist = x_diff * x_diff + y_diff * y_diff;
    if (sq_dist > oscillation_reset_dist_sq) {
      return true;
    }
  }
  if (oscillation_reset_angle >= 0.0) {
    double th_diff = pose.theta - prev_stationary_pose.theta;
    if (fabs(th_diff) > oscillation_reset_angle) {
      return true;
    }
  }
  if (oscillation_reset_time >= 0.0) {
    auto t_diff =
        std::chrono::duration_cast<std::chrono::duration<double>>(
            std::chrono::steady_clock::now() - prev_reset_time);
    if (t_diff.count() > oscillation_reset_time) {
      return true;
    }
  }
  return false;
}

void OscillationCritic::reset() {
  x_trend.reset();
  y_trend.reset();
  theta_trend.reset();
}

bool OscillationCritic::setOscillationFlags(
    const nav_2d_msgs::Twist2D& cmd_vel) {
  bool flag_set = false;
  // set oscillation flags for moving forward and backward
  flag_set |= x_trend.update(cmd_vel.x);

  // we'll only set flags for strafing and rotating when we're not moving
  // forward at all
  if (x_only_threshold < 0.0 || fabs(cmd_vel.x) <= x_only_threshold) {
    flag_set |= y_trend.update(cmd_vel.y);
    flag_set |= theta_trend.update(cmd_vel.theta);
  }
  return flag_set;
}

double OscillationCritic::scoreTrajectory(
    const dwb_msgs::Trajectory2D& traj) {
  if (x_trend.isOscillating(traj.velocity.x) ||
      y_trend.isOscillating(traj.velocity.y) ||
      theta_trend.isOscillating(traj.velocity.theta)) {
    throw dwb_core::IllegalTrajectoryException(name,
                                               "Trajectory is oscillating.");
  }
  return 0.0;
}

}  // namespace dwb_critics
