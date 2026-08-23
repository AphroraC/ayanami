// oscillation.cpp — 迁移自 dwb_critics/src/oscillation.cpp (rclcpp::Clock 换
// std::chrono::steady_clock, 语义一致)。
#include "oscillation.hpp"

#include <chrono>
#include <cmath>
#include <string>
#include <vector>

#include "../exceptions.hpp"

namespace dwb_critics {

OscillationCritic::CommandTrend::CommandTrend() { reset(); }

void OscillationCritic::CommandTrend::reset() {
  sign_ = Sign::ZERO;
  positive_only_ = false;
  negative_only_ = false;
}

bool OscillationCritic::CommandTrend::update(double velocity) {
  bool flag_set = false;
  if (velocity < 0.0) {
    if (sign_ == Sign::POSITIVE) {
      negative_only_ = true;
      flag_set = true;
    }
    sign_ = Sign::NEGATIVE;
  } else if (velocity > 0.0) {
    if (sign_ == Sign::NEGATIVE) {
      positive_only_ = true;
      flag_set = true;
    }
    sign_ = Sign::POSITIVE;
  }
  return flag_set;
}

bool OscillationCritic::CommandTrend::isOscillating(double velocity) {
  return (positive_only_ && velocity < 0.0) || (negative_only_ && velocity > 0.0);
}

bool OscillationCritic::CommandTrend::hasSignFlipped() {
  return positive_only_ || negative_only_;
}

void OscillationCritic::onInit() {
  oscillation_reset_dist_ =
      config().param<double>(dwb_plugin_name_,
                             name_ + ".oscillation_reset_dist", 0.05);
  oscillation_reset_dist_sq_ = oscillation_reset_dist_ * oscillation_reset_dist_;
  oscillation_reset_angle_ =
      config().param<double>(dwb_plugin_name_,
                             name_ + ".oscillation_reset_angle", 0.2);
  oscillation_reset_time_ =
      config().param<double>(dwb_plugin_name_,
                             name_ + ".oscillation_reset_time", -1.0);

  /**
   * Historical Parameter Loading
   * If x_only_threshold is set, use that.
   * (min_speed_xy / min_trans_vel 历史兼容分支在 json 配置模型下无对应, 省略)
   */
  x_only_threshold_ =
      config().param<double>(dwb_plugin_name_,
                             name_ + ".x_only_threshold", 0.05);

  reset();
}

bool OscillationCritic::prepare(const geometry_msgs::Pose2D& pose,
                                const nav_2d_msgs::Twist2D&,
                                const geometry_msgs::Pose2D&,
                                const nav_2d_msgs::Path2D&) {
  pose_ = pose;
  return true;
}

void OscillationCritic::debrief(const nav_2d_msgs::Twist2D& cmd_vel) {
  if (setOscillationFlags(cmd_vel)) {
    prev_stationary_pose_ = pose_;
    prev_reset_time_ = std::chrono::steady_clock::now();
  }

  // if we've got restrictions... check if we can reset any oscillation flags
  if (x_trend_.hasSignFlipped() || y_trend_.hasSignFlipped() ||
      theta_trend_.hasSignFlipped()) {
    // Reset flags if enough time or distance has passed
    if (resetAvailable()) {
      reset();
    }
  }
}

bool OscillationCritic::resetAvailable() {
  if (oscillation_reset_dist_ >= 0.0) {
    double x_diff = pose_.x - prev_stationary_pose_.x;
    double y_diff = pose_.y - prev_stationary_pose_.y;
    double sq_dist = x_diff * x_diff + y_diff * y_diff;
    if (sq_dist > oscillation_reset_dist_sq_) {
      return true;
    }
  }
  if (oscillation_reset_angle_ >= 0.0) {
    double th_diff = pose_.theta - prev_stationary_pose_.theta;
    if (fabs(th_diff) > oscillation_reset_angle_) {
      return true;
    }
  }
  if (oscillation_reset_time_ >= 0.0) {
    auto t_diff =
        std::chrono::duration_cast<std::chrono::duration<double>>(
            std::chrono::steady_clock::now() - prev_reset_time_);
    if (t_diff.count() > oscillation_reset_time_) {
      return true;
    }
  }
  return false;
}

void OscillationCritic::reset() {
  x_trend_.reset();
  y_trend_.reset();
  theta_trend_.reset();
}

bool OscillationCritic::setOscillationFlags(
    const nav_2d_msgs::Twist2D& cmd_vel) {
  bool flag_set = false;
  // set oscillation flags for moving forward and backward
  flag_set |= x_trend_.update(cmd_vel.x);

  // we'll only set flags for strafing and rotating when we're not moving
  // forward at all
  if (x_only_threshold_ < 0.0 || fabs(cmd_vel.x) <= x_only_threshold_) {
    flag_set |= y_trend_.update(cmd_vel.y);
    flag_set |= theta_trend_.update(cmd_vel.theta);
  }
  return flag_set;
}

double OscillationCritic::scoreTrajectory(
    const dwb_msgs::Trajectory2D& traj) {
  if (x_trend_.isOscillating(traj.velocity.x) ||
      y_trend_.isOscillating(traj.velocity.y) ||
      theta_trend_.isOscillating(traj.velocity.theta)) {
    throw dwb_core::IllegalTrajectoryException(name_,
                                               "Trajectory is oscillating.");
  }
  return 0.0;
}

}  // namespace dwb_critics
