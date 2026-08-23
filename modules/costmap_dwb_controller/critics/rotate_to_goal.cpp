// rotate_to_goal.cpp — 迁移自 dwb_critics/src/rotate_to_goal.cpp
// (angles::shortest_angular_distance 换数学实现)。
#include "rotate_to_goal.hpp"

#include <string>
#include <vector>

#include "../exceptions.hpp"
#include "../trajectory_utils.hpp"

namespace dwb_critics {

inline double hypot_sq(double dx, double dy) { return dx * dx + dy * dy; }

void RotateToGoalCritic::onInit() {
  xy_goal_tolerance_ =
      config().param<double>(dwb_plugin_name_, "xy_goal_tolerance", 0.25);
  xy_goal_tolerance_sq_ = xy_goal_tolerance_ * xy_goal_tolerance_;
  double stopped_xy_velocity =
      config().param<double>(dwb_plugin_name_, "trans_stopped_velocity", 0.25);
  stopped_xy_velocity_sq_ = stopped_xy_velocity * stopped_xy_velocity;
  slowing_factor_ =
      config().param<double>(dwb_plugin_name_, name_ + ".slowing_factor", 5.0);
  lookahead_time_ =
      config().param<double>(dwb_plugin_name_, name_ + ".lookahead_time", -1.0);
  reset();
}

void RotateToGoalCritic::reset() {
  in_window_ = false;
  rotating_ = false;
}

bool RotateToGoalCritic::prepare(const geometry_msgs::Pose2D& pose,
                                 const nav_2d_msgs::Twist2D& vel,
                                 const geometry_msgs::Pose2D& goal,
                                 const nav_2d_msgs::Path2D&) {
  double dxy_sq = hypot_sq(pose.x - goal.x, pose.y - goal.y);
  in_window_ = in_window_ || dxy_sq <= xy_goal_tolerance_sq_;
  current_xy_speed_sq_ = hypot_sq(vel.x, vel.y);
  rotating_ = rotating_ ||
              (in_window_ && current_xy_speed_sq_ <= stopped_xy_velocity_sq_);
  goal_yaw_ = goal.theta;
  return true;
}

double RotateToGoalCritic::scoreTrajectory(
    const dwb_msgs::Trajectory2D& traj) {
  // If we're not sufficiently close to the goal, we don't care what the twist
  // is
  if (!in_window_) {
    return 0.0;
  } else if (!rotating_) {
    double speed_sq = hypot_sq(traj.velocity.x, traj.velocity.y);
    if (speed_sq >= current_xy_speed_sq_) {
      throw dwb_core::IllegalTrajectoryException(name_,
                                                 "Not slowing down near goal.");
    }
    return speed_sq * slowing_factor_ + scoreRotation(traj);
  }

  // If we're sufficiently close to the goal, any transforming velocity is
  // invalid
  if (fabs(traj.velocity.x) > 0 || fabs(traj.velocity.y) > 0) {
    throw dwb_core::IllegalTrajectoryException(name_,
                                               "Nonrotation command near goal.");
  }

  return scoreRotation(traj);
}

double RotateToGoalCritic::scoreRotation(const dwb_msgs::Trajectory2D& traj) {
  if (traj.poses.empty()) {
    throw dwb_core::IllegalTrajectoryException(name_, "Empty trajectory.");
  }

  double end_yaw;
  if (lookahead_time_ >= 0.0) {
    geometry_msgs::Pose2D eval_pose =
        dwb_core::projectPose(traj, lookahead_time_);
    end_yaw = eval_pose.theta;
  } else {
    end_yaw = traj.poses.back().theta;
  }
  // angles::shortest_angular_distance(from, to): 归一化到 [-pi, pi] 的最短角差
  return fabs(std::remainder(end_yaw - goal_yaw_, 2.0 * M_PI));
}

}  // namespace dwb_critics
