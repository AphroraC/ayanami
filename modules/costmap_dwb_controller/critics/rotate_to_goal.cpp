// rotate_to_goal.cpp -- ported from dwb_critics/src/rotate_to_goal.cpp
// (angles::shortest_angular_distance becomes a math implementation).
#include "rotate_to_goal.hpp"

#include <filesystem>
#include <string>
#include <vector>

#include <spdlog/spdlog.h>

#include "../exceptions.hpp"
#include "../trajectory_utils.hpp"
#include "ayanami/utility/config.hpp"

namespace dwb_critics {

inline double hypotSq(double dx, double dy) { return dx * dx + dy * dy; }

void RotateToGoalCritic::onInit() {
  xy_goal_tolerance =
      config().param<double>(dwb_module_name, "xy_goal_tolerance", 0.25);
  xy_goal_tolerance_sq = xy_goal_tolerance * xy_goal_tolerance;

  double stopped_xy_velocity =
      config().param<double>(dwb_module_name, "trans_stopped_velocity", 0.25);
  stopped_xy_velocity_sq = stopped_xy_velocity * stopped_xy_velocity;
  slowing_factor =
      config().param<double>(dwb_module_name, name + ".slowing_factor", 5.0);
  lookahead_time =
      config().param<double>(dwb_module_name, name + ".lookahead_time", -1.0);
  reset();
}

void RotateToGoalCritic::setGoalXyTolerance(double tolerance) {
  // Consistency check between this critic's "start rotating in place" radius and
  // the goal checker's acceptance radius. The value is pushed by the controller
  // (it came from the checker through AbstractController::setGoalTolerances),
  // so this no longer has to open another module's config file -- which is what
  // the previous implementation did, and why it silently skipped whenever the
  // navigation config was not laid out the way it expected.
  if (tolerance <= 0.0) {
    return;
  }
  if (xy_goal_tolerance > tolerance + 1e-9) {
    spdlog::warn(
        "RotateToGoal xy_goal_tolerance ({:.3f}) > goal checker position "
        "tolerance ({:.3f}): the in-place rotation band overlaps the arrival "
        "band near the goal; prefer xy_goal_tolerance <= position_tolerance",
        xy_goal_tolerance, tolerance);
  }
}

void RotateToGoalCritic::reset() {
  in_window = false;
  rotating = false;
}

bool RotateToGoalCritic::prepare(const geometry_msgs::Pose2D& pose,
                                 const nav_2d_msgs::Twist2D& vel,
                                 const geometry_msgs::Pose2D& goal,
                                 const nav_2d_msgs::Path2D&) {
  double dxy_sq = hypotSq(pose.x - goal.x, pose.y - goal.y);
  in_window = in_window || dxy_sq <= xy_goal_tolerance_sq;
  current_xy_speed_sq = hypotSq(vel.x, vel.y);
  rotating = rotating ||
              (in_window && current_xy_speed_sq <= stopped_xy_velocity_sq);
  goal_yaw = goal.theta;
  return true;
}

double RotateToGoalCritic::scoreTrajectory(
    const dwb_msgs::Trajectory2D& traj) {
  // If we're not sufficiently close to the goal, we don't care what the twist
  // is
  if (!in_window) {
    return 0.0;
  } else if (!rotating) {
    double speed_sq = hypotSq(traj.velocity.x, traj.velocity.y);
    if (speed_sq >= current_xy_speed_sq) {
      throw dwb_core::IllegalTrajectoryException(name,
                                                 "Not slowing down near goal.");
    }
    return speed_sq * slowing_factor + scoreRotation(traj);
  }

  // If we're sufficiently close to the goal, any transforming velocity is
  // invalid
  if (fabs(traj.velocity.x) > 0 || fabs(traj.velocity.y) > 0) {
    throw dwb_core::IllegalTrajectoryException(name,
                                               "Nonrotation command near goal.");
  }

  return scoreRotation(traj);
}

double RotateToGoalCritic::scoreRotation(const dwb_msgs::Trajectory2D& traj) {
  if (traj.poses.empty()) {
    throw dwb_core::IllegalTrajectoryException(name, "Empty trajectory.");
  }

  double end_yaw;
  if (lookahead_time >= 0.0) {
    geometry_msgs::Pose2D eval_pose =
        dwb_core::projectPose(traj, lookahead_time);
    end_yaw = eval_pose.theta;
  } else {
    end_yaw = traj.poses.back().theta;
  }
  // angles::shortest_angular_distance(from, to): shortest angular difference normalized to [-pi, pi]
  return fabs(std::remainder(end_yaw - goal_yaw, 2.0 * M_PI));
}

}  // namespace dwb_critics
