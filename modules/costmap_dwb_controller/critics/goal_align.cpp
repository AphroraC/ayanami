// goal_align.cpp -- ported from dwb_critics/src/goal_align.cpp.
#include "goal_align.hpp"

#include <string>
#include <vector>

#include "alignment_util.hpp"

namespace dwb_critics {

void GoalAlignCritic::onInit() {
  GoalDistCritic::onInit();
  stop_on_failure = false;

  forward_point_distance =
      config().param<double>(dwb_module_name,
                             name + ".forward_point_distance", 0.325);
}

bool GoalAlignCritic::prepare(const geometry_msgs::Pose2D& pose,
                              const nav_2d_msgs::Twist2D& vel,
                              const geometry_msgs::Pose2D& goal,
                              const nav_2d_msgs::Path2D& global_plan) {
  // we want the robot nose to be drawn to its final position
  // (before robot turns towards goal orientation), not the end of the
  // path for the robot center. Choosing the final position after
  // turning towards goal orientation causes instability when the
  // robot needs to make a 180 degree turn at the end
  double angle_to_goal = atan2(goal.y - pose.y, goal.x - pose.x);

  nav_2d_msgs::Path2D target_poses = global_plan;
  target_poses.poses.back().x += forward_point_distance * cos(angle_to_goal);
  target_poses.poses.back().y += forward_point_distance * sin(angle_to_goal);

  return GoalDistCritic::prepare(pose, vel, goal, target_poses);
}

double GoalAlignCritic::scorePose(const geometry_msgs::Pose2D& pose) {
  return GoalDistCritic::scorePose(getForwardPose(pose, forward_point_distance));
}

}  // namespace dwb_critics
