// trajectory_utils.cpp — 迁移自 dwb_core/src/trajectory_utils.cpp。
#include "trajectory_utils.hpp"

#include <cmath>

#include "exceptions.hpp"

namespace dwb_core {

const geometry_msgs::Pose2D& getClosestPose(
    const dwb_msgs::Trajectory2D& trajectory, const double time_offset) {
  const double goal_time = time_offset;
  const unsigned int num_poses = trajectory.poses.size();
  if (num_poses == 0) {
    throw PlannerException("Cannot call getClosestPose on empty trajectory.");
  }
  unsigned int closest_index = num_poses;
  double closest_diff = 0.0;
  for (unsigned int i = 0; i < num_poses; ++i) {
    double diff = std::fabs((trajectory.time_offsets[i]) - goal_time);
    if (closest_index == num_poses || diff < closest_diff) {
      closest_index = i;
      closest_diff = diff;
    }
    if (goal_time < trajectory.time_offsets[i]) {
      break;
    }
  }
  return trajectory.poses[closest_index];
}

geometry_msgs::Pose2D projectPose(const dwb_msgs::Trajectory2D& trajectory,
                                  const double time_offset) {
  const double goal_time = time_offset;
  const unsigned int num_poses = trajectory.poses.size();
  if (num_poses == 0) {
    throw PlannerException("Cannot call projectPose on empty trajectory.");
  }
  if (goal_time <= (trajectory.time_offsets[0])) {
    return trajectory.poses[0];
  } else if (goal_time >= trajectory.time_offsets[num_poses - 1]) {
    return trajectory.poses[num_poses - 1];
  }

  for (unsigned int i = 0; i < num_poses - 1; ++i) {
    if (goal_time >= trajectory.time_offsets[i] &&
        goal_time < trajectory.time_offsets[i + 1]) {
      double time_diff = trajectory.time_offsets[i + 1] -
                         trajectory.time_offsets[i];
      double ratio = (goal_time - trajectory.time_offsets[i]) / time_diff;
      double inv_ratio = 1.0 - ratio;
      const geometry_msgs::Pose2D& pose_a = trajectory.poses[i];
      const geometry_msgs::Pose2D& pose_b = trajectory.poses[i + 1];
      geometry_msgs::Pose2D projected;
      projected.x = pose_a.x * inv_ratio + pose_b.x * ratio;
      projected.y = pose_a.y * inv_ratio + pose_b.y * ratio;
      projected.theta = pose_a.theta * inv_ratio + pose_b.theta * ratio;
      return projected;
    }
  }

  // Should not reach this point
  return trajectory.poses[num_poses - 1];
}

}  // namespace dwb_core
