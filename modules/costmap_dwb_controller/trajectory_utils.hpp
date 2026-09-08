// trajectory_utils.hpp -- ported from dwb_core/trajectory_utils.hpp (rclcpp::Duration
// becomes double seconds).
#pragma once

#include "dwb_types.hpp"

namespace dwb_core {

/**
 * @brief Helper function to find a pose in the trajectory with a particular
 * time time_offset
 * @param trajectory The trajectory to search
 * @param time_offset The desired time_offset
 * @return reference to the pose that is closest to the particular time offset
 */
const geometry_msgs::Pose2D& getClosestPose(
    const dwb_msgs::Trajectory2D& trajectory, const double time_offset);

/**
 * @brief Helper function to create a pose with an exact time_offset by
 * linearly interpolating between existing poses
 */
geometry_msgs::Pose2D projectPose(const dwb_msgs::Trajectory2D& trajectory,
                                  const double time_offset);

}  // namespace dwb_core
