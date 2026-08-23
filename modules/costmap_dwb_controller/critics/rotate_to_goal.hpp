// rotate_to_goal.hpp — 迁移自 dwb_critics/rotate_to_goal.hpp。
#pragma once

#include <string>
#include <vector>

#include "../trajectory_critic.hpp"

namespace dwb_critics {

/**
 * @class RotateToGoalCritic
 * @brief Forces the commanded trajectories to only be rotations if within a
 * certain distance window
 */
class RotateToGoalCritic : public dwb_core::TrajectoryCritic {
public:
  void onInit() override;
  void reset() override;
  bool prepare(const geometry_msgs::Pose2D& pose,
               const nav_2d_msgs::Twist2D& vel,
               const geometry_msgs::Pose2D& goal,
               const nav_2d_msgs::Path2D& global_plan) override;
  double scoreTrajectory(const dwb_msgs::Trajectory2D& traj) override;
  /**
   * @brief Assuming that this is an actual rotation when near the goal, score
   * the trajectory.
   */
  virtual double scoreRotation(const dwb_msgs::Trajectory2D& traj);

private:
  bool in_window_;
  bool rotating_;
  double goal_yaw_;
  double xy_goal_tolerance_;
  double xy_goal_tolerance_sq_;  ///< Cached squared tolerance
  double current_xy_speed_sq_, stopped_xy_velocity_sq_;
  double slowing_factor_;
  double lookahead_time_;
};

}  // namespace dwb_critics
