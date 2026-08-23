// goal_align.hpp — 迁移自 dwb_critics/goal_align.hpp。
#pragma once

#include <string>
#include <vector>

#include "goal_dist.hpp"

namespace dwb_critics {

/**
 * @class GoalAlignCritic
 * @brief Scores trajectories based on whether the robot ends up pointing toward
 * the eventual goal
 */
class GoalAlignCritic : public GoalDistCritic {
public:
  GoalAlignCritic() : forward_point_distance_(0.0) {}
  void onInit() override;
  bool prepare(const geometry_msgs::Pose2D& pose,
               const nav_2d_msgs::Twist2D& vel,
               const geometry_msgs::Pose2D& goal,
               const nav_2d_msgs::Path2D& global_plan) override;
  double scorePose(const geometry_msgs::Pose2D& pose) override;

protected:
  double forward_point_distance_;
};

}  // namespace dwb_critics
