// goal_dist.hpp -- ported from dwb_critics/goal_dist.hpp.
#pragma once

#include <vector>

#include "map_grid.hpp"

namespace dwb_critics {
/**
 * @class GoalDistCritic
 * @brief Scores trajectories based on how far along the global path they end up.
 *
 * The last pose of the global plan which is within the local costmap bounds is
 * used as the goal point. The distance from the goal point to the trajectory
 * is calculated as the Manhattan distance in the costmap grid.
 */
class GoalDistCritic : public MapGridCritic {
public:
  bool prepare(const geometry_msgs::Pose2D& pose,
               const nav_2d_msgs::Twist2D& vel,
               const geometry_msgs::Pose2D& goal,
               const nav_2d_msgs::Path2D& global_plan) override;

  /**
   * @brief Get the last pose of the global plan that is within the local
   * costmap bounds
   */
  virtual bool getLastPoseOnCostmap(const nav_2d_msgs::Path2D& global_plan,
                                    unsigned int& x, unsigned int& y);
};
}  // namespace dwb_critics
