// base_obstacle.hpp — 迁移自 dwb_critics/base_obstacle.hpp。
#pragma once

#include <string>
#include <utility>
#include <vector>

#include "../trajectory_critic.hpp"

namespace dwb_critics {
/**
 * @class BaseObstacleCritic
 * @brief Uses costmap 2d to assign negative costs if a circular robot would
 * collide at any point of the trajectory.
 */
class BaseObstacleCritic : public dwb_core::TrajectoryCritic {
public:
  void onInit() override;
  double scoreTrajectory(const dwb_msgs::Trajectory2D& traj) override;
  void addCriticVisualization(
      std::vector<std::pair<std::string, std::vector<float>>>&
          cost_channels) override;

  /**
   * @brief Return the obstacle score for a particular pose
   */
  virtual double scorePose(const geometry_msgs::Pose2D& pose);

  /**
   * @brief Check to see whether a given cell cost is valid for driving through.
   */
  virtual bool isValidCost(const unsigned char cost);

protected:
  bool sum_scores;
};
}  // namespace dwb_critics
