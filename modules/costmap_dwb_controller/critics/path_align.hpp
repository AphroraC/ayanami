// path_align.hpp — 迁移自 dwb_critics/path_align.hpp。
#pragma once

#include <string>
#include <vector>

#include "path_dist.hpp"

namespace dwb_critics {
/**
 * @class PathAlignCritic
 * @brief Scores trajectories based on how far from the global path the front of
 * the robot ends up.
 */
class PathAlignCritic : public PathDistCritic {
public:
  PathAlignCritic() : zero_scale(false), forward_point_distance(0.0) {}
  void onInit() override;
  bool prepare(const geometry_msgs::Pose2D& pose,
               const nav_2d_msgs::Twist2D& vel,
               const geometry_msgs::Pose2D& goal,
               const nav_2d_msgs::Path2D& global_plan) override;
  double getScale() const override;
  double scorePose(const geometry_msgs::Pose2D& pose) override;

protected:
  bool zero_scale;
  double forward_point_distance;
};

}  // namespace dwb_critics
