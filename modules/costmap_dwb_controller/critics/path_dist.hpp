// path_dist.hpp -- ported from dwb_critics/path_dist.hpp.
#pragma once

#include <vector>

#include "map_grid.hpp"

namespace dwb_critics {
/**
 * @class PathDistCritic
 * @brief Scores trajectories based on how far they end up from the global plan
 */
class PathDistCritic : public MapGridCritic {
public:
  bool prepare(const geometry_msgs::Pose2D& pose,
               const nav_2d_msgs::Twist2D& vel,
               const geometry_msgs::Pose2D& goal,
               const nav_2d_msgs::Path2D& global_plan) override;
};
}  // namespace dwb_critics
