// obstacle_footprint.hpp -- ported from dwb_critics/obstacle_footprint.hpp.
#pragma once

#include <vector>

#include "../dwb_types.hpp"
#include "base_obstacle.hpp"

namespace dwb_critics {
typedef std::vector<geometry_msgs::Point> Footprint;

/**
 * @brief Transform the footprint spec to be centered at the given pose
 */
Footprint getOrientedFootprint(const geometry_msgs::Pose2D& pose,
                               const Footprint& footprint_spec);

/**
 * @class ObstacleFootprintCritic
 * @brief Uses costmap 2d to assign negative costs if robot footprint is in
 * obstacle on any point of the trajectory.
 */
class ObstacleFootprintCritic : public BaseObstacleCritic {
public:
  bool prepare(const geometry_msgs::Pose2D& pose,
               const nav_2d_msgs::Twist2D& vel,
               const geometry_msgs::Pose2D& goal,
               const nav_2d_msgs::Path2D& global_plan) override;
  double scorePose(const geometry_msgs::Pose2D& pose) override;
  virtual double scorePose(const geometry_msgs::Pose2D& pose,
                           const Footprint& oriented_footprint);
  double getScale() const override {
    return costmap->getResolution() * scale;
  }

protected:
  /**
   * @brief Rasterizes a line in the costmap grid and checks for collisions
   */
  double lineCost(int x0, int x1, int y0, int y1);

  /**
   * @brief Checks the cost of a point in the costmap
   */
  double pointCost(int x, int y);

  Footprint footprint_spec;
};
}  // namespace dwb_critics
