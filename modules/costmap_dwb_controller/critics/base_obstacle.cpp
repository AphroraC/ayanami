// base_obstacle.cpp -- ported from dwb_critics/src/base_obstacle.cpp.
#include "base_obstacle.hpp"

#include <string>
#include <utility>
#include <vector>

#include <costmap_2d/cost_values.hpp>

#include "../exceptions.hpp"

namespace dwb_critics {

void BaseObstacleCritic::onInit() {
  sum_scores = config().param<bool>(dwb_module_name,
                                     name + ".sum_scores", false);
}

double BaseObstacleCritic::scoreTrajectory(
    const dwb_msgs::Trajectory2D& traj) {
  double score = 0.0;
  for (unsigned int i = 0; i < traj.poses.size(); ++i) {
    double pose_score = scorePose(traj.poses[i]);
    // Optimized/branchless version of if (sum_scores) score += pose_score,
    // else score = pose_score;
    score = static_cast<double>(sum_scores) * score + pose_score;
  }
  return score;
}

double BaseObstacleCritic::scorePose(const geometry_msgs::Pose2D& pose) {
  unsigned int cell_x, cell_y;
  if (!costmap->worldToMap(pose.x, pose.y, cell_x, cell_y)) {
    throw dwb_core::IllegalTrajectoryException(name,
                                               "Trajectory Goes Off Grid.");
  }
  unsigned char cost = costmap->getCost(cell_x, cell_y);
  if (!isValidCost(cost)) {
    throw dwb_core::IllegalTrajectoryException(name,
                                               "Trajectory Hits Obstacle.");
  }
  return cost;
}

bool BaseObstacleCritic::isValidCost(const unsigned char cost) {
  return cost != costmap_2d::LETHAL_OBSTACLE &&
         cost != costmap_2d::INSCRIBED_INFLATED_OBSTACLE &&
         cost != costmap_2d::NO_INFORMATION;
}

void BaseObstacleCritic::addCriticVisualization(
    std::vector<std::pair<std::string, std::vector<float>>>& cost_channels) {
  std::pair<std::string, std::vector<float>> grid_scores;
  grid_scores.first = name;

  unsigned int size_x = costmap->getSizeInCellsX();
  unsigned int size_y = costmap->getSizeInCellsY();
  grid_scores.second.resize(size_x * size_y);
  unsigned int i = 0;
  for (unsigned int cy = 0; cy < size_y; cy++) {
    for (unsigned int cx = 0; cx < size_x; cx++) {
      grid_scores.second[i] = costmap->getCost(cx, cy);
      i++;
    }
  }
  cost_channels.push_back(grid_scores);
}

}  // namespace dwb_critics
