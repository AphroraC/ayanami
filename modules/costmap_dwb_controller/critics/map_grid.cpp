// map_grid.cpp — 迁移自 dwb_critics/src/map_grid.cpp (RCLCPP 换 spdlog,
// node 参数换 Config)。
#include "map_grid.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <spdlog/spdlog.h>

#include <costmap_2d/cost_values.hpp>

#include "../exceptions.hpp"

using std::abs;
using costmap_queue::CellData;

namespace dwb_critics {

// Customization of the CostmapQueue validCellToQueue method
bool MapGridCritic::MapGridQueue::validCellToQueue(
    const costmap_queue::CellData& /*cell*/) {
  return true;
}

void MapGridCritic::onInit() {
  queue_ = std::make_shared<MapGridQueue>(*costmap_, *this);

  // Always set to true, but can be overriden by subclasses
  stop_on_failure_ = true;

  std::string aggro_str =
      config().param<std::string>(dwb_plugin_name_, name_ + ".aggregation_type",
                                  std::string("last"));
  std::transform(aggro_str.begin(), aggro_str.end(), aggro_str.begin(),
                 ::tolower);
  if (aggro_str == "last") {
    aggregationType_ = ScoreAggregationType::Last;
  } else if (aggro_str == "sum") {
    aggregationType_ = ScoreAggregationType::Sum;
  } else if (aggro_str == "product") {
    aggregationType_ = ScoreAggregationType::Product;
  } else {
    spdlog::error("aggregation_type parameter \"{}\" invalid. Using Last.",
                  aggro_str);
    aggregationType_ = ScoreAggregationType::Last;
  }
}

void MapGridCritic::setAsObstacle(unsigned int index) {
  cell_values_[index] = obstacle_score_;
}

void MapGridCritic::reset() {
  queue_->reset();
  cell_values_.resize(costmap_->getSizeInCellsX() * costmap_->getSizeInCellsY());
  obstacle_score_ = static_cast<double>(cell_values_.size());
  unreachable_score_ = obstacle_score_ + 1.0;
  std::fill(cell_values_.begin(), cell_values_.end(), unreachable_score_);
}

void MapGridCritic::propogateManhattanDistances() {
  while (!queue_->isEmpty()) {
    costmap_queue::CellData cell = queue_->getNextCell();
    cell_values_[cell.index_] =
        CellData::absolute_difference(cell.src_x_, cell.x_) +
        CellData::absolute_difference(cell.src_y_, cell.y_);
  }
}

double MapGridCritic::scoreTrajectory(const dwb_msgs::Trajectory2D& traj) {
  double score = 0.0;
  unsigned int start_index = 0;
  if (aggregationType_ == ScoreAggregationType::Product) {
    score = 1.0;
  } else if (aggregationType_ == ScoreAggregationType::Last &&
             !stop_on_failure_) {
    start_index = traj.poses.size() - 1;
  }
  double grid_dist;

  for (unsigned int i = start_index; i < traj.poses.size(); ++i) {
    grid_dist = scorePose(traj.poses[i]);
    if (stop_on_failure_) {
      if (grid_dist == obstacle_score_) {
        throw dwb_core::IllegalTrajectoryException(name_,
                                                   "Trajectory Hits Obstacle.");
      } else if (grid_dist == unreachable_score_) {
        throw dwb_core::IllegalTrajectoryException(
            name_, "Trajectory Hits Unreachable Area.");
      }
    }

    switch (aggregationType_) {
      case ScoreAggregationType::Last:
        score = grid_dist;
        break;
      case ScoreAggregationType::Sum:
        score += grid_dist;
        break;
      case ScoreAggregationType::Product:
        if (score > 0) {
          score *= grid_dist;
        }
        break;
    }
  }

  return score;
}

double MapGridCritic::scorePose(const geometry_msgs::Pose2D& pose) {
  unsigned int cell_x, cell_y;
  // we won't allow trajectories that go off the map... shouldn't happen that
  // often anyways
  if (!costmap_->worldToMap(pose.x, pose.y, cell_x, cell_y)) {
    throw dwb_core::IllegalTrajectoryException(name_,
                                               "Trajectory Goes Off Grid.");
  }
  return getScore(cell_x, cell_y);
}

void MapGridCritic::addCriticVisualization(
    std::vector<std::pair<std::string, std::vector<float>>>& cost_channels) {
  std::pair<std::string, std::vector<float>> grid_scores;
  grid_scores.first = name_;

  unsigned int size_x = costmap_->getSizeInCellsX();
  unsigned int size_y = costmap_->getSizeInCellsY();
  grid_scores.second.resize(size_x * size_y);
  unsigned int i = 0;
  for (unsigned int cy = 0; cy < size_y; cy++) {
    for (unsigned int cx = 0; cx < size_x; cx++) {
      grid_scores.second[i] = costmap_->getCost(cx, cy);
      i++;
    }
  }
  cost_channels.push_back(grid_scores);
}

}  // namespace dwb_critics
