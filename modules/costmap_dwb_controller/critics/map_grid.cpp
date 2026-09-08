// map_grid.cpp -- ported from dwb_critics/src/map_grid.cpp (RCLCPP becomes spdlog,
// node parameters become Config).
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
  // Snapshot architecture: at initialize time there may be no grid yet (it is bound once per cycle). The queue is
  // created when setCostmap first binds a real grid (see setCostmap).
  if (costmap) {
    queue = std::make_shared<MapGridQueue>(*costmap, *this);
  }

  // Always set to true, but can be overriden by subclasses
  stop_on_failure = true;

  std::string aggro_str =
      config().param<std::string>(dwb_plugin_name, name + ".aggregation_type",
                                  std::string("last"));
  std::transform(aggro_str.begin(), aggro_str.end(), aggro_str.begin(),
                 ::tolower);
  if (aggro_str == "last") {
    aggregation_type = ScoreAggregationType::Last;
  } else if (aggro_str == "sum") {
    aggregation_type = ScoreAggregationType::Sum;
  } else if (aggro_str == "product") {
    aggregation_type = ScoreAggregationType::Product;
  } else {
    spdlog::error("aggregation_type parameter \"{}\" invalid. Using Last.",
                  aggro_str);
    aggregation_type = ScoreAggregationType::Last;
  }
}

// Snapshot discipline: called by the controller every cycle after refreshing the local snapshot. The new epoch is a
// brand-new immutable object; MapGridQueue holds a reference to the grid -- the queue is rebuilt when the pointer changes.
void MapGridCritic::setCostmap(const costmap_2d::Costmap2D* costmap) {
  if (this->costmap == costmap) {
    return;
  }
  this->costmap = costmap;
  queue = costmap ? std::make_shared<MapGridQueue>(*costmap, *this) : nullptr;
}

void MapGridCritic::setAsObstacle(unsigned int index) {
  cell_values[index] = obstacle_score;
}

void MapGridCritic::reset() {
  // Snapshot discipline: setPlan may run before the first setCostmap snapshot binding (execution path:
  // ControllerExecution calls setPlan first, then computeVelocityCommands). At that point
  // neither costmap nor queue is bound -- reset must safely skip, and cell_values are filled in
  // on the first cycle after binding (setCostmap inside computeVelocityCommands has already rebuilt the queue).
  if (!costmap) {
    cell_values.clear();
    obstacle_score = 0.0;
    unreachable_score = 1.0;
    return;
  }
  if (queue) {
    queue->reset();
  }
  cell_values.resize(costmap->getSizeInCellsX() * costmap->getSizeInCellsY());
  obstacle_score = static_cast<double>(cell_values.size());
  unreachable_score = obstacle_score + 1.0;
  std::fill(cell_values.begin(), cell_values.end(), unreachable_score);
}

void MapGridCritic::propogateManhattanDistances() {
  while (!queue->isEmpty()) {
    costmap_queue::CellData cell = queue->getNextCell();
    cell_values[cell.index] =
        CellData::absolute_difference(cell.src_x, cell.x) +
        CellData::absolute_difference(cell.src_y, cell.y);
  }
}

double MapGridCritic::scoreTrajectory(const dwb_msgs::Trajectory2D& traj) {
  double score = 0.0;
  unsigned int start_index = 0;
  if (aggregation_type == ScoreAggregationType::Product) {
    score = 1.0;
  } else if (aggregation_type == ScoreAggregationType::Last &&
             !stop_on_failure) {
    start_index = traj.poses.size() - 1;
  }
  double grid_dist;

  for (unsigned int i = start_index; i < traj.poses.size(); ++i) {
    grid_dist = scorePose(traj.poses[i]);
    if (stop_on_failure) {
      if (grid_dist == obstacle_score) {
        throw dwb_core::IllegalTrajectoryException(name,
                                                   "Trajectory Hits Obstacle.");
      } else if (grid_dist == unreachable_score) {
        throw dwb_core::IllegalTrajectoryException(
            name, "Trajectory Hits Unreachable Area.");
      }
    }

    switch (aggregation_type) {
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
  if (!costmap->worldToMap(pose.x, pose.y, cell_x, cell_y)) {
    throw dwb_core::IllegalTrajectoryException(name,
                                               "Trajectory Goes Off Grid.");
  }
  return getScore(cell_x, cell_y);
}

void MapGridCritic::addCriticVisualization(
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
