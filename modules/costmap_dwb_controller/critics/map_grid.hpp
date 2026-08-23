// map_grid.hpp — 迁移自 dwb_critics/map_grid.hpp (nav2_costmap_2d::Costmap2D
// 换 costmap_2d::Costmap2D)。
#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "../trajectory_critic.hpp"
#include "../costmap_queue/costmap_queue.hpp"

namespace dwb_critics {
/**
 * @class MapGridCritic
 * @brief breadth-first scoring of all the cells in the costmap
 */
class MapGridCritic : public dwb_core::TrajectoryCritic {
public:
  // Standard TrajectoryCritic Interface
  void onInit() override;
  double scoreTrajectory(const dwb_msgs::Trajectory2D& traj) override;
  void addCriticVisualization(
      std::vector<std::pair<std::string, std::vector<float>>>&
          cost_channels) override;
  double getScale() const override {
    return costmap_->getResolution() * 0.5 * scale_;
  }

  // Helper Functions
  /**
   * @brief Retrieve the score for a single pose
   */
  virtual double scorePose(const geometry_msgs::Pose2D& pose);

  /**
   * @brief Retrieve the score for a particular cell of the costmap
   */
  inline double getScore(unsigned int x, unsigned int y) {
    return cell_values_[costmap_->getIndex(x, y)];
  }

  /**
   * @brief Sets the score of a particular cell to the obstacle cost
   */
  void setAsObstacle(unsigned int index);

protected:
  /**
   * @brief Separate modes for aggregating scores across the multiple poses in
   * a trajectory.
   */
  enum class ScoreAggregationType { Last, Sum, Product };

  /**
   * @class MapGridQueue
   * @brief Subclass of CostmapQueue that avoids Obstacles and Unknown Values
   */
  class MapGridQueue : public costmap_queue::CostmapQueue {
public:
    MapGridQueue(costmap_2d::Costmap2D& costmap, MapGridCritic& parent)
    : costmap_queue::CostmapQueue(costmap, true), parent_(parent) {}
    virtual ~MapGridQueue() = default;
    bool validCellToQueue(const costmap_queue::CellData& cell) override;

protected:
    MapGridCritic& parent_;
  };

  /**
   * @brief Clear the queue and set cell_values_ to the appropriate number of
   * unreachableCellScore
   */
  void reset() override;

  /**
   * @brief Go through the queue and set the cells to the Manhattan distance
   * from their parents
   */
  void propogateManhattanDistances();

  std::shared_ptr<MapGridQueue> queue_;
  std::vector<double> cell_values_;
  double obstacle_score_, unreachable_score_;  ///< Special cell_values
  bool stop_on_failure_;
  ScoreAggregationType aggregationType_;
};
}  // namespace dwb_critics
