// map_grid.hpp -- ported from dwb_critics/map_grid.hpp (nav2_costmap_2d::Costmap2D
// becomes costmap_2d::Costmap2D).
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
  void setCostmap(const costmap_2d::Costmap2D* costmap) override;
  void addCriticVisualization(
      std::vector<std::pair<std::string, std::vector<float>>>&
          cost_channels) override;
  double getScale() const override {
    // Snapshot discipline: when no grid is bound yet (setPlan runs before the first computeVelocityCommands binds it),
    // return 0.0 so the scoring loop treats this critic as contributing nothing (cs.scale==0 skips it) instead of crashing.
    return costmap ? costmap->getResolution() * 0.5 * scale : 0.0;
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
    return cell_values[costmap->getIndex(x, y)];
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
    MapGridQueue(const costmap_2d::Costmap2D& costmap, MapGridCritic& parent)
    : costmap_queue::CostmapQueue(costmap, true), parent(parent) {}
    virtual ~MapGridQueue() = default;
    bool validCellToQueue(const costmap_queue::CellData& cell) override;

protected:
    MapGridCritic& parent;
  };

  /**
   * @brief Clear the queue and set cell_values to the appropriate number of
   * unreachableCellScore
   */
  void reset() override;

  /**
   * @brief Go through the queue and set the cells to the Manhattan distance
   * from their parents
   */
  void propogateManhattanDistances();

  std::shared_ptr<MapGridQueue> queue;
  std::vector<double> cell_values;
  double obstacle_score, unreachable_score;  ///< Special cell_values
  bool stop_on_failure;
  ScoreAggregationType aggregation_type;
};
}  // namespace dwb_critics
