#pragma once

#include <atomic>
#include <memory>
#include <string>

#include <ayanami/costmap/core/costmap_planner.hpp>

namespace ayanami {
namespace modules {

/**
 * @brief A* global path planning module (CostmapPlanner family).
 *
 * Runs standard 4/8-connected A* on the costmap global grid (shared-lock zero-copy read view).
 * - Blocking: cell cost >= INSCRIBED_INFLATED_OBSTACLE (lethal from 253 on)
 * - Traversal cost: step * (neutral_cost + cost_factor * cell_cost)
 * - Goal inside an obstacle: nearestValidGoal searches for the nearest valid cell within tolerance (approximate path)
 *
 * Map access discipline: configure(context, map) injects the own configuration + a const smart pointer to the map module;
 * makePlan internally takes a shared lock via map->lockGlobal() and reads the master grid directly with zero copy,
 * holding the lock for a single plan (<10 ms); the map writer waits for the lock to write.
 */
class CostmapSmacPlanner final : public ayanami::costmap::CostmapPlanner {
 public:
  CostmapSmacPlanner() = default;
  // Derived class adds the virtual destructor with override (base AbstractPlanner is already virtual)
  ~CostmapSmacPlanner() override = default;

  void configure(const PluginContext& context,
                 ayanami::costmap::CostmapMapContext::ConstPtr map) override;

  ResultT<void> makePlan(const geometry_msgs::PoseStamped& start,
                          const geometry_msgs::PoseStamped& goal,
                          double tolerance,
                          nav_msgs::Path& plan) override;

  bool cancel() override;
  bool setNewGoal(const geometry_msgs::PoseStamped& start,
                  const geometry_msgs::PoseStamped& goal,
                  double tolerance) override;

 private:
  struct CostmapSmacPlannerParam {
    bool use_diagonal = true;
    double tolerance = 0.0;
    double cost_factor = 0.5;
    int lethal_cost = 254;
    double neutral_cost = 1.0;
    bool use_final_approach = false;
  };

  CostmapSmacPlannerParam param;

  ayanami::costmap::CostmapMapContext::ConstPtr map_context;

  std::atomic<bool> canceled{false};
  std::atomic<uint64_t> goal_revision{0};
};

}  // namespace modules
}  // namespace ayanami
