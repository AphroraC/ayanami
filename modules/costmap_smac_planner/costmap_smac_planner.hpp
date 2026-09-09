#pragma once

#include <atomic>
#include <memory>
#include <vector>
#include <string>

#include <ayanami/costmap/core/costmap_planner.hpp>

namespace ayanami {
namespace modules {

/**
 * @brief A* global path planning module (CostmapPlanner family).
 *
 * Runs standard 4/8-connected A* on the global+local overlay (family read
 * contract: planner = global+local superposition, per-cell conservative max;
 * NO_INFORMATION dominates).
 * - Blocking: cell cost >= INSCRIBED_INFLATED_OBSTACLE (lethal from 253 on)
 * - Traversal cost: step * (neutral_cost + cost_factor * cell_cost)
 * - Goal inside an obstacle: nearestValidGoal searches for the nearest valid cell within tolerance (approximate path)
 *
 * Map access discipline: configure(context, map) injects the own
 * configuration + a const smart pointer to the map module; makePlan calls
 * map->composePlanningOverlay() which holds the two map shared locks ONLY
 * for the O(N) compose pass, then releases them -- the A* itself queries the
 * returned PlanningOverlay buffer lock-free in O(1) per cell, so a slow
 * search (unreachable goal / large map) never stalls the map writer. The
 * start pose arrives fully resolved (the execution layer substitutes the
 * current robot pose for an empty start; plugins never interpret sentinels).
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
  /// @brief Reusable overlay scratch (global+local max per global cell):
  /// composed once per makePlan while the map locks are held, then queried
  /// lock-free by the search (see composePlanningOverlay).
  ayanami::costmap::PlanningOverlay overlay_buffer;

  std::atomic<bool> canceled{false};
  std::atomic<uint64_t> goal_revision{0};
};

}  // namespace modules
}  // namespace ayanami
