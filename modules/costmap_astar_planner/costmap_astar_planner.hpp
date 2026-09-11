#pragma once

#include <atomic>
#include <memory>
#include <vector>
#include <string>

#include <ayanami/costmap/core/costmap_planner.hpp>
#include <ayanami/costmap/core/costmap_snapshot.hpp>

namespace ayanami {
namespace modules {

/**
 * @brief A* global path planning module (CostmapPlanner family).
 *
 * Runs standard 4/8-connected A* directly on the GLOBAL costmap master grid
 * (family read contract: planner = global map only; fresh local observations
 * never shape the global plan).
 * - Blocking: cell cost >= INSCRIBED_INFLATED_OBSTACLE (lethal from 253 on)
 * - Traversal cost: step * (neutral_cost + cost_factor * cell_cost)
 * - Goal inside an obstacle: nearestValidGoal searches for the nearest valid cell within tolerance (approximate path)
 *
 * Map access discipline: configure(context, map) injects the own
 * configuration + a const smart pointer to the map module; makePlan takes a
 * SHORT-LOCK SNAPSHOT of the global master grid
 * (map->snapshotGlobal(): the shared lock lives only for the memcpy) and
 * runs the search on that immutable copy, so the map writer is never
 * delayed by planning. The start pose arrives fully resolved (the
 * execution layer substitutes the current robot pose for an empty start;
 * modules never interpret sentinels).
 */
class CostmapAstarPlanner final : public ayanami::costmap::CostmapPlanner {
 public:
  CostmapAstarPlanner() = default;
  // Derived class adds the virtual destructor with override (base AbstractPlanner is already virtual)
  ~CostmapAstarPlanner() override = default;

  /// @brief Parameter-loading seam (ExtensionModule): called by the loader
  /// right after a successful load, and by configure() for callers that build
  /// the object directly.
  void configureParameters(const ayanami::ModuleContext& context) override;

  void configure(const ModuleContext& context,
                 ayanami::costmap::CostmapMapContext::ConstPtr map) override;

  ResultT<void> makePlan(const geometry_msgs::PoseStamped& start,
                          const geometry_msgs::PoseStamped& goal,
                          double tolerance,
                          nav_msgs::Path& plan) override;

  bool cancel() override;

 private:
  struct CostmapAstarPlannerParam {
    bool use_diagonal = true;
    double tolerance = 0.0;
    double cost_factor = 0.5;
    int lethal_cost = 254;
    double neutral_cost = 1.0;
    bool use_final_approach = false;
  };

  CostmapAstarPlannerParam param;

  ayanami::costmap::CostmapMapContext::ConstPtr map_context;
  /// @brief Search body (see makePlan): runs on the short-lock snapshot
  /// with no map lock held.
  ResultT<void> makePlanWithMap(
      const geometry_msgs::PoseStamped& start,
      const geometry_msgs::PoseStamped& goal, double tolerance,
      nav_msgs::Path& plan_out,
      const ayanami::costmap::CostmapSnapshot& snapshot);

  std::atomic<bool> canceled{false};
};

}  // namespace modules
}  // namespace ayanami
