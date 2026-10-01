#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <vector>
#include <string>

#include <ayanami/costmap/core/costmap_planner.hpp>
#include <ayanami/costmap/core/costmap_snapshot.hpp>
#include <ayanami/costmap/core/fused_map_view.hpp>

namespace ayanami {
namespace modules {

/**
 * @brief A* global path planning module (CostmapPlanner family).
 *
 * Plans on the OVERLAY of the held global grid copy and a fresh local
 * snapshot (static-global policy: the global grid carries static+inflation
 * only, fresh dynamic obstacle marks live in the local rolling map, so a
 * plan must respect both).
 * - Blocking: fused cell cost >= INSCRIBED_INFLATED_OBSTACLE (lethal from 253 on)
 * - Traversal cost: step * (neutral_cost + cost_factor * cell_cost)
 * - Goal inside an obstacle: nearestValidGoal searches for the nearest valid cell within tolerance (approximate path)
 * - Start inside the inscribed band (253) is planned from as-is in ESCAPE mode: only LETHAL
 *   (254) cells block (the band thickness toward free space is itself >= 253, so the strict
 *   predicate would leave the search nowhere to expand and every replan would report
 *   PATH_NOT_FOUND). A LETHAL (254) start is still rejected (START_INVALID).
 *
 * Map access discipline: configure(context, map) injects the own
 * configuration + a const smart pointer to the map module. The GLOBAL grid
 * is held as a member copy refreshed lazily by generation (the writer bumps
 * the generation on every actual mutation, so for a static global the copy
 * is taken once and reused; clear commands advance it and trigger one
 * re-copy). Each planning call additionally takes ONE short-lock snapshot
 * of the local map (absent/failed local map or transform degrades to
 * global-only planning). All search work runs on these immutable inputs
 * with no map lock held, so the map writer is never delayed by planning.
 * The start pose arrives fully resolved (the execution layer substitutes
 * the current robot pose for an empty start; modules never interpret
 * sentinels).
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
  /// @brief Held copy of the (static) global master grid: refreshed only
  /// when the map's generation advances past global_copy_generation
  /// (configure-time materialization, clear commands).
  ayanami::costmap::CostmapSnapshot global_copy;
  std::uint64_t global_copy_generation{0};
  bool have_global_copy{false};
  /// @brief Reused local-snapshot buffer (one short-lock copy per planning
  /// call; the context reuses its costs capacity across calls).
  ayanami::costmap::CostmapSnapshot local_snapshot;
  /// @brief Search body (see makePlan): runs on the overlay view (held
  /// global copy + local snapshot) with no map lock held.
  ResultT<void> makePlanWithMap(
      const geometry_msgs::PoseStamped& start,
      const geometry_msgs::PoseStamped& goal, double tolerance,
      nav_msgs::Path& plan_out, const ayanami::costmap::FusedMapView& view);

  std::atomic<bool> canceled{false};
};

}  // namespace modules
}  // namespace ayanami
