#pragma once

#include <memory>
#include <string>
#include <utility>

#include "ayanami/abstract/components/planner_execution.hpp"
#include "ayanami/costmap/components/costmap_map_context.hpp"
#include "ayanami/costmap/core/costmap_planner.hpp"

namespace ayanami {
namespace costmap {

/**
 * @brief The costmap variant of the global planning execution.
 *
 * Binds a costmap context on top of PlannerExecution:
 * - preRun: checkActivate() idempotently activates the global costmap;
 * - postRun: checkDeactivate() idempotently deactivates the global costmap;
 * - safetyCheck: global wrapper isCurrent() (map freshness, atomic read);
 * - callPlanner: typed call into CostmapPlanner; the module reads the map
 *   under a lock internally via its held map module's readGlobal() callback (shared
 *   lock, zero copy, single planning call <10ms).
 */
class CostmapPlannerExecution : public ayanami::components::PlannerExecution {
 public:
  using Ptr = std::shared_ptr<CostmapPlannerExecution>;

  CostmapPlannerExecution(const std::string& name,
                          ayanami::abstract::AbstractPlanner::Ptr planner,
                          CostmapMapContext::Ptr ctx)
      : PlannerExecution(std::move(planner)),
        name(name),
        ctx(std::move(ctx)) {}

  ~CostmapPlannerExecution() override = default;

  void preRun() override {
    if (ctx && ctx->globalCostmap()) {
      ctx->globalCostmap()->checkActivate();
    }
  }

  void postRun() override {
    if (ctx && ctx->globalCostmap()) {
      ctx->globalCostmap()->checkDeactivate();
    }
  }

  bool safetyCheck() override {
    return !ctx || !ctx->globalCostmap() || ctx->globalCostmap()->isCurrent();
  }

 protected:
  ResultT<void> callPlanner(const ayanami::geometry_msgs::PoseStamped& start,
                             const ayanami::geometry_msgs::PoseStamped& goal,
                             double tolerance,
                             ayanami::nav_msgs::Path& plan) override {
    auto* typed = dynamic_cast<CostmapPlanner*>(planner.get());
    if (!typed) {
      return ResultT<void>::failure(
          ErrorCode::INTERNAL_ERROR,
          "CostmapPlannerExecution: module is not a CostmapPlanner");
    }
    if (!ctx) {
      return ResultT<void>::failure(
          ErrorCode::MAP_UNAVAILABLE,
          "CostmapPlannerExecution: no costmap module");
    }
    return typed->makePlan(start, goal, tolerance, plan);
  }

 private:
  std::string name;
  CostmapMapContext::Ptr ctx;
};

}  // namespace costmap
}  // namespace ayanami
