#pragma once

#include <functional>
#include <memory>
#include <string>
#include <utility>

#include "ayanami/abstract/components/planner_action.hpp"
#include "ayanami/costmap/components/costmap_map_context.hpp"
#include "ayanami/costmap/components/costmap_planner_execution.hpp"

namespace ayanami {
namespace costmap {

/**
 * @brief The costmap variant of the global planning Action (production
 * path).
 *
 * Reuses the entire PlannerAction protocol (TaskHandle direct + side-path
 * emit + preemption) and only overrides the createExecution factory:
 * returns a CostmapPlannerExecution (with CostmapMapContext activation /
 * deactivation / safetyCheck / locked map reads).
 * The execution configurator (server assembly: no extra injection) is
 * injected via configure; a no-op by default.
 */
class CostmapPlannerAction : public ayanami::components::PlannerAction {
 public:
  using ExecutionConfigurator =
      std::function<void(ayanami::components::PlannerExecution::Ptr)>;

  CostmapPlannerAction(ayanami::abstract::AbstractPlanner::Ptr planner,
                       CostmapMapContext::Ptr ctx,
                       ExecutionConfigurator configure = nullptr)
      : PlannerAction(std::move(planner)),
        ctx(std::move(ctx)),
        configure(std::move(configure)) {}

  ~CostmapPlannerAction() override = default;

 protected:
  ayanami::components::PlannerExecution::Ptr createExecution() override {
    if (!ctx) {
      return PlannerAction::createExecution();
    }
    auto exec = std::make_shared<CostmapPlannerExecution>(
        "planner", planner, ctx);
    if (configure) {
      configure(exec);
    }
    return exec;
  }

 private:
  CostmapMapContext::Ptr ctx;
  ExecutionConfigurator configure;
};

}  // namespace costmap
}  // namespace ayanami
