#pragma once

#include <functional>
#include <memory>
#include <string>
#include <utility>

#include "ayanami/abstract/components/smoother_action.hpp"
#include "ayanami/costmap/components/costmap_map_context.hpp"
#include "ayanami/costmap/components/costmap_smoother_execution.hpp"

namespace ayanami {
namespace costmap {

/**
 * @brief The costmap variant of the path smoothing Action (production
 * path).
 *
 * Reuses the entire SmootherAction protocol (TaskHandle direct + side-path
 * emit) and only overrides createExecution: returns a
 * CostmapSmootherExecution (with CostmapMapContext global
 * activation/deactivation + safetyCheck->isCurrent). The execution
 * configurator (server assembly: velocity limits etc.) is injected via
 * configure; a no-op by default.
 */
class CostmapSmootherAction : public ayanami::components::SmootherAction {
 public:
  using ExecutionConfigurator =
      std::function<void(ayanami::components::SmootherExecution::Ptr)>;

  CostmapSmootherAction(ayanami::abstract::AbstractSmoother::Ptr smoother,
                        CostmapMapContext::Ptr ctx,
                        ExecutionConfigurator configure = nullptr)
      : SmootherAction(std::move(smoother)),
        ctx(std::move(ctx)),
        configure(std::move(configure)) {}

  ~CostmapSmootherAction() override = default;

 protected:
  ayanami::components::SmootherExecution::Ptr createExecution() override {
    if (!ctx) {
      return SmootherAction::createExecution();
    }
    auto exec = std::make_shared<CostmapSmootherExecution>(
        "smoother", smoother, ctx);
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