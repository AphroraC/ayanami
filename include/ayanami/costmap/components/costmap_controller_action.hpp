#pragma once

#include <functional>
#include <memory>
#include <string>
#include <utility>

#include "ayanami/abstract/components/controller_action.hpp"
#include "ayanami/costmap/components/costmap_map_context.hpp"
#include "ayanami/costmap/components/costmap_controller_execution.hpp"

namespace ayanami {
namespace costmap {

/**
 * @brief The costmap variant of the local control Action (production
 * path).
 *
 * Reuses the entire ControllerAction protocol and only overrides the
 * createExecution factory: it returns a CostmapControllerExecution (with
 * CostmapMapContext: activate/deactivate the local map + per-cycle locked
 * map reads + safetyCheck->isCurrent; the zeroed stop command is still
 * guaranteed by ControllerExecution). The execution configurator (server
 * assembly: goal/progress checkers + velocity limits + robot state source)
 * is injected via configure.
 */
class CostmapControllerAction : public ayanami::components::ControllerAction {
 public:
  using ExecutionConfigurator =
      std::function<void(ayanami::components::ControllerExecution::Ptr)>;

  CostmapControllerAction(
      ayanami::abstract::AbstractController::Ptr controller,
      CostmapMapContext::Ptr ctx, ExecutionConfigurator configure = nullptr)
      : ControllerAction(std::move(controller)),
        ctx(std::move(ctx)),
        configure(std::move(configure)) {}

  ~CostmapControllerAction() override = default;

 protected:
  ayanami::components::ControllerExecution::Ptr createExecution() override {
    if (!ctx) {
      return ControllerAction::createExecution();
    }
    auto exec = std::make_shared<CostmapControllerExecution>(
        "controller", controller, ctx);
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
