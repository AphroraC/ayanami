#pragma once

#include <functional>
#include <memory>
#include <string>
#include <utility>

#include "ayanami/abstract/components/recovery_action.hpp"
#include "ayanami/costmap/components/costmap_map_context.hpp"
#include "ayanami/costmap/components/costmap_recovery_execution.hpp"

namespace ayanami {
namespace costmap {

/**
 * @brief The costmap variant of the recovery Action (production path).
 *
 * Reuses the entire RecoveryAction protocol and only overrides the
 * createExecution factory:
 * returns a CostmapRecoveryExecution (activate/deactivate global+local +
 * locked map reads; map-modifying recoveries submit commands via
 * CommandPort instead of writing the master grid under a lock directly).
 * The execution configurator (server assembly: velocity limits) is
 * injected via configure; a no-op by default.
 */
class CostmapRecoveryAction : public ayanami::components::RecoveryAction {
 public:
  using ExecutionConfigurator =
      std::function<void(ayanami::components::RecoveryExecution::Ptr)>;

  CostmapRecoveryAction(ayanami::abstract::AbstractRecovery::Ptr recovery,
                        CostmapMapContext::Ptr ctx,
                        ExecutionConfigurator configure = nullptr)
      : RecoveryAction(std::move(recovery)),
        ctx(std::move(ctx)),
        configure(std::move(configure)) {}

  ~CostmapRecoveryAction() override = default;

 protected:
  ayanami::components::RecoveryExecution::Ptr createExecution() override {
    if (!ctx) {
      return RecoveryAction::createExecution();
    }
    auto exec = std::make_shared<CostmapRecoveryExecution>(
        "recovery", recovery, ctx);
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
