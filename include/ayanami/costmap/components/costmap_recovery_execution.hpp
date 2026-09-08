#pragma once

#include <memory>
#include <string>
#include <utility>

#include "ayanami/abstract/components/recovery_execution.hpp"
#include "ayanami/costmap/components/costmap_map_context.hpp"
#include "ayanami/costmap/core/costmap_recovery.hpp"

namespace ayanami {
namespace costmap {

/**
 * @brief The costmap variant of the recovery execution.
 *
 * Binds a costmap context on top of RecoveryExecution:
 * - preRun: checkActivate() on both global+local; postRun:
 *   checkDeactivate() on both;
 * - safetyCheck: returns true only when both wrappers are current (hook
 *   retained; the recovery cycle loop does not gate on it -- recovery must
 *   always advance to a terminal state and publish zeros);
 * - callRecoveryTick: typed call into CostmapRecovery; collision-type
 *   recovery plugins read the map under a lock internally via their held
 *   map module; map-modifying recoveries (clear) submit commands via
 *   CostmapCommandPort, executed by the sole writer
 *   (CostmapNavigationServer).
 */
class CostmapRecoveryExecution : public ayanami::components::RecoveryExecution {
 public:
  using Ptr = std::shared_ptr<CostmapRecoveryExecution>;

  CostmapRecoveryExecution(const std::string& name,
                           ayanami::abstract::AbstractRecovery::Ptr recovery,
                           CostmapMapContext::Ptr ctx)
      : RecoveryExecution(std::move(recovery)),
        name(name),
        ctx(std::move(ctx)) {}

  ~CostmapRecoveryExecution() override = default;

  void preRun() override {
    if (ctx && ctx->globalCostmap()) {
      ctx->globalCostmap()->checkActivate();
    }
    if (ctx && ctx->localCostmap()) {
      ctx->localCostmap()->checkActivate();
    }
  }

  void postRun() override {
    if (ctx && ctx->globalCostmap()) {
      ctx->globalCostmap()->checkDeactivate();
    }
    if (ctx && ctx->localCostmap()) {
      ctx->localCostmap()->checkDeactivate();
    }
  }

  bool safetyCheck() override {
    return (!ctx || !ctx->globalCostmap() ||
            ctx->globalCostmap()->isCurrent()) &&
           (!ctx || !ctx->localCostmap() ||
            ctx->localCostmap()->isCurrent());
  }

 protected:
  ResultT<void> callRecoveryTick(RecoveryCycleData& cycle) override {
    auto* typed = dynamic_cast<CostmapRecovery*>(recovery.get());
    if (!typed) {
      return ResultT<void>::failure(
          ErrorCode::INTERNAL_ERROR,
          "CostmapRecoveryExecution: plugin is not a CostmapRecovery");
    }
    if (!ctx) {
      return ResultT<void>::failure(
          ErrorCode::MAP_UNAVAILABLE,
          "CostmapRecoveryExecution: no costmap module");
    }
    return typed->tick(cycle);
  }

 private:
  std::string name;
  CostmapMapContext::Ptr ctx;
};

}  // namespace costmap
}  // namespace ayanami
