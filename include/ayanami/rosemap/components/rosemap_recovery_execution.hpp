#pragma once

#include <memory>
#include <string>
#include <utility>

#include "ayanami/abstract/components/recovery_execution.hpp"
#include "ayanami/rosemap/core/rosemap_map_context.hpp"
#include "ayanami/rosemap/core/rosemap_recovery.hpp"

namespace ayanami {
namespace rosemap {

/// @brief The rosemap variant of the recovery execution.
///
/// Binds a rosemap field context on top of RecoveryExecution:
/// - safetyCheck: field freshness (hook retained; the recovery cycle loop
///   does not gate on it -- recovery must always advance to a terminal
///   state and publish zeros);
/// - callRecoveryTick: typed call into RosemapRecovery; escape-type modules
///   query the distance field / gradient per tick through their held field
///   module; field-modifying recoveries submit commands via
///   RosemapCommandPort, executed by the sole writer.
class RosemapRecoveryExecution : public ayanami::components::RecoveryExecution {
 public:
  using Ptr = std::shared_ptr<RosemapRecoveryExecution>;

  RosemapRecoveryExecution(const std::string& name,
                           ayanami::abstract::AbstractRecovery::Ptr recovery,
                           RosemapMapContext::Ptr ctx)
      : RecoveryExecution(std::move(recovery)),
        name(name),
        ctx(std::move(ctx)) {}

  ~RosemapRecoveryExecution() override = default;

  bool safetyCheck() override { return !ctx || ctx->current(); }

 protected:
  ResultT<void> callRecoveryTick(RecoveryCycleData& cycle) override {
    auto* typed = dynamic_cast<RosemapRecovery*>(recovery.get());
    if (!typed) {
      return ResultT<void>::failure(
          ErrorCode::INTERNAL_ERROR,
          "RosemapRecoveryExecution: module is not a RosemapRecovery");
    }
    if (!ctx) {
      return ResultT<void>::failure(
          ErrorCode::MAP_UNAVAILABLE,
          "RosemapRecoveryExecution: no rosemap field");
    }
    return typed->tick(cycle);
  }

 private:
  std::string name;
  RosemapMapContext::Ptr ctx;
};

}  // namespace rosemap
}  // namespace ayanami
