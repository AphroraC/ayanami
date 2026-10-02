#pragma once

#include <ayanami/abstract/core/abstract_recovery.hpp>
#include <ayanami/messages/recovery_params.hpp>
#include <ayanami/types.hpp>

namespace ayanami {
namespace modules {

/**
 * @brief No-op recovery module (abstract family): the test-phase placeholder
 * while no real recovery behavior is deployed.
 *
 * Runs the FULL start/tick/finish protocol and settles after exactly one
 * tick: tick() logs the invocation (with the triggering failure context's
 * error code) and reports SUCCESS. The recovery ORCHESTRATION in the FSM
 * (retry budget, non-recoverable short-circuit, rotation cursor) stays
 * exercised and observable -- swapping in real behaviors later is a plugin
 * change, not a framework change.
 */
class NothingRecovery final : public ayanami::abstract::AbstractRecovery {
 public:
  using Ptr = std::shared_ptr<NothingRecovery>;
  NothingRecovery() = default;
  ~NothingRecovery() override = default;

  void start() override {}
  ResultT<void> tick(RecoveryCycleData& cycle) override;
  void finish() override {}
};

}  // namespace modules
}  // namespace ayanami
