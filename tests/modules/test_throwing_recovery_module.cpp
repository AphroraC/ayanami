// Test module: a recovery whose start throws (for exception-path closure regression).
//
// Reproduces the original scenario of high-priority defect #1: a module start exception used to leave RecoveryExecution
// stuck in the STARTED intermediate state -> the action-driving task polled forever -> recovery_exclusive deadlock.

#include <ayanami/abstract/core/abstract_recovery.hpp>
#include <ayanami/types.hpp>
#include <ayanami/utility/extension_module.hpp>

#include <stdexcept>

namespace {

class ThrowingStartRecovery : public ayanami::abstract::AbstractRecovery {
 public:
  void start() override { throw std::runtime_error("recovery start exploded"); }

  ayanami::ResultT<void> tick(ayanami::RecoveryCycleData& /*cycle*/) override {
    return ayanami::ResultT<void>::success();
  }
};

}  // namespace

extern "C" ayanami::ExtensionModule* create_extension_module() {
  return new ThrowingStartRecovery;
}
