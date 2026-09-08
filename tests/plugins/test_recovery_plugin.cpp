// Test plugin: success recovery (SuccessRecovery), exported via the common create_extension_module factory.
//
// The old API runRecovery returned success in one shot; under the new protocol start prepares and the first tick
// cycle returns the SUCCESS terminal state (the original one-shot semantics preserved).

#include <ayanami/abstract/core/abstract_recovery.hpp>
#include <ayanami/messages/failure.hpp>
#include <ayanami/types.hpp>
#include <ayanami/utility/extension_module.hpp>

namespace {

using ayanami::RecoveryCycleData;
using ayanami::ResultT;
using ayanami::RecoveryStatus;

class SuccessRecovery : public ayanami::abstract::AbstractRecovery {
public:
  ResultT<void> tick(RecoveryCycleData& cycle) override {
    RecoveryCycleData data;
    data.status = RecoveryStatus::SUCCESS;
    cycle = std::move(data);
    return ResultT<void>::success();
  }
  bool cancel() override { return true; }
};

}  // namespace

extern "C" ayanami::ExtensionModule* create_extension_module() {
  return new SuccessRecovery;
}
