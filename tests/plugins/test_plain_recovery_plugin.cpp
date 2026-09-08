// Test plugin: non-costmap-family recovery (PlainRecovery), verifies the costmap host reject path.
//
// The old API runRecovery returned Canceled in one shot; under the new protocol the first tick cycle returns
// the cancelled terminal state (the original one-shot semantics preserved).

#include <ayanami/abstract/core/abstract_recovery.hpp>
#include <ayanami/messages/failure.hpp>
#include <ayanami/types.hpp>
#include <ayanami/utility/extension_module.hpp>

namespace {

using ayanami::RecoveryCycleData;
using ayanami::ResultT;
class PlainRecovery : public ayanami::abstract::AbstractRecovery {
 public:
  ResultT<void> tick(RecoveryCycleData& cycle) override {
    return ResultT<void>::failure(ayanami::ErrorCode::CANCELED, "canceled");
  }
  bool cancel() override { return true; }
};

}  // namespace

extern "C" ayanami::ExtensionModule* create_extension_module() {
  return new PlainRecovery;
}
