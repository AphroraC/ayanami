// Test plugin: costmap-family recovery (SoCostmapRecovery), common factory export.
//
// The old API runRecovery returned success in one shot; under the new protocol the first tick cycle returns the SUCCESS
// terminal state (the original one-shot semantics preserved).

#include <ayanami/costmap/core/costmap_recovery.hpp>
#include <ayanami/messages/failure.hpp>
#include <ayanami/types.hpp>
#include <ayanami/utility/extension_module.hpp>

namespace {

using ayanami::RecoveryCycleData;
using ayanami::ResultT;
using ayanami::RecoveryStatus;

class SoCostmapRecovery : public ayanami::costmap::CostmapRecovery {
 public:
  void configure(const ayanami::PluginContext& /*context*/,
                 ayanami::costmap::CostmapMapContext::ConstPtr /*map*/) override {}
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
  return new SoCostmapRecovery;
}
