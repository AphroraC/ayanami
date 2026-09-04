// 测试插件: costmap 族 recovery (SoCostmapRecovery), 公共工厂导出。
//
// 旧 API runRecovery 一次性返回成功; 新协议下 tick() 首个周期即返回 SUCCESS
// 终态 (保持原一次性语义)。

#include <ayanami/costmap/core/costmap_recovery.hpp>
#include <ayanami/messages/failure.hpp>
#include <ayanami/types.hpp>
#include <ayanami/utility/extension_module.hpp>

namespace {

using ayanami::RecoveryCycleData;
using ayanami::RecoveryResultT;
using ayanami::RecoveryStatus;

class SoCostmapRecovery : public ayanami::costmap::CostmapRecovery {
 public:
  void configure(const ayanami::PluginContext& /*context*/,
                 ayanami::costmap::CostmapContext::ConstPtr /*map*/) override {}
  RecoveryResultT tick() override {
    RecoveryCycleData data;
    data.status = RecoveryStatus::SUCCESS;
    return RecoveryResultT::success(std::move(data));
  }
  bool cancel() override { return true; }
};

}  // namespace

extern "C" ayanami::ExtensionModule* create_extension_module() {
  return new SoCostmapRecovery;
}
