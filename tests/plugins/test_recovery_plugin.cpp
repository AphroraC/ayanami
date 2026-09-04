// 测试插件: 成功 recovery (SuccessRecovery), 经公共工厂 create_extension_module 导出。
//
// 旧 API runRecovery 一次性返回成功; 新协议下由 start() 做准备, tick() 首个
// 周期即返回 SUCCESS 终态 (保持原一次性语义)。

#include <ayanami/abstract/core/abstract_recovery.hpp>
#include <ayanami/messages/failure.hpp>
#include <ayanami/types.hpp>
#include <ayanami/utility/extension_module.hpp>

namespace {

using ayanami::RecoveryCycleData;
using ayanami::RecoveryResultT;
using ayanami::RecoveryStatus;

class SuccessRecovery : public ayanami::abstract::AbstractRecovery {
public:
  RecoveryResultT tick() override {
    RecoveryCycleData data;
    data.status = RecoveryStatus::SUCCESS;
    return RecoveryResultT::success(std::move(data));
  }
  bool cancel() override { return true; }
};

}  // namespace

extern "C" ayanami::ExtensionModule* create_extension_module() {
  return new SuccessRecovery;
}
