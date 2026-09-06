// 测试插件: 非 costmap 族 recovery (PlainRecovery), 验证 costmap 宿主拒绝路径。
//
// 旧 API runRecovery 一次性返回 Canceled; 新协议下 tick 首个周期即返回
// 取消终态 (保持原一次性语义)。

#include <ayanami/abstract/core/abstract_recovery.hpp>
#include <ayanami/messages/failure.hpp>
#include <ayanami/types.hpp>
#include <ayanami/utility/extension_module.hpp>

namespace {

using ayanami::RecoveryResultT;

class PlainRecovery : public ayanami::abstract::AbstractRecovery {
 public:
  RecoveryResultT tick() override {
    return RecoveryResultT::failure(ayanami::ErrorCode::CANCELED, "canceled");
  }
  bool cancel() override { return true; }
};

}  // namespace

extern "C" ayanami::ExtensionModule* create_extension_module() {
  return new PlainRecovery;
}
