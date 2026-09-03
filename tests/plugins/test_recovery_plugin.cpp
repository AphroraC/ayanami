// 测试插件: 成功 recovery (SuccessRecovery), 经公共工厂 create_extension_module 导出。

#include <ayanami/abstract/core/abstract_recovery.hpp>
#include <ayanami/messages/failure.hpp>
#include <ayanami/utility/extension_module.hpp>

namespace {

using ayanami::FailureCode;

class SuccessRecovery : public ayanami::abstract::AbstractRecovery {
public:
  FailureCode runRecovery(std::string& /*message*/) override {
    return FailureCode::None;
  }
  bool cancel() override { return true; }
};

}  // namespace

extern "C" ayanami::ExtensionModule* create_extension_module() {
  return new SuccessRecovery;
}