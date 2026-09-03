// 测试插件: 非 costmap 族 recovery (PlainRecovery), 验证 costmap 宿主拒绝路径。

#include <ayanami/abstract/core/abstract_recovery.hpp>
#include <ayanami/messages/failure.hpp>
#include <ayanami/utility/extension_module.hpp>

namespace {

using ayanami::FailureCode;

class PlainRecovery : public ayanami::abstract::AbstractRecovery {
 public:
  FailureCode runRecovery(std::string& /*message*/) override {
    return FailureCode::Canceled;
  }
  bool cancel() override { return true; }
};

}  // namespace

extern "C" ayanami::ExtensionModule* create_extension_module() {
  return new PlainRecovery;
}