// 测试插件: start() 抛异常的 recovery (异常路径闭合回归用)。
//
// 复现高优缺陷 #1 的原始场景: 插件 start() 抛异常曾使 RecoveryExecution
// 停在 STARTED 中间态 → action 驱动任务死轮询 → recovery_exclusive 死锁。

#include <ayanami/abstract/core/abstract_recovery.hpp>
#include <ayanami/types.hpp>
#include <ayanami/utility/extension_module.hpp>

#include <stdexcept>

namespace {

class ThrowingStartRecovery : public ayanami::abstract::AbstractRecovery {
 public:
  void start() override { throw std::runtime_error("recovery start exploded"); }

  ayanami::RecoveryResultT tick() override {
    return ayanami::RecoveryResultT::success(
        ayanami::RecoveryCycleData{});
  }
};

}  // namespace

extern "C" ayanami::ExtensionModule* create_extension_module() {
  return new ThrowingStartRecovery;
}
