#pragma once

#include <functional>
#include <memory>
#include <string>
#include <utility>

#include "ayanami/abstract/components/recovery_action.hpp"
#include "ayanami/costmap/components/costmap_context.hpp"
#include "ayanami/costmap/components/costmap_recovery_execution.hpp"

namespace ayanami {
namespace costmap {

/**
 * @brief 恢复行为的 costmap 版 Action (spec §5.4, 生产路径)。
 *
 * 复用 RecoveryAction 全部协议, 只覆写 createExecution 工厂:
 * 返回 CostmapRecoveryExecution (激活/停用 global+local + 加锁读图; 改图
 * 类恢复经 CommandPort 提交命令, 不再持锁直写 master)。执行配置器
 * (server 装配: 速度上限) 经 configure 注入, 默认无操作。
 */
class CostmapRecoveryAction : public ayanami::components::RecoveryAction {
 public:
  using ExecutionConfigurator =
      std::function<void(ayanami::components::RecoveryExecution::Ptr)>;

  CostmapRecoveryAction(ayanami::abstract::AbstractRecovery::Ptr recovery,
                        CostmapContext::Ptr ctx,
                        ExecutionConfigurator configure = nullptr)
      : RecoveryAction(std::move(recovery)),
        ctx(std::move(ctx)),
        configure(std::move(configure)) {}

  ~CostmapRecoveryAction() override = default;

 protected:
  ayanami::components::RecoveryExecution::Ptr createExecution() override {
    if (!ctx) {
      return RecoveryAction::createExecution();
    }
    auto exec = std::make_shared<CostmapRecoveryExecution>(
        "recovery", recovery, ctx);
    if (configure) {
      configure(exec);
    }
    return exec;
  }

 private:
  CostmapContext::Ptr ctx;
  ExecutionConfigurator configure;
};

}  // namespace costmap
}  // namespace ayanami
