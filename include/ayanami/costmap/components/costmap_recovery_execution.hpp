#pragma once

#include <memory>
#include <string>
#include <utility>

#include <costmap_server/costmap_wrapper.hpp>

#include "ayanami/abstract/components/recovery_execution.hpp"
#include "ayanami/costmap/components/costmap_context.hpp"
#include "ayanami/costmap/core/costmap_recovery.hpp"

namespace ayanami {
namespace costmap {

/**
 * @brief 恢复行为的 costmap 版执行 (spec §5.4, 对标 MBF RecoveryCostmapExecution)。
 *
 * 在 RecoveryExecution 之上绑定 costmap 上下文:
 * - preRun: global+local 两者 checkActivate(); postRun: 两者 checkDeactivate();
 * - safetyCheck: 两个 wrapper 都 current 才返回 true (保留钩子; recovery 周期
 *   循环不以之为门, 恢复必须始终推进到终态并补零);
 * - callRecoveryCycle: 经 abstract 签名 (onCycleUpdate(message)) 直调插件;
 *   碰撞类恢复插件内部经持有的地图模块加锁读图; 需要改图的恢复 (clear) 经
 *   CostmapCommandPort 提交命令, 由唯一写者 (CostmapNavigationServer) 执行。
 */
class CostmapRecoveryExecution : public ayanami::components::RecoveryExecution {
 public:
  using Ptr = std::shared_ptr<CostmapRecoveryExecution>;

  CostmapRecoveryExecution(const std::string& name,
                           ayanami::abstract::AbstractRecovery::Ptr recovery,
                           CostmapContext::Ptr ctx)
      : RecoveryExecution(std::move(recovery)),
        name(name),
        ctx(std::move(ctx)) {}

  ~CostmapRecoveryExecution() override = default;

  void preRun() override {
    if (ctx && ctx->globalCostmap()) {
      ctx->globalCostmap()->checkActivate();
    }
    if (ctx && ctx->localCostmap()) {
      ctx->localCostmap()->checkActivate();
    }
  }

  void postRun() override {
    if (ctx && ctx->globalCostmap()) {
      ctx->globalCostmap()->checkDeactivate();
    }
    if (ctx && ctx->localCostmap()) {
      ctx->localCostmap()->checkDeactivate();
    }
  }

  bool safetyCheck() override {
    return (!ctx || !ctx->globalCostmap() ||
            ctx->globalCostmap()->isCurrent()) &&
           (!ctx || !ctx->localCostmap() ||
            ctx->localCostmap()->isCurrent());
  }

 protected:
  ayanami::abstract::RecoveryCycleResult callRecoveryCycle(
      std::string& message) override {
    auto* typed = dynamic_cast<CostmapRecovery*>(recovery.get());
    if (!typed) {
      message = "CostmapRecoveryExecution: plugin is not a CostmapRecovery";
      ayanami::abstract::RecoveryCycleResult r;
      r.status = ayanami::abstract::RecoveryCycleResult::Status::FAILED;
      r.failure = FailureCode::InternalError;
      return r;
    }
    if (!ctx) {
      message = "CostmapRecoveryExecution: no costmap module";
      ayanami::abstract::RecoveryCycleResult result;
      result.status = ayanami::abstract::RecoveryCycleResult::Status::FAILED;
      result.failure = FailureCode::MapUnavailable;
      return result;
    }
    return typed->onCycleUpdate(message);
  }

 private:
  std::string name;
  CostmapContext::Ptr ctx;
};

}  // namespace costmap
}  // namespace ayanami
