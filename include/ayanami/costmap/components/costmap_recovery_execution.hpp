#pragma once

#include <memory>
#include <string>
#include <utility>

#include "ayanami/abstract/components/recovery_execution.hpp"
#include "ayanami/costmap/components/costmap_map_context.hpp"
#include "ayanami/costmap/core/costmap_recovery.hpp"

namespace ayanami {
namespace costmap {

/**
 * @brief 恢复行为的 costmap 版执行。
 *
 * 在 RecoveryExecution 之上绑定 costmap 上下文:
 * - preRun: global+local 两者 checkActivate(); postRun: 两者 checkDeactivate();
 * - safetyCheck: 两个 wrapper 都 current 才返回 true (保留钩子; recovery
 *   周期循环不以之为门, 恢复必须始终推进到终态并补零);
 * - callRecoveryTick: typed 调用 CostmapRecovery; 碰撞类恢复插件内部经
 *   持有的地图模块加锁读图; 需要改图的恢复 (clear) 经 CostmapCommandPort
 *   提交命令, 由唯一写者 (CostmapNavigationServer) 执行。
 */
class CostmapRecoveryExecution : public ayanami::components::RecoveryExecution {
 public:
  using Ptr = std::shared_ptr<CostmapRecoveryExecution>;

  CostmapRecoveryExecution(const std::string& name,
                           ayanami::abstract::AbstractRecovery::Ptr recovery,
                           CostmapMapContext::Ptr ctx)
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
  RecoveryResultT callRecoveryTick() override {
    auto* typed = dynamic_cast<CostmapRecovery*>(recovery.get());
    if (!typed) {
      return RecoveryResultT::failure(
          ErrorCode::INTERNAL_ERROR,
          "CostmapRecoveryExecution: plugin is not a CostmapRecovery");
    }
    if (!ctx) {
      return RecoveryResultT::failure(
          ErrorCode::MAP_UNAVAILABLE,
          "CostmapRecoveryExecution: no costmap module");
    }
    return typed->tick();
  }

 private:
  std::string name;
  CostmapMapContext::Ptr ctx;
};

}  // namespace costmap
}  // namespace ayanami
