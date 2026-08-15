#pragma once

#include <memory>
#include <utility>

#include <costmap_server/costmap_wrapper.hpp>

#include "ayanami/components/recovery_action.hpp"
#include "ayanami/costmap/components/costmap_recovery_execution.hpp"

namespace ayanami {
namespace costmap {

/**
 * @brief 恢复行为的 costmap 版 Action (spec §5.4)。
 *
 * 复用 RecoveryAction 全部协议, 只覆写 createExecution 工厂:
 * 返回 CostmapRecoveryExecution (激活/停用 global+local 两 costmap + 持锁)。
 */
class CostmapRecoveryAction : public ayanami::components::RecoveryAction {
 public:
  CostmapRecoveryAction(ayanami::abstract::AbstractRecovery::Ptr recovery,
                        costmap_2d::CostmapWrapper::Ptr global,
                        costmap_2d::CostmapWrapper::Ptr local)
      : RecoveryAction(std::move(recovery)),
        global_costmap_(std::move(global)),
        local_costmap_(std::move(local)) {}

  ~CostmapRecoveryAction() override = default;

 protected:
  ayanami::components::RecoveryExecution::Ptr createExecution() override {
    return std::make_shared<CostmapRecoveryExecution>(
        "recovery", recovery_, global_costmap_, local_costmap_);
  }

 private:
  costmap_2d::CostmapWrapper::Ptr global_costmap_;
  costmap_2d::CostmapWrapper::Ptr local_costmap_;
};

}  // namespace costmap
}  // namespace ayanami
