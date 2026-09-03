#pragma once

#include <functional>
#include <memory>
#include <string>
#include <utility>

#include "ayanami/abstract/components/planner_action.hpp"
#include "ayanami/costmap/components/costmap_context.hpp"
#include "ayanami/costmap/components/costmap_planner_execution.hpp"

namespace ayanami {
namespace costmap {

/**
 * @brief 全局路径规划的 costmap 版 Action (spec §5.4, 生产路径)。
 *
 * 复用 PlannerAction 全部协议 (TaskHandle 直连 + 旁路 emit + 抢占),
 * 只覆写 createExecution 工厂: 返回 CostmapPlannerExecution
 * (带 CostmapContext 的激活动作 / 停用 / safetyCheck / 加锁读图)。
 * 执行配置器 (server 装配: 无额外注入) 经 configure 注入, 默认无操作。
 */
class CostmapPlannerAction : public ayanami::components::PlannerAction {
 public:
  using ExecutionConfigurator =
      std::function<void(ayanami::components::PlannerExecution::Ptr)>;

  CostmapPlannerAction(ayanami::abstract::AbstractPlanner::Ptr planner,
                       CostmapContext::Ptr ctx,
                       ExecutionConfigurator configure = nullptr)
      : PlannerAction(std::move(planner)),
        ctx(std::move(ctx)),
        configure(std::move(configure)) {}

  ~CostmapPlannerAction() override = default;

 protected:
  ayanami::components::PlannerExecution::Ptr createExecution() override {
    if (!ctx) {
      return PlannerAction::createExecution();
    }
    auto exec = std::make_shared<CostmapPlannerExecution>(
        "planner", planner, ctx);
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
