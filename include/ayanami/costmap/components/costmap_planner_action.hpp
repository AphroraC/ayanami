#pragma once

#include <memory>
#include <utility>

#include <costmap_server/costmap_wrapper.hpp>

#include "ayanami/components/planner_action.hpp"
#include "ayanami/costmap/components/costmap_planner_execution.hpp"

namespace ayanami {
namespace costmap {

/**
 * @brief 全局路径规划的 costmap 版 Action (spec §5.4)。
 *
 * 复用 PlannerAction 全部协议 (TaskHandle 直连 + 旁路 emit + 抢占),
 * 只覆写 createExecution 工厂: 返回 CostmapPlannerExecution
 * (带 CostmapWrapper 激活/停用/持锁/safetyCheck)。
 */
class CostmapPlannerAction : public ayanami::components::PlannerAction {
 public:
  CostmapPlannerAction(ayanami::abstract::AbstractPlanner::Ptr planner,
                       costmap_2d::CostmapWrapper::Ptr costmap_wrapper)
      : PlannerAction(std::move(planner)),
        costmap_wrapper_(std::move(costmap_wrapper)) {}

  ~CostmapPlannerAction() override = default;

 protected:
  ayanami::components::PlannerExecution::Ptr createExecution() override {
    return std::make_shared<CostmapPlannerExecution>("planner", planner_,
                                                     costmap_wrapper_);
  }

 private:
  costmap_2d::CostmapWrapper::Ptr costmap_wrapper_;
};

}  // namespace costmap
}  // namespace ayanami
