#pragma once

#include <memory>
#include <string>
#include <utility>

#include "ayanami/abstract/components/planner_execution.hpp"
#include "ayanami/costmap/components/costmap_context.hpp"
#include "ayanami/costmap/core/costmap_planner.hpp"

namespace ayanami {
namespace costmap {

/**
 * @brief 全局路径规划的 costmap 版执行。
 *
 * 在 PlannerExecution 之上绑定 costmap 上下文:
 * - preRun: checkActivate() 幂等激活 global costmap;
 * - postRun: checkDeactivate() 幂等停用 global costmap;
 * - safetyCheck: global wrapper isCurrent() (地图新鲜度, 原子读);
 * - callPlanner: typed 调用 CostmapPlanner; 插件内部经持有的地图模块
 *   lockGlobal() 加锁读图 (shared 锁, 零拷贝, 单次规划 <10ms)。
 */
class CostmapPlannerExecution : public ayanami::components::PlannerExecution {
 public:
  using Ptr = std::shared_ptr<CostmapPlannerExecution>;

  CostmapPlannerExecution(const std::string& name,
                          ayanami::abstract::AbstractPlanner::Ptr planner,
                          CostmapContext::Ptr ctx)
      : PlannerExecution(std::move(planner)),
        name(name),
        ctx(std::move(ctx)) {}

  ~CostmapPlannerExecution() override = default;

  void preRun() override {
    if (ctx && ctx->globalCostmap()) {
      ctx->globalCostmap()->checkActivate();
    }
  }

  void postRun() override {
    if (ctx && ctx->globalCostmap()) {
      ctx->globalCostmap()->checkDeactivate();
    }
  }

  bool safetyCheck() override {
    return !ctx || !ctx->globalCostmap() || ctx->globalCostmap()->isCurrent();
  }

 protected:
  PlannerResultT callPlanner(const ayanami::geometry_msgs::PoseStamped& start,
                             const ayanami::geometry_msgs::PoseStamped& goal,
                             double tolerance) override {
    auto* typed = dynamic_cast<CostmapPlanner*>(planner.get());
    if (!typed) {
      return PlannerResultT::failure(
          ErrorCode::INTERNAL_ERROR,
          "CostmapPlannerExecution: plugin is not a CostmapPlanner");
    }
    if (!ctx) {
      return PlannerResultT::failure(
          ErrorCode::MAP_UNAVAILABLE,
          "CostmapPlannerExecution: no costmap module");
    }
    return typed->makePlan(start, goal, tolerance);
  }

 private:
  std::string name;
  CostmapContext::Ptr ctx;
};

}  // namespace costmap
}  // namespace ayanami
