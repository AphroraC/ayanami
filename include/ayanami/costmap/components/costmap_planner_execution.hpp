#pragma once

#include <memory>
#include <string>
#include <utility>

#include <costmap_server/costmap_wrapper.hpp>

#include "ayanami/abstract/components/planner_execution.hpp"
#include "ayanami/costmap/components/costmap_context.hpp"
#include "ayanami/costmap/core/costmap_planner.hpp"

namespace ayanami {
namespace costmap {

/**
 * @brief 全局路径规划的 costmap 版执行 (spec §5.4, 对标 MBF PlannerCostmapExecution)。
 *
 * 在 PlannerExecution 之上绑定 costmap 上下文:
 * - preRun: checkActivate() 幂等激活 global costmap;
 * - postRun: checkDeactivate() 幂等停用 global costmap;
 * - safetyCheck: global wrapper isCurrent() (地图新鲜度, 原子读);
 * - callPlanner: 经 abstract 签名直调插件; 插件内部经持有的地图模块
 *   lockGlobal() 加锁读图 (shared 锁, 零拷贝)。
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
  FailureCode callPlanner(const ayanami::geometry_msgs::PoseStamped& start,
                          const ayanami::geometry_msgs::PoseStamped& goal,
                          double tolerance, ayanami::nav_msgs::Path& plan,
                          double& cost, std::string& message) override {
    auto* typed = dynamic_cast<CostmapPlanner*>(planner.get());
    if (!typed) {
      message = "CostmapPlannerExecution: plugin is not a CostmapPlanner";
      return FailureCode::InternalError;
    }
    if (!ctx) {
      message = "CostmapPlannerExecution: no costmap module";
      return FailureCode::MapUnavailable;
    }
    return typed->makePlan(start, goal, tolerance, plan, cost, message);
  }

 private:
  std::string name;
  CostmapContext::Ptr ctx;
};

}  // namespace costmap
}  // namespace ayanami
