#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <utility>

#include <costmap_server/costmap_wrapper.hpp>

#include "ayanami/components/planner_execution.hpp"

namespace ayanami {
namespace costmap {

/**
 * @brief 全局路径规划的 costmap 版执行 (spec §5.4, 对标 MBF PlannerCostmapExecution)。
 *
 * 在 PlannerExecution 之上注入 CostmapWrapper 生命周期钩子:
 * - preRun: checkActivate() 幂等激活 costmap (可重复执行不重复 act);
 * - postRun: checkDeactivate() 幂等停用 costmap;
 * - safetyCheck: costmap_->isCurrent() (地图新鲜度);
 * - run: 覆写加 recursive_mutex scoped_lock 再调父 (执行全程持锁)。
 */
class CostmapPlannerExecution : public ayanami::components::PlannerExecution {
 public:
  using Ptr = std::shared_ptr<CostmapPlannerExecution>;

  CostmapPlannerExecution(const std::string& name,
                          ayanami::abstract::AbstractPlanner::Ptr planner,
                          costmap_2d::CostmapWrapper::Ptr costmap_wrapper)
      : PlannerExecution(std::move(planner)),
        name_(name),
        costmap_wrapper_(std::move(costmap_wrapper)) {}

  ~CostmapPlannerExecution() override = default;

  void preRun() override {
    if (costmap_wrapper_) {
      costmap_wrapper_->checkActivate();
    }
  }

  void postRun() override {
    if (costmap_wrapper_) {
      costmap_wrapper_->checkDeactivate();
    }
  }

  bool safetyCheck() override {
    return !costmap_wrapper_ || costmap_wrapper_->isCurrent();
  }

 protected:
  void run() override {
    if (costmap_wrapper_) {
      std::lock_guard<std::recursive_mutex> lock(costmap_wrapper_->mutex());
      PlannerExecution::run();
    } else {
      PlannerExecution::run();
    }
  }

 private:
  std::string name_;
  costmap_2d::CostmapWrapper::Ptr costmap_wrapper_;
};

}  // namespace costmap
}  // namespace ayanami
