#pragma once

#include <memory>
#include <string>
#include <utility>

#include "ayanami/abstract/components/planner_execution.hpp"
#include "ayanami/rosemap/core/rosemap_map_context.hpp"
#include "ayanami/rosemap/core/rosemap_planner.hpp"

namespace ayanami {
namespace rosemap {

/// @brief The rosemap variant of the global planning execution.
///
/// Binds a rosemap field context on top of PlannerExecution:
/// - safetyCheck: field freshness (snapshot exists + optional stale-timeout
///   gate); no activation machinery (the field has none).
/// - callPlanner: typed call into RosemapPlanner; the module reads its held
///   snapshot internally (immutable, no locks).
class RosemapPlannerExecution : public ayanami::components::PlannerExecution {
 public:
  using Ptr = std::shared_ptr<RosemapPlannerExecution>;

  RosemapPlannerExecution(const std::string& name,
                          ayanami::abstract::AbstractPlanner::Ptr planner,
                          RosemapMapContext::Ptr ctx)
      : PlannerExecution(std::move(planner)), name(name), ctx(std::move(ctx)) {}

  ~RosemapPlannerExecution() override = default;

  bool safetyCheck() override { return !ctx || ctx->current(); }

 protected:
  ResultT<void> callPlanner(const ayanami::geometry_msgs::PoseStamped& start,
                            const ayanami::geometry_msgs::PoseStamped& goal,
                            double tolerance,
                            ayanami::nav_msgs::Path& plan) override {
    auto* typed = dynamic_cast<RosemapPlanner*>(planner.get());
    if (!typed) {
      return ResultT<void>::failure(
          ErrorCode::INTERNAL_ERROR,
          "RosemapPlannerExecution: module is not a RosemapPlanner");
    }
    if (!ctx) {
      return ResultT<void>::failure(ErrorCode::MAP_UNAVAILABLE,
                                    "RosemapPlannerExecution: no rosemap field");
    }
    return typed->makePlan(start, goal, tolerance, plan);
  }

 private:
  std::string name;
  RosemapMapContext::Ptr ctx;
};

}  // namespace rosemap
}  // namespace ayanami
