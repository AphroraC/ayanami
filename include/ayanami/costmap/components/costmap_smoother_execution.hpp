#pragma once

#include <memory>
#include <string>
#include <utility>

#include "ayanami/abstract/components/smoother_execution.hpp"
#include "ayanami/costmap/components/costmap_map_context.hpp"
#include "ayanami/costmap/core/costmap_smoother.hpp"

namespace ayanami {
namespace costmap {

/**
 * @brief The costmap variant of the path smoothing execution.
 *
 * Binds a costmap context on top of SmootherExecution:
 * - preRun/postRun: checkActivate/checkDeactivate on BOTH maps (family read
 *   contract: the smoother may read the global and the local map);
 * - safetyCheck: BOTH wrappers current (missing wrapper counts as current);
 * - callSmoother: typed in-place call into CostmapSmoother; the module
 *   reads the maps under a lock internally as needed.
 */
class CostmapSmootherExecution : public ayanami::components::SmootherExecution {
 public:
  using Ptr = std::shared_ptr<CostmapSmootherExecution>;

  CostmapSmootherExecution(const std::string& name,
                           ayanami::abstract::AbstractSmoother::Ptr smoother,
                           CostmapMapContext::Ptr ctx)
      : SmootherExecution(std::move(smoother)),
        name(name),
        ctx(std::move(ctx)) {}

  ~CostmapSmootherExecution() override = default;

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
           (!ctx || !ctx->localCostmap() || ctx->localCostmap()->isCurrent());
  }

 protected:
  ResultT<void> callSmoother(ayanami::nav_msgs::Path& path,
                             double max_duration) override {
    auto* typed = dynamic_cast<CostmapSmoother*>(smoother.get());
    if (!typed) {
      return ResultT<void>::failure(
          ErrorCode::INTERNAL_ERROR,
          "CostmapSmootherExecution: module is not a CostmapSmoother");
    }
    if (!ctx) {
      return ResultT<void>::failure(
          ErrorCode::MAP_UNAVAILABLE,
          "CostmapSmootherExecution: no costmap module");
    }
    return typed->smoothPath(path, max_duration);
  }

 private:
  std::string name;
  CostmapMapContext::Ptr ctx;
};

}  // namespace costmap
}  // namespace ayanami
