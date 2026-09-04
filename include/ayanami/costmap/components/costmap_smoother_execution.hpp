#pragma once

#include <memory>
#include <string>
#include <utility>

#include "ayanami/abstract/components/smoother_execution.hpp"
#include "ayanami/costmap/components/costmap_context.hpp"
#include "ayanami/costmap/core/costmap_smoother.hpp"

namespace ayanami {
namespace costmap {

/**
 * @brief 路径平滑的 costmap 版执行。
 *
 * 在 SmootherExecution 之上绑定 costmap 上下文:
 * - preRun/postRun: checkActivate/checkDeactivate (global costmap);
 * - safetyCheck: global wrapper isCurrent();
 * - callSmoother: typed in-place 调用 CostmapSmoother; 插件内部按需加锁读图。
 */
class CostmapSmootherExecution : public ayanami::components::SmootherExecution {
 public:
  using Ptr = std::shared_ptr<CostmapSmootherExecution>;

  CostmapSmootherExecution(const std::string& name,
                           ayanami::abstract::AbstractSmoother::Ptr smoother,
                           CostmapContext::Ptr ctx)
      : SmootherExecution(std::move(smoother)),
        name(name),
        ctx(std::move(ctx)) {}

  ~CostmapSmootherExecution() override = default;

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
  SmootherResultT callSmoother(aayanami::nav_msgs::Path& path,
                               double max_duration,
                               bool check_for_collision) override {
    auto* typed = dynamic_cast<CostmapSmoother*>(smoother.get());
    if (!typed) {
      return SmootherResultT::failure(
          ErrorCode::INTERNAL_ERROR,
          "CostmapSmootherExecution: plugin is not a CostmapSmoother");
    }
    if (!ctx) {
      return SmootherResultT::failure(
          ErrorCode::MAP_UNAVAILABLE,
          "CostmapSmootherExecution: no costmap module");
    }
    return typed->smoothPath(path, max_duration, check_for_collision);
  }

 private:
  std::string name;
  CostmapContext::Ptr ctx;
};

}  // namespace costmap
}  // namespace ayanami
