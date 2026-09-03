#pragma once

#include <memory>
#include <string>
#include <utility>

#include <costmap_server/costmap_wrapper.hpp>

#include "ayanami/abstract/components/smoother_execution.hpp"
#include "ayanami/costmap/components/costmap_context.hpp"
#include "ayanami/costmap/core/costmap_smoother.hpp"

namespace ayanami {
namespace costmap {

/**
 * @brief 路径平滑的 costmap 版执行 (spec §5.4)。
 *
 * 在 SmootherExecution 之上绑定 costmap 上下文:
 * - preRun/postRun: checkActivate/checkDeactivate (global costmap);
 * - safetyCheck: global wrapper isCurrent();
 * - callSmoother: 经 abstract 签名直调插件; 插件内部按需加锁读图。
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
  FailureCode callSmoother(const ayanami::nav_msgs::Path& in,
                           ayanami::nav_msgs::Path& out, double max_duration,
                           bool check_for_collision,
                           std::string& message) override {
    auto* typed = dynamic_cast<CostmapSmoother*>(smoother.get());
    if (!typed) {
      message = "CostmapSmootherExecution: plugin is not a CostmapSmoother";
      return FailureCode::InternalError;
    }
    if (!ctx) {
      message = "CostmapSmootherExecution: no costmap module";
      return FailureCode::MapUnavailable;
    }
    return typed->smoothPath(in, out, max_duration, check_for_collision,
                             message);
  }

 private:
  std::string name;
  CostmapContext::Ptr ctx;
};

}  // namespace costmap
}  // namespace ayanami
