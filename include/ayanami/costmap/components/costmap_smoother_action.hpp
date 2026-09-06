#pragma once

#include <functional>
#include <memory>
#include <string>
#include <utility>

#include "ayanami/abstract/components/smoother_action.hpp"
#include "ayanami/costmap/components/costmap_map_context.hpp"
#include "ayanami/costmap/components/costmap_smoother_execution.hpp"

namespace ayanami {
namespace costmap {

/**
 * @brief 路径平滑的 costmap 版 Action ( 生产路径)。
 *
 * 复用 SmootherAction 全部协议 (TaskHandle 直连 + 旁路 emit), 只覆写
 * createExecution: 返回 CostmapSmootherExecution (带 CostmapMapContext 的
 * 激活/停用 global + safetyCheck→isCurrent)。执行配置器 (server 装配:
 * 速度上限等) 经 configure 注入, 默认无操作。
 */
class CostmapSmootherAction : public ayanami::components::SmootherAction {
 public:
  using ExecutionConfigurator =
      std::function<void(ayanami::components::SmootherExecution::Ptr)>;

  CostmapSmootherAction(ayanami::abstract::AbstractSmoother::Ptr smoother,
                        CostmapMapContext::Ptr ctx,
                        ExecutionConfigurator configure = nullptr)
      : SmootherAction(std::move(smoother)),
        ctx(std::move(ctx)),
        configure(std::move(configure)) {}

  ~CostmapSmootherAction() override = default;

 protected:
  ayanami::components::SmootherExecution::Ptr createExecution() override {
    if (!ctx) {
      return SmootherAction::createExecution();
    }
    auto exec = std::make_shared<CostmapSmootherExecution>(
        "smoother", smoother, ctx);
    if (configure) {
      configure(exec);
    }
    return exec;
  }

 private:
  CostmapMapContext::Ptr ctx;
  ExecutionConfigurator configure;
};

}  // namespace costmap
}  // namespace ayanami