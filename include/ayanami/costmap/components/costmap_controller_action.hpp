#pragma once

#include <functional>
#include <memory>
#include <string>
#include <utility>

#include "ayanami/abstract/components/controller_action.hpp"
#include "ayanami/costmap/components/costmap_map_context.hpp"
#include "ayanami/costmap/components/costmap_controller_execution.hpp"

namespace ayanami {
namespace costmap {

/**
 * @brief 局部控制的 costmap 版 Action ( 生产路径)。
 *
 * 复用 ControllerAction 全部协议, 只覆写 createExecution 工厂:
 * 返回 CostmapControllerExecution (带 CostmapMapContext: 激活/停用 local +
 * 每 cycle 加锁读图 + safetyCheck→isCurrent; 清零停止命令仍由
 * ControllerExecution 保证)。执行配置器 (server 装配: goal/progress
 * checker + 速度上限 + 机器人状态源) 经 configure 注入。
 */
class CostmapControllerAction : public ayanami::components::ControllerAction {
 public:
  using ExecutionConfigurator =
      std::function<void(ayanami::components::ControllerExecution::Ptr)>;

  CostmapControllerAction(
      ayanami::abstract::AbstractController::Ptr controller,
      CostmapMapContext::Ptr ctx, ExecutionConfigurator configure = nullptr)
      : ControllerAction(std::move(controller)),
        ctx(std::move(ctx)),
        configure(std::move(configure)) {}

  ~CostmapControllerAction() override = default;

 protected:
  ayanami::components::ControllerExecution::Ptr createExecution() override {
    if (!ctx) {
      return ControllerAction::createExecution();
    }
    auto exec = std::make_shared<CostmapControllerExecution>(
        "controller", controller, ctx);
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
