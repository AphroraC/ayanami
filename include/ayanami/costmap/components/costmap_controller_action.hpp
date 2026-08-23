#pragma once

#include <memory>
#include <utility>

#include <costmap_server/costmap_wrapper.hpp>

#include "ayanami/abstract/components/controller_action.hpp"
#include "ayanami/costmap/components/costmap_controller_execution.hpp"

namespace ayanami {
namespace costmap {

/**
 * @brief 局部控制的 costmap 版 Action (spec §5.4)。
 *
 * 复用 ControllerAction 全部协议, 只覆写 createExecution 工厂:
 * 返回 CostmapControllerExecution (带 CostmapWrapper 激活/停用/持锁/
 * safetyCheck→isCurrent; 清零停止命令仍由 ControllerExecution 保证)。
 */
class CostmapControllerAction : public ayanami::components::ControllerAction {
 public:
  CostmapControllerAction(
      ayanami::abstract::AbstractController::Ptr controller,
      costmap_2d::CostmapWrapper::Ptr costmap_wrapper)
      : ControllerAction(std::move(controller)),
        costmap_wrapper_(std::move(costmap_wrapper)) {}

  ~CostmapControllerAction() override = default;

 protected:
  ayanami::components::ControllerExecution::Ptr createExecution() override {
    return std::make_shared<CostmapControllerExecution>("controller", controller_,
                                                        costmap_wrapper_);
  }

 private:
  costmap_2d::CostmapWrapper::Ptr costmap_wrapper_;
};

}  // namespace costmap
}  // namespace ayanami
