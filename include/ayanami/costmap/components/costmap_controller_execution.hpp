#pragma once

#include <memory>
#include <string>
#include <utility>

#include <costmap_server/costmap_wrapper.hpp>

#include "ayanami/abstract/components/controller_execution.hpp"
#include "ayanami/costmap/components/costmap_context.hpp"
#include "ayanami/costmap/core/costmap_controller.hpp"

namespace ayanami {
namespace costmap {

/**
 * @brief 局部控制的 costmap 版执行 (spec §5.4, 对标 MBF ControllerCostmapExecution)。
 *
 * 在 ControllerExecution 之上绑定 costmap 上下文:
 * - preRun: checkActivate() 幂等激活 local costmap;
 * - postRun: checkDeactivate() 幂等停用 local costmap;
 * - safetyCheck: local wrapper isCurrent(); 非 current 时基类循环发布清零
 *   停止命令 (不发速度);
 * - callController: 每 cycle 经 abstract 签名直调插件; 插件内部加锁读
 *   local master grid (shared 锁, 周期结束释放)。
 */
class CostmapControllerExecution : public ayanami::components::ControllerExecution {
 public:
  using Ptr = std::shared_ptr<CostmapControllerExecution>;

  CostmapControllerExecution(const std::string& name,
                             ayanami::abstract::AbstractController::Ptr controller,
                             CostmapContext::Ptr ctx)
      : ControllerExecution(std::move(controller)),
        name(name),
        ctx(std::move(ctx)) {}

  ~CostmapControllerExecution() override = default;

  void preRun() override {
    if (ctx && ctx->localCostmap()) {
      ctx->localCostmap()->checkActivate();
    }
  }

  void postRun() override {
    if (ctx && ctx->localCostmap()) {
      ctx->localCostmap()->checkDeactivate();
    }
  }

  bool safetyCheck() override {
    return !ctx || !ctx->localCostmap() || ctx->localCostmap()->isCurrent();
  }

 protected:
  FailureCode callController(const ayanami::geometry_msgs::PoseStamped& pose,
                             const ayanami::geometry_msgs::TwistStamped& velocity,
                             ayanami::geometry_msgs::TwistStamped& cmd_vel,
                             std::string& message) override {
    auto* typed = dynamic_cast<CostmapController*>(controller.get());
    if (!typed) {
      message = "CostmapControllerExecution: plugin is not a CostmapController";
      return FailureCode::InternalError;
    }
    if (!ctx) {
      message = "CostmapControllerExecution: no costmap module";
      return FailureCode::MapUnavailable;
    }
    return typed->computeVelocityCommands(pose, velocity, cmd_vel, message);
  }

 private:
  std::string name;
  CostmapContext::Ptr ctx;
};

}  // namespace costmap
}  // namespace ayanami
