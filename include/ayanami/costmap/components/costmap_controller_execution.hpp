#pragma once

#include <memory>
#include <string>
#include <utility>

#include "ayanami/abstract/components/controller_execution.hpp"
#include "ayanami/costmap/components/costmap_map_context.hpp"
#include "ayanami/costmap/core/costmap_controller.hpp"

namespace ayanami {
namespace costmap {

/**
 * @brief The costmap variant of the local control execution.
 *
 * Binds a costmap context on top of ControllerExecution:
 * - preRun: checkActivate() idempotently activates the local costmap;
 * - postRun: checkDeactivate() idempotently deactivates the local costmap;
 * - safetyCheck: local wrapper isCurrent(); when not current the base loop
 *   publishes zeroed stop commands (no velocity);
 * - callController: per-cycle typed call into CostmapController; the module
 *   reads the local master grid under a lock internally (shared lock,
 *   released at cycle end).
 */
class CostmapControllerExecution : public ayanami::components::ControllerExecution {
 public:
  using Ptr = std::shared_ptr<CostmapControllerExecution>;

  CostmapControllerExecution(const std::string& name,
                             ayanami::abstract::AbstractController::Ptr controller,
                             CostmapMapContext::Ptr ctx)
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
  ResultT<void> callController(
      const ayanami::geometry_msgs::PoseStamped& pose,
      const ayanami::geometry_msgs::TwistStamped& velocity,
      ayanami::geometry_msgs::TwistStamped& cmd) override {
    auto* typed = dynamic_cast<CostmapController*>(controller.get());
    if (!typed) {
      return ResultT<void>::failure(
          ErrorCode::INTERNAL_ERROR,
          "CostmapControllerExecution: module is not a CostmapController");
    }
    if (!ctx) {
      return ResultT<void>::failure(
          ErrorCode::MAP_UNAVAILABLE,
          "CostmapControllerExecution: no costmap module");
    }
    return typed->computeVelocityCommand(pose, velocity, cmd);
  }

 private:
  std::string name;
  CostmapMapContext::Ptr ctx;
};

}  // namespace costmap
}  // namespace ayanami
