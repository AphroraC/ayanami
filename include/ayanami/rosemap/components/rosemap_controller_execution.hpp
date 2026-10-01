#pragma once

#include <memory>
#include <string>
#include <utility>

#include "ayanami/abstract/components/controller_execution.hpp"
#include "ayanami/rosemap/core/rosemap_controller.hpp"
#include "ayanami/rosemap/core/rosemap_map_context.hpp"

namespace ayanami {
namespace rosemap {

/// @brief The rosemap variant of the local control execution.
///
/// Binds a rosemap field context on top of ControllerExecution:
/// - safetyCheck: field freshness (snapshot exists + optional stale gate) --
///   an MPC controller that never reads the field still gates on it, so a
///   dead map pipeline surfaces as MAP_STALE -> BT recovery instead of
///   blind tracking;
/// - callController: per-cycle typed call into RosemapController.
class RosemapControllerExecution
    : public ayanami::components::ControllerExecution {
 public:
  using Ptr = std::shared_ptr<RosemapControllerExecution>;

  RosemapControllerExecution(
      const std::string& name,
      ayanami::abstract::AbstractController::Ptr controller,
      RosemapMapContext::Ptr ctx)
      : ControllerExecution(std::move(controller)),
        name(name),
        ctx(std::move(ctx)) {}

  ~RosemapControllerExecution() override = default;

  bool safetyCheck() override { return !ctx || ctx->current(); }

 protected:
  ResultT<void> callController(
      const ayanami::geometry_msgs::PoseStamped& pose,
      const ayanami::geometry_msgs::TwistStamped& velocity,
      ayanami::geometry_msgs::TwistStamped& cmd) override {
    auto* typed = dynamic_cast<RosemapController*>(controller.get());
    if (!typed) {
      return ResultT<void>::failure(
          ErrorCode::INTERNAL_ERROR,
          "RosemapControllerExecution: module is not a RosemapController");
    }
    if (!ctx) {
      return ResultT<void>::failure(
          ErrorCode::MAP_UNAVAILABLE,
          "RosemapControllerExecution: no rosemap field");
    }
    return typed->computeVelocityCommand(pose, velocity, cmd);
  }

 private:
  std::string name;
  RosemapMapContext::Ptr ctx;
};

}  // namespace rosemap
}  // namespace ayanami
