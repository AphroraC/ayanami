#pragma once

#include <functional>
#include <memory>
#include <string>
#include <utility>

#include "ayanami/abstract/components/controller_action.hpp"
#include "ayanami/abstract/components/planner_action.hpp"
#include "ayanami/abstract/components/recovery_action.hpp"
#include "ayanami/abstract/components/smoother_action.hpp"
#include "ayanami/rosemap/components/rosemap_controller_execution.hpp"
#include "ayanami/rosemap/components/rosemap_planner_execution.hpp"
#include "ayanami/rosemap/components/rosemap_recovery_execution.hpp"
#include "ayanami/rosemap/components/rosemap_smoother_execution.hpp"
#include "ayanami/rosemap/core/rosemap_map_context.hpp"

namespace ayanami {
namespace rosemap {

/// @brief The rosemap variants of the four navigation Actions (production
/// path; the rosemap_navigation_server's createActions constructs these
/// directly, mirroring costmap_action).
///
/// Each reuses the entire abstract Action protocol (TaskHandle direct +
/// side-path emit + preemption) and only overrides the createExecution
/// factory: returns a Rosemap*Execution carrying the field context. The
/// execution configurator (server assembly: checkers / velocity limits /
/// tick cadence) is injected via configure; a no-op by default.

class RosemapPlannerAction : public ayanami::components::PlannerAction {
 public:
  using ExecutionConfigurator =
      std::function<void(ayanami::components::PlannerExecution::Ptr)>;

  RosemapPlannerAction(ayanami::abstract::AbstractPlanner::Ptr planner,
                       RosemapMapContext::Ptr ctx,
                       ExecutionConfigurator configure = nullptr)
      : PlannerAction(std::move(planner)),
        ctx(std::move(ctx)),
        configure(std::move(configure)) {}

  ~RosemapPlannerAction() override = default;

 protected:
  ayanami::components::PlannerExecution::Ptr createExecution() override {
    if (!ctx) {
      return PlannerAction::createExecution();
    }
    auto exec = std::make_shared<RosemapPlannerExecution>(
        "planner", planner, ctx);
    if (configure) {
      configure(exec);
    }
    return exec;
  }

 private:
  RosemapMapContext::Ptr ctx;
  ExecutionConfigurator configure;
};

class RosemapSmootherAction : public ayanami::components::SmootherAction {
 public:
  using ExecutionConfigurator =
      std::function<void(ayanami::components::SmootherExecution::Ptr)>;

  RosemapSmootherAction(ayanami::abstract::AbstractSmoother::Ptr smoother,
                        RosemapMapContext::Ptr ctx,
                        ExecutionConfigurator configure = nullptr)
      : SmootherAction(std::move(smoother)),
        ctx(std::move(ctx)),
        configure(std::move(configure)) {}

  ~RosemapSmootherAction() override = default;

 protected:
  ayanami::components::SmootherExecution::Ptr createExecution() override {
    if (!ctx) {
      return SmootherAction::createExecution();
    }
    auto exec = std::make_shared<RosemapSmootherExecution>(
        "smoother", smoother, ctx);
    if (configure) {
      configure(exec);
    }
    return exec;
  }

 private:
  RosemapMapContext::Ptr ctx;
  ExecutionConfigurator configure;
};

class RosemapControllerAction : public ayanami::components::ControllerAction {
 public:
  using ExecutionConfigurator =
      std::function<void(ayanami::components::ControllerExecution::Ptr)>;

  RosemapControllerAction(
      ayanami::abstract::AbstractController::Ptr controller,
      RosemapMapContext::Ptr ctx,
      ExecutionConfigurator configure = nullptr)
      : ControllerAction(std::move(controller)),
        ctx(std::move(ctx)),
        configure(std::move(configure)) {}

  ~RosemapControllerAction() override = default;

 protected:
  ayanami::components::ControllerExecution::Ptr createExecution() override {
    if (!ctx) {
      return ControllerAction::createExecution();
    }
    auto exec = std::make_shared<RosemapControllerExecution>(
        "controller", controller, ctx);
    if (configure) {
      configure(exec);
    }
    return exec;
  }

 private:
  RosemapMapContext::Ptr ctx;
  ExecutionConfigurator configure;
};

class RosemapRecoveryAction : public ayanami::components::RecoveryAction {
 public:
  using ExecutionConfigurator =
      std::function<void(ayanami::components::RecoveryExecution::Ptr)>;

  RosemapRecoveryAction(ayanami::abstract::AbstractRecovery::Ptr recovery,
                        RosemapMapContext::Ptr ctx,
                        ExecutionConfigurator configure = nullptr)
      : RecoveryAction(std::move(recovery)),
        ctx(std::move(ctx)),
        configure(std::move(configure)) {}

  ~RosemapRecoveryAction() override = default;

 protected:
  ayanami::components::RecoveryExecution::Ptr createExecution() override {
    if (!ctx) {
      return RecoveryAction::createExecution();
    }
    auto exec = std::make_shared<RosemapRecoveryExecution>(
        "recovery", recovery, ctx);
    if (configure) {
      configure(exec);
    }
    return exec;
  }

 private:
  RosemapMapContext::Ptr ctx;
  ExecutionConfigurator configure;
};

}  // namespace rosemap
}  // namespace ayanami
