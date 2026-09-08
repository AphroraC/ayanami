#pragma once

#include "ayanami/abstract/components/action_base.hpp"
#include "ayanami/abstract/components/action_server.hpp"
#include "ayanami/abstract/components/controller_execution.hpp"
#include "ayanami/abstract/core/abstract_controller.hpp"
#include "ayanami/interfaces/callbacks.hpp"
#include "ayanami/messages/actions.hpp"

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>

namespace ayanami {
namespace components {

/**
 * @brief Local control Action (modeled after MBF ControllerAction).
 *
 * Mainline: acceptGoal returns a TaskHandle directly; the side path emits
 * on_exe_path_result. cmd_vel publishing is done by ControllerExecution via
 * the on_velocity_command side path.
 * Mapping: ARRIVED_GOAL->SUCCEEDED, CANCELED->CANCELED, other
 * terminal->ABORTED.
 */
class ControllerAction : public ActionBase<ayanami::actions::ExePathGoal,
                                           ControllerExecution> {
public:
  using Base = ActionBase<ayanami::actions::ExePathGoal, ControllerExecution>;
  using Goal = ayanami::actions::ExePathGoal;
  using Result = ayanami::actions::ExePathResult;
  using Feedback = ayanami::actions::ExePathFeedback;
  using Server = ActionServer<Goal, Result, Feedback>;
  using ServerPtr = std::shared_ptr<Server>;
  using Task = Server::Task;

  explicit ControllerAction(ayanami::abstract::AbstractController::Ptr controller)
      : controller(std::move(controller)),
        action_server(std::make_shared<Server>()) {
    action_server->setGoalCallback([](const Server::GoalHandleType& gh) {
      ayanami::interfaces::NavigationCallbacks::on_exe_path_goal(gh.goal);
    });
    action_server->setCancelCallback([this](uint64_t goal_id) {
      Base::cancelGoal(goal_id);
    });
    action_server->setFeedbackCallback([](const GoalFeedback<Feedback>& gf) {
      ayanami::interfaces::NavigationCallbacks::on_exe_path_feedback(gf.feedback);
    });
    action_server->setResultCallback([](const Server::GoalResultType& gr) {
      ayanami::interfaces::NavigationCallbacks::on_exe_path_result(gr);
    });
  }

  ~ControllerAction() override { cancelAll(); }

  Task acceptGoal(const Goal& goal) {
    auto task = action_server->acceptGoal(goal);
    auto gh = action_server->getGoal(task.goal_id());
    if (!gh) {
      return task;
    }
    if (!controller) {
      Result result;
      result.error = ErrorCode::INTERNAL_ERROR;
      result.message = "Controller plugin is null";
      result.session_id = goal.session_id;
      result.plan_version = goal.plan_version;
      action_server->publishResult(gh->id, ayanami::actions::GoalStatus::ABORTED,
                             result);
      return task;
    }
    Base::start(*gh, createExecution());
    return task;
  }

  bool cancelGoal(uint64_t goal_id) { return action_server->cancelGoal(goal_id); }

  /// @brief Driver task exception fallback: publish ABORTED to guarantee TaskHandle completion (idempotent).
  void abortGoal(const GoalHandleType& gh) override {
    Result result;
    result.error = ErrorCode::INTERNAL_ERROR;
    result.message = "controller action driver exception";
    result.session_id = gh.goal.session_id;
    result.plan_version = gh.goal.plan_version;
    action_server->publishResult(gh.id,
                                 ayanami::actions::GoalStatus::ABORTED, result);
  }

  /// @brief In-place update of the goal path + plan version while running
  /// (spec in-place plan update).
  /// No cancel / no join / no new Execution; thread-safe (execution
  /// plan_mutex).
  /// Returns false when the goal is no longer running (task already
  /// finished/canceled) or the execution has reached a terminal state (that
  /// update will not be consumed). The caller should re-check the finished
  /// task and consume its result instead of falsely reporting failure.
  bool updatePlan(uint64_t goal_id, const nav_msgs::Path& path,
                  uint64_t plan_version) {
    auto exec = Base::executionForGoal(goal_id);
    if (!exec) {
      return false;
    }
    return exec->updatePlan(path, plan_version);
  }

  void cancelAll() {
    action_server->cancelAll();
    Base::cancelAll();
  }

  ServerPtr server() const { return action_server; }

  /// @brief Inject an execution factory (for server assembly: the virtual
  /// new*Execution hooks return executions carrying a map). Replaces the
  /// old Server*Action private-subclass indirection layer.
  /// When not injected, createExecution default-constructs the abstract
  /// execution.
  void setExecutionFactory(std::function<ControllerExecution::Ptr()> factory) {
    execution_factory = std::move(factory);
  }

protected:
  void runImpl(const GoalHandleType& gh, ControllerExecution& exec) override {
    action_server->setActive(gh.id);
    exec.setNewPlan(gh.goal.path);
    exec.setPlanVersion(gh.goal.plan_version);
    exec.start();
    runImplCommon(
        gh, exec,
        [this](const GoalHandleType& gh, ControllerExecution& exec) -> bool {
          if (action_server->isCancelRequested(gh.id)) {
            exec.cancel();
          }
          const bool canceled =
              action_server->isCancelRequested(gh.id) ||
              exec.getState() == ControllerExecution::CANCELED;
          if (canceled) {
            finishResult(gh, exec, ayanami::actions::GoalStatus::CANCELED);
            return true;
          }
          if (exec.getState() == ControllerExecution::ARRIVED_GOAL) {
            finishResult(gh, exec, ayanami::actions::GoalStatus::SUCCEEDED);
            return true;
          }
          switch (exec.getState()) {
            case ControllerExecution::NO_PLAN:
            case ControllerExecution::EMPTY_PLAN:
            case ControllerExecution::INVALID_PLAN:
            case ControllerExecution::NO_PROGRESS:
            case ControllerExecution::STOPPED:
            case ControllerExecution::INTERNAL_ERROR:
            case ControllerExecution::ROBOT_DISABLED:
              finishResult(gh, exec, ayanami::actions::GoalStatus::ABORTED);
              return true;
            default:
              break;
          }
          return false;
        });
  }

  virtual ControllerExecution::Ptr createExecution() {
    if (execution_factory) {
      return execution_factory();
    }
    return std::make_shared<ControllerExecution>(controller);
  }

protected:
  ayanami::abstract::AbstractController::Ptr controller;

private:
  void finishResult(const GoalHandleType& gh, const ControllerExecution& exec,
                    ayanami::actions::GoalStatus status) {
    Result result;
    result.error = exec.getError();
    if (status == ayanami::actions::GoalStatus::CANCELED) {
      result.error = ErrorCode::CANCELED;
    } else if (status == ayanami::actions::GoalStatus::SUCCEEDED) {
      result.error = ErrorCode::NONE;
    } else if (result.error == ErrorCode::NONE ||
               result.error == ErrorCode::CANCELED) {
      result.error = ErrorCode::INTERNAL_ERROR;
    }
    result.message = exec.getMessage();
    result.reached = exec.getState() == ControllerExecution::ARRIVED_GOAL;
    result.session_id = gh.goal.session_id;
    // The final result carries only the applied version actually consumed
    // by controller->setPlan, not a merely in-flight/unconsumed pending
    // version: a version submitted by in-place updatePlan is reflected in
    // exec.getPlanVersion only after the execution loop consumes it and
    // setPlan succeeds.
    result.plan_version = exec.getPlanVersion();
    action_server->publishResult(gh.id, status, result);
  }

  std::function<ControllerExecution::Ptr()> execution_factory;
  ServerPtr action_server;
};

} // namespace components
} // namespace ayanami
