#pragma once

#include "ayanami/abstract/components/action_base.hpp"
#include "ayanami/abstract/components/action_server.hpp"
#include "ayanami/abstract/components/smoother_execution.hpp"
#include "ayanami/abstract/core/abstract_smoother.hpp"
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
 * @brief Path smoothing Action (modeled after MBF PlanRefinerAction).
 *
 * Mainline: acceptGoal returns a TaskHandle directly; the side path emits
 * on_smooth_path_result.
 * Mapping: SMOOTHED_PATH->SUCCEEDED, CANCELED->CANCELED, other
 * terminal->ABORTED.
 */
class SmootherAction : public ActionBase<ayanami::actions::SmoothPathGoal,
                                         SmootherExecution> {
public:
  using Base = ActionBase<ayanami::actions::SmoothPathGoal, SmootherExecution>;
  using Goal = ayanami::actions::SmoothPathGoal;
  using Result = ayanami::actions::SmoothPathResult;
  using Feedback = ayanami::actions::SmoothPathFeedback;
  using Server = ActionServer<Goal, Result, Feedback>;
  using ServerPtr = std::shared_ptr<Server>;
  using Task = Server::Task;

  explicit SmootherAction(ayanami::abstract::AbstractSmoother::Ptr smoother)
      : smoother(std::move(smoother)),
        action_server(std::make_shared<Server>()) {
    action_server->setGoalCallback([](const Server::GoalHandleType& gh) {
      ayanami::interfaces::NavigationCallbacks::on_smooth_path_goal(gh.goal);
    });
    action_server->setCancelCallback([this](uint64_t goal_id) {
      Base::cancelGoal(goal_id);
    });
    action_server->setFeedbackCallback([](const GoalFeedback<Feedback>& gf) {
      ayanami::interfaces::NavigationCallbacks::on_smooth_path_feedback(gf.feedback);
    });
    action_server->setResultCallback([](const Server::GoalResultType& gr) {
      ayanami::interfaces::NavigationCallbacks::on_smooth_path_result(gr);
    });
  }

  ~SmootherAction() override { cancelAll(); }

  Task acceptGoal(const Goal& goal) {
    auto task = action_server->acceptGoal(goal);
    auto gh = action_server->getGoal(task.goal_id());
    if (!gh) {
      return task;
    }
    if (!smoother) {
      Result result;
      result.error = ErrorCode::INTERNAL_ERROR;
      result.message = "Smoother plugin is null";
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

  /// @brief Settle a goal preempted from the pending slot (canceled before
  /// its driver started) so its TaskHandle never hangs.
  void finishPendingGoal(const GoalHandleType& gh) override {
    Result result;
    result.error = ErrorCode::CANCELED;
    result.message = "canceled before the smoother task started";
    result.session_id = gh.goal.session_id;
    result.plan_version = gh.goal.plan_version;
    action_server->publishResult(gh.id,
                                 ayanami::actions::GoalStatus::CANCELED,
                                 result);
  }

  /// @brief Driver task exception fallback: publish ABORTED to guarantee TaskHandle completion (idempotent).
  void abortGoal(const GoalHandleType& gh) override {
    Result result;
    result.error = ErrorCode::INTERNAL_ERROR;
    result.message = "smoother action driver exception";
    result.session_id = gh.goal.session_id;
    result.plan_version = gh.goal.plan_version;
    action_server->publishResult(gh.id,
                                 ayanami::actions::GoalStatus::ABORTED, result);
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
  void setExecutionFactory(std::function<SmootherExecution::Ptr()> factory) {
    execution_factory = std::move(factory);
  }

protected:
  void runImpl(const GoalHandleType& gh, SmootherExecution& exec) override {
    action_server->setActive(gh.id);
    exec.smoothPath(gh.goal.path, gh.goal.max_duration);
    runImplCommon(
        gh, exec,
        [this](const GoalHandleType& gh, SmootherExecution& exec) -> bool {
          if (action_server->isCancelRequested(gh.id)) {
            exec.cancel();
          }
          const bool canceled =
              action_server->isCancelRequested(gh.id) ||
              exec.getState() == SmootherExecution::CANCELED;
          if (canceled) {
            finishResult(gh, exec, ayanami::actions::GoalStatus::CANCELED);
            return true;
          }
          if (exec.getState() == SmootherExecution::SMOOTHED_PATH) {
            finishResult(gh, exec, ayanami::actions::GoalStatus::SUCCEEDED);
            return true;
          }
          switch (exec.getState()) {
            case SmootherExecution::NO_SMOOTHED_PATH:
            case SmootherExecution::STOPPED:
            case SmootherExecution::INTERNAL_ERROR:
              finishResult(gh, exec, ayanami::actions::GoalStatus::ABORTED);
              return true;
            default:
              break;
          }
          return false;
        });
  }

  virtual SmootherExecution::Ptr createExecution() {
    if (execution_factory) {
      return execution_factory();
    }
    return std::make_shared<SmootherExecution>(smoother);
  }

protected:
  ayanami::abstract::AbstractSmoother::Ptr smoother;

private:
  void finishResult(const GoalHandleType& gh, SmootherExecution& exec,
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
    result.smoothed_path = exec.takeSmoothedPath();
    result.session_id = gh.goal.session_id;
    result.plan_version = gh.goal.plan_version;
    action_server->publishResult(gh.id, status, result);
  }

  std::function<SmootherExecution::Ptr()> execution_factory;
  ServerPtr action_server;
};

} // namespace components
} // namespace ayanami
