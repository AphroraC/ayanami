#pragma once

#include "ayanami/abstract/components/action_base.hpp"
#include "ayanami/abstract/components/action_server.hpp"
#include "ayanami/abstract/components/recovery_execution.hpp"
#include "ayanami/abstract/core/abstract_recovery.hpp"
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
 * @brief Recovery behavior Action (modeled after MBF RecoveryAction).
 *
 * Mainline: acceptGoal returns a TaskHandle directly; the side path emits
 * on_recovery_result.
 * Mapping: RECOVERY_DONE->SUCCEEDED, CANCELED->CANCELED, other
 * terminal->ABORTED.
 */
class RecoveryAction : public ActionBase<ayanami::actions::RecoveryGoal,
                                         RecoveryExecution> {
public:
  using Base = ActionBase<ayanami::actions::RecoveryGoal, RecoveryExecution>;
  using Goal = ayanami::actions::RecoveryGoal;
  using Result = ayanami::actions::RecoveryResult;
  using Feedback = ayanami::actions::RecoveryFeedback;
  using Server = ActionServer<Goal, Result, Feedback>;
  using ServerPtr = std::shared_ptr<Server>;
  using Task = Server::Task;

  explicit RecoveryAction(ayanami::abstract::AbstractRecovery::Ptr recovery)
      : recovery(std::move(recovery)),
        action_server(std::make_shared<Server>()) {
    action_server->setGoalCallback([](const Server::GoalHandleType& gh) {
      ayanami::interfaces::NavigationCallbacks::on_recovery_goal(gh.goal);
    });
    action_server->setCancelCallback([this](uint64_t goal_id) {
      Base::cancelGoal(goal_id);
    });
    action_server->setFeedbackCallback([](const GoalFeedback<Feedback>& gf) {
      ayanami::interfaces::NavigationCallbacks::on_recovery_feedback(gf.feedback);
    });
    action_server->setResultCallback([](const Server::GoalResultType& gr) {
      ayanami::interfaces::NavigationCallbacks::on_recovery_result(gr);
    });
  }

  ~RecoveryAction() override { cancelAll(); }

  Task acceptGoal(const Goal& goal) {
    auto task = action_server->acceptGoal(goal);
    auto gh = action_server->getGoal(task.goal_id());
    if (!gh) {
      return task;
    }
    if (!recovery) {
      Result result;
      result.error = ErrorCode::INTERNAL_ERROR;
      result.message = "Recovery plugin is null";
      result.session_id = goal.session_id;
      result.plan_version = goal.plan_version;
      action_server->publishResult(gh->id, ayanami::actions::GoalStatus::ABORTED,
                             result);
      return task;
    }
    auto exec = createExecution();
    exec->setBehaviorParams(goal.behavior);
    Base::start(*gh, std::move(exec));
    return task;
  }

  bool cancelGoal(uint64_t goal_id) { return action_server->cancelGoal(goal_id); }

  /// @brief Settle a goal preempted from the pending slot (canceled before
  /// its driver started) so its TaskHandle never hangs. The recovery
  /// resource mutex is released via the completion callback.
  void finishPendingGoal(const GoalHandleType& gh) override {
    Result result;
    result.error = ErrorCode::CANCELED;
    result.message = "canceled before the recovery task started";
    result.session_id = gh.goal.session_id;
    result.plan_version = gh.goal.plan_version;
    action_server->publishResult(gh.id,
                                 ayanami::actions::GoalStatus::CANCELED,
                                 result);
  }

  /// @brief Driver task exception fallback: publish ABORTED to guarantee
  /// TaskHandle completion (idempotent).
  /// The result completion callback triggers requestRecovery's mutex
  /// release; the exception path must not leave the resource protocol
  /// hanging.
  void abortGoal(const GoalHandleType& gh) override {
    Result result;
    result.error = ErrorCode::INTERNAL_ERROR;
    result.message = "recovery action driver exception";
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
  void setExecutionFactory(std::function<RecoveryExecution::Ptr()> factory) {
    execution_factory = std::move(factory);
  }

protected:
  void runImpl(const GoalHandleType& gh, RecoveryExecution& exec) override {
    action_server->setActive(gh.id);
    exec.start();
    runImplCommon(
        gh, exec,
        [this](const GoalHandleType& gh, RecoveryExecution& exec) -> bool {
          if (action_server->isCancelRequested(gh.id)) {
            exec.cancel();
          }
          const bool canceled =
              action_server->isCancelRequested(gh.id) ||
              exec.getState() == RecoveryExecution::CANCELED;
          if (canceled) {
            finishResult(gh, exec, ayanami::actions::GoalStatus::CANCELED);
            return true;
          }
          if (exec.getState() == RecoveryExecution::RECOVERY_DONE) {
            finishResult(gh, exec, ayanami::actions::GoalStatus::SUCCEEDED);
            return true;
          }
          switch (exec.getState()) {
            case RecoveryExecution::RECOVERY_FAILED:
            case RecoveryExecution::PAT_EXCEEDED:
            case RecoveryExecution::STOPPED:
            case RecoveryExecution::INTERNAL_ERROR:
              finishResult(gh, exec, ayanami::actions::GoalStatus::ABORTED);
              return true;
            default:
              break;
          }
          return false;
        });
  }

  virtual RecoveryExecution::Ptr createExecution() {
    if (execution_factory) {
      return execution_factory();
    }
    return std::make_shared<RecoveryExecution>(recovery);
  }

protected:
  ayanami::abstract::AbstractRecovery::Ptr recovery;

private:
  void finishResult(const GoalHandleType& gh, const RecoveryExecution& exec,
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
    result.session_id = gh.goal.session_id;
    result.plan_version = gh.goal.plan_version;
    Feedback feedback;
    feedback.message = "goal finished";
    action_server->publishFeedback(gh.id, feedback);
    action_server->publishResult(gh.id, status, result);
  }

  std::function<RecoveryExecution::Ptr()> execution_factory;
  ServerPtr action_server;
};

} // namespace components
} // namespace ayanami
