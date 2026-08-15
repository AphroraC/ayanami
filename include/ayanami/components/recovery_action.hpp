#pragma once

#include "ayanami/components/action_base.hpp"
#include "ayanami/components/action_server.hpp"
#include "ayanami/components/recovery_execution.hpp"
#include "ayanami/core/abstract_recovery.hpp"
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
 * @brief 恢复行为 Action (对标 MBF RecoveryAction, spec §5.1)。
 *
 * 主流: acceptGoal 返回 TaskHandle 直连; 旁路 emit on_recovery_result。
 * 映射: RECOVERY_DONE→SUCCEEDED, CANCELED→CANCELED, 其余 terminal→ABORTED。
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
      : recovery_(std::move(recovery)),
        server_(std::make_shared<Server>()) {
    server_->setGoalCallback([](const Server::GoalHandleType& gh) {
      ayanami::interfaces::NavigationCallbacks::on_recovery_goal(gh.goal);
    });
    server_->setCancelCallback([this](uint64_t goal_id) {
      Base::cancelGoal(goal_id);
    });
    server_->setFeedbackCallback([](const GoalFeedback<Feedback>& gf) {
      ayanami::interfaces::NavigationCallbacks::on_recovery_feedback(gf.feedback);
    });
    server_->setResultCallback([](const Server::GoalResultType& gr) {
      ayanami::interfaces::NavigationCallbacks::on_recovery_result(gr);
    });
  }

  ~RecoveryAction() override { cancelAll(); }

  Task acceptGoal(const Goal& goal) {
    auto task = server_->acceptGoal(goal);
    auto gh = server_->getGoal(task.goal_id());
    if (!gh) {
      return task;
    }
    if (!recovery_) {
      Result result;
      result.outcome = Result::INTERNAL_ERROR;
      result.message = "Recovery plugin is null";
      result.session_id = goal.session_id;
      result.plan_version = goal.plan_version;
      server_->publishResult(gh->id, ayanami::actions::GoalStatus::ABORTED,
                             result);
      return task;
    }
    Base::start(*gh, createExecution(), 0);
    return task;
  }

  bool cancelGoal(uint64_t goal_id) { return server_->cancelGoal(goal_id); }

  void cancelAll() {
    server_->cancelAll();
    Base::cancelAll();
  }

  ServerPtr server() const { return server_; }

protected:
  void runImpl(const GoalHandleType& gh, RecoveryExecution& exec) override {
    server_->setActive(gh.id);
    exec.start();
    runImplCommon(
        gh, exec,
        [this](const GoalHandleType& gh, RecoveryExecution& exec) -> bool {
          if (server_->isCancelRequested(gh.id)) {
            exec.cancel();
          }
          const bool canceled =
              server_->isCancelRequested(gh.id) ||
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
    return std::make_shared<RecoveryExecution>(recovery_);
  }

protected:
  ayanami::abstract::AbstractRecovery::Ptr recovery_;

private:
  void finishResult(const GoalHandleType& gh, const RecoveryExecution& exec,
                    ayanami::actions::GoalStatus status) {
    Result result;
    result.outcome = exec.getOutcome();
    result.message = exec.getMessage();
    result.session_id = gh.goal.session_id;
    result.plan_version = gh.goal.plan_version;
    server_->publishResult(gh.id, status, result);
  }

  ServerPtr server_;
};

} // namespace components
} // namespace ayanami
