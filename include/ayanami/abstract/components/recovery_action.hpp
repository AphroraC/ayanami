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
    Base::start(*gh, std::move(exec), 0);
    return task;
  }

  bool cancelGoal(uint64_t goal_id) { return action_server->cancelGoal(goal_id); }

  void cancelAll() {
    action_server->cancelAll();
    Base::cancelAll();
  }

  ServerPtr server() const { return action_server; }

  /// @brief 注入 execution 工厂 (server 装配用: 经虚 new*Execution 钩子
  /// 返回带地图的 execution)。取代旧的 Server*Action 私有子类间接层。
  /// 未注入时 createExecution 默认构造 abstract execution。
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
    action_server->publishResult(gh.id, status, result);
  }

  std::function<RecoveryExecution::Ptr()> execution_factory;
  ServerPtr action_server;
};

} // namespace components
} // namespace ayanami
