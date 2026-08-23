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
 * @brief 局部控制 Action (对标 MBF ControllerAction, spec §5.1)。
 *
 * 主流: acceptGoal 返回 TaskHandle 直连; 旁路 emit on_exe_path_result。
 * cmd_vel 发布由 ControllerExecution 经 on_velocity_command 旁路完成。
 * 映射: ARRIVED_GOAL→SUCCEEDED, CANCELED→CANCELED, 其余 terminal→ABORTED。
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
      : controller_(std::move(controller)),
        server_(std::make_shared<Server>()) {
    server_->setGoalCallback([](const Server::GoalHandleType& gh) {
      ayanami::interfaces::NavigationCallbacks::on_exe_path_goal(gh.goal);
    });
    server_->setCancelCallback([this](uint64_t goal_id) {
      Base::cancelGoal(goal_id);
    });
    server_->setFeedbackCallback([](const GoalFeedback<Feedback>& gf) {
      ayanami::interfaces::NavigationCallbacks::on_exe_path_feedback(gf.feedback);
    });
    server_->setResultCallback([](const Server::GoalResultType& gr) {
      ayanami::interfaces::NavigationCallbacks::on_exe_path_result(gr);
    });
  }

  ~ControllerAction() override { cancelAll(); }

  Task acceptGoal(const Goal& goal) {
    auto task = server_->acceptGoal(goal);
    auto gh = server_->getGoal(task.goal_id());
    if (!gh) {
      return task;
    }
    if (!controller_) {
      Result result;
      result.failure = FailureCode::InternalError;
      result.message = "Controller plugin is null";
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
  void runImpl(const GoalHandleType& gh, ControllerExecution& exec) override {
    server_->setActive(gh.id);
    exec.setNewPlan(gh.goal.path);
    exec.start();
    runImplCommon(
        gh, exec,
        [this](const GoalHandleType& gh, ControllerExecution& exec) -> bool {
          if (server_->isCancelRequested(gh.id)) {
            exec.cancel();
          }
          const bool canceled =
              server_->isCancelRequested(gh.id) ||
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
            case ControllerExecution::MAX_RETRIES:
            case ControllerExecution::PAT_EXCEEDED:
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
    return std::make_shared<ControllerExecution>(controller_);
  }

protected:
  ayanami::abstract::AbstractController::Ptr controller_;

private:
  void finishResult(const GoalHandleType& gh, const ControllerExecution& exec,
                    ayanami::actions::GoalStatus status) {
    Result result;
    result.failure = exec.getFailure();
    if (status == ayanami::actions::GoalStatus::CANCELED) {
      result.failure = FailureCode::Canceled;
    } else if (status == ayanami::actions::GoalStatus::SUCCEEDED) {
      result.failure = FailureCode::None;
    } else if (result.failure == FailureCode::None ||
               result.failure == FailureCode::Canceled) {
      result.failure = FailureCode::InternalError;
    }
    result.message = exec.getMessage();
    result.reached = exec.getState() == ControllerExecution::ARRIVED_GOAL;
    result.session_id = gh.goal.session_id;
    result.plan_version = gh.goal.plan_version;
    server_->publishResult(gh.id, status, result);
  }

  ServerPtr server_;
};

} // namespace components
} // namespace ayanami
