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
 * @brief 路径平滑 Action (对标 MBF PlanRefinerAction, spec §5.1)。
 *
 * 主流: acceptGoal 返回 TaskHandle 直连; 旁路 emit on_smooth_path_result。
 * 映射: SMOOTHED_PATH→SUCCEEDED, CANCELED→CANCELED, 其余 terminal→ABORTED。
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
      : smoother_(std::move(smoother)),
        server_(std::make_shared<Server>()) {
    server_->setGoalCallback([](const Server::GoalHandleType& gh) {
      ayanami::interfaces::NavigationCallbacks::on_smooth_path_goal(gh.goal);
    });
    server_->setCancelCallback([this](uint64_t goal_id) {
      Base::cancelGoal(goal_id);
    });
    server_->setFeedbackCallback([](const GoalFeedback<Feedback>& gf) {
      ayanami::interfaces::NavigationCallbacks::on_smooth_path_feedback(gf.feedback);
    });
    server_->setResultCallback([](const Server::GoalResultType& gr) {
      ayanami::interfaces::NavigationCallbacks::on_smooth_path_result(gr);
    });
  }

  ~SmootherAction() override { cancelAll(); }

  Task acceptGoal(const Goal& goal) {
    auto task = server_->acceptGoal(goal);
    auto gh = server_->getGoal(task.goal_id());
    if (!gh) {
      return task;
    }
    if (!smoother_) {
      Result result;
      result.failure = FailureCode::InternalError;
      result.message = "Smoother plugin is null";
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
  void runImpl(const GoalHandleType& gh, SmootherExecution& exec) override {
    server_->setActive(gh.id);
    exec.smoothPath(gh.goal.path, gh.goal.max_duration,
                    gh.goal.check_for_collision);
    runImplCommon(
        gh, exec,
        [this](const GoalHandleType& gh, SmootherExecution& exec) -> bool {
          if (server_->isCancelRequested(gh.id)) {
            exec.cancel();
          }
          const bool canceled =
              server_->isCancelRequested(gh.id) ||
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
            case SmootherExecution::MAX_RETRIES:
            case SmootherExecution::PAT_EXCEEDED:
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
    return std::make_shared<SmootherExecution>(smoother_);
  }

protected:
  ayanami::abstract::AbstractSmoother::Ptr smoother_;

private:
  void finishResult(const GoalHandleType& gh, const SmootherExecution& exec,
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
    result.smoothed_path = exec.getSmoothedPath();
    result.session_id = gh.goal.session_id;
    result.plan_version = gh.goal.plan_version;
    server_->publishResult(gh.id, status, result);
  }

  ServerPtr server_;
};

} // namespace components
} // namespace ayanami
