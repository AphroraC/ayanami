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
    Base::start(*gh, createExecution(), 0);
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
  void setExecutionFactory(std::function<SmootherExecution::Ptr()> factory) {
    execution_factory = std::move(factory);
  }

protected:
  void runImpl(const GoalHandleType& gh, SmootherExecution& exec) override {
    action_server->setActive(gh.id);
    exec.smoothPath(gh.goal.path, gh.goal.max_duration,
                    gh.goal.check_for_collision);
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
  void finishResult(const GoalHandleType& gh, const SmootherExecution& exec,
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
    result.smoothed_path = exec.getSmoothedPath();
    result.session_id = gh.goal.session_id;
    result.plan_version = gh.goal.plan_version;
    action_server->publishResult(gh.id, status, result);
  }

  std::function<SmootherExecution::Ptr()> execution_factory;
  ServerPtr action_server;
};

} // namespace components
} // namespace ayanami
