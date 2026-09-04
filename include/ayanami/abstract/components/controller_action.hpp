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
    Base::start(*gh, createExecution(), 0);
    return task;
  }

  bool cancelGoal(uint64_t goal_id) { return action_server->cancelGoal(goal_id); }

  /// @brief 运行中 in-place 更新目标路径 + 规划版本 (spec in-place plan update)。
  /// 不 cancel / 不 join / 不新建 Execution; 线程安全 (execution plan_mutex)。
  /// 返回 false 表示 goal 已不在跑 (任务已完成/取消) 或 execution 已到终态
  /// (该次更新不会被消费)。调用方应重查已完成任务并消费结果, 而非误报失败。
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

  /// @brief 注入 execution 工厂 (server 装配用: 经虚 new*Execution 钩子
  /// 返回带地图的 execution)。取代旧的 Server*Action 私有子类间接层。
  /// 未注入时 createExecution 默认构造 abstract execution。
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
    // 最终结果只携带真正被 controller->setPlan 消费的已应用版本 (applied),
    // 不携带仅在途/未消费的 pending 版本: in-place updatePlan 提交的版本只有
    // 在执行循环消费并 setPlan 成功后才会反映到 exec.getPlanVersion()。
    result.plan_version = exec.getPlanVersion();
    action_server->publishResult(gh.id, status, result);
  }

  std::function<ControllerExecution::Ptr()> execution_factory;
  ServerPtr action_server;
};

} // namespace components
} // namespace ayanami
