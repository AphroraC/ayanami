#pragma once

#include "ayanami/abstract/components/action_base.hpp"
#include "ayanami/abstract/components/action_server.hpp"
#include "ayanami/abstract/components/planner_execution.hpp"
#include "ayanami/abstract/core/abstract_planner.hpp"
#include "ayanami/interfaces/callbacks.hpp"
#include "ayanami/messages/actions.hpp"

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

namespace ayanami {
namespace components {

/**
 * @brief 全局路径规划 Action (对标 MBF PlannerAction)。
 *
 * acceptGoal 返回 TaskHandle 直连调用者 (主流, BT 节点); 结果同时 emit 到
 * NavigationCallbacks::on_get_path_result 作旁路 (rviz 扩展/回退/监控)。
 * ExecutionState→GoalStatus 映射 (Ruling #2): FOUND_PLAN→SUCCEEDED,
 * CANCELED→CANCELED, 其余 terminal (NO_PLAN_FOUND/STOPPED/INTERNAL_ERROR)
 * →ABORTED + ErrorCode。
 */
class PlannerAction : public ActionBase<ayanami::actions::GetPathGoal,
                                        PlannerExecution> {
public:
  using Base = ActionBase<ayanami::actions::GetPathGoal, PlannerExecution>;
  using Goal = ayanami::actions::GetPathGoal;
  using Result = ayanami::actions::GetPathResult;
  using Feedback = ayanami::actions::GetPathFeedback;
  using Server = ActionServer<Goal, Result, Feedback>;
  using ServerPtr = std::shared_ptr<Server>;
  using Task = Server::Task;

  explicit PlannerAction(ayanami::abstract::AbstractPlanner::Ptr planner)
      : planner(std::move(planner)),
        action_server(std::make_shared<Server>()) {
    action_server->setGoalCallback([](const Server::GoalHandleType& gh) {
      ayanami::interfaces::NavigationCallbacks::on_get_path_goal(gh.goal);
    });
    action_server->setCancelCallback([this](uint64_t goal_id) {
      Base::cancelGoal(goal_id);
    });
    action_server->setFeedbackCallback([](const GoalFeedback<Feedback>& gf) {
      ayanami::interfaces::NavigationCallbacks::on_get_path_feedback(gf.feedback);
    });
    action_server->setResultCallback([](const Server::GoalResultType& gr) {
      ayanami::interfaces::NavigationCallbacks::on_get_path_result(gr);
    });
  }

  ~PlannerAction() override { cancelAll(); }

  /// @brief 主流: 接受 goal 并拿 TaskHandle 直连 (spec §3.4)。
  /// 同一 execution 在跑且插件支持 in-place 目标更新时走 latest-wins seam
  /// (不 cancel / 不新建 execution), 旧 goal 立即以 CANCELED 结算。
  Task acceptGoal(const Goal& goal) {
    auto task = action_server->acceptGoal(goal);
    auto gh = action_server->getGoal(task.goal_id());
    if (!gh) {
      return task;
    }
    if (!planner) {
      Result result;
      result.error = ErrorCode::INTERNAL_ERROR;
      result.message = "Planner plugin is null";
      result.session_id = goal.session_id;
      result.plan_version = goal.plan_version;
      action_server->publishResult(gh->id, ayanami::actions::GoalStatus::ABORTED,
                             result);
      return task;
    }
    {
      std::lock_guard<std::mutex> lock(goal_mutex);
      if (active_execution && Base::isRunning(0) &&
          active_execution->setNewGoal(goal.start, goal.goal,
                                       goal.tolerance)) {
        if (active_goal_id != 0) {
          Result canceled;
          canceled.error = ErrorCode::CANCELED;
          canceled.message = "Superseded by a newer planner goal";
          canceled.session_id = active_goal.session_id;
          canceled.plan_version = active_goal.plan_version;
          action_server->publishResult(active_goal_id,
                                       ayanami::actions::GoalStatus::CANCELED,
                                       canceled);
        }
        active_goal_id = gh->id;
        active_goal = goal;
        action_server->setActive(gh->id);
        return task;
      }
      active_execution = createExecution();
      active_goal_id = gh->id;
      active_goal = goal;
    }
    Base::start(*gh, active_execution, 0);
    return task;
  }

  bool cancelGoal(uint64_t goal_id) { return action_server->cancelGoal(goal_id); }

  /// @brief 驱动任务异常兜底: 发布 ABORTED 保证 TaskHandle 完成 (幂等)。
  void abortGoal(const GoalHandleType& gh) override {
    Result result;
    result.error = ErrorCode::INTERNAL_ERROR;
    result.message = "planner action driver exception";
    result.session_id = gh.goal.session_id;
    result.plan_version = gh.goal.plan_version;
    action_server->publishResult(gh.id,
                                 ayanami::actions::GoalStatus::ABORTED, result);
    clearActiveExecution(gh.id, active_execution.get());
  }

  void cancelAll() {
    action_server->cancelAll();
    Base::cancelAll();
  }

  ServerPtr server() const { return action_server; }

  /// @brief 注入 execution 工厂 (server 装配用: 经地图族工厂返回带地图的
  /// execution)。未注入时 createExecution 默认构造 abstract execution。
  void setExecutionFactory(std::function<PlannerExecution::Ptr()> factory) {
    execution_factory = std::move(factory);
  }

protected:
  void runImpl(const GoalHandleType& gh, PlannerExecution& exec) override {
    action_server->setActive(gh.id);
    exec.start(gh.goal.start, gh.goal.goal, gh.goal.tolerance);
    while (true) {
      uint64_t goal_id = 0;
      Goal goal;
      {
        std::lock_guard<std::mutex> lock(goal_mutex);
        if (active_execution.get() == &exec) {
          goal_id = active_goal_id;
          goal = active_goal;
        } else {
          // This execution was replaced because the plugin does not support
          // in-place goal updates. It must complete its own original handle.
          goal_id = gh.id;
          goal = gh.goal;
        }
      }
      if (goal_id != 0 && action_server->isCancelRequested(goal_id)) {
        exec.cancel();
      }
      const bool canceled = goal_id != 0 &&
          (action_server->isCancelRequested(goal_id) ||
           exec.getState() == PlannerExecution::CANCELED);
      if (canceled) {
        finishResult(goal_id, goal, exec,
                     ayanami::actions::GoalStatus::CANCELED);
        clearActiveExecution(goal_id, &exec);
        return;
      }
      if (exec.getState() == PlannerExecution::FOUND_PLAN) {
        finishResult(goal_id, goal, exec,
                     ayanami::actions::GoalStatus::SUCCEEDED);
        clearActiveExecution(goal_id, &exec);
        return;
      }
      switch (exec.getState()) {
        case PlannerExecution::NO_PLAN_FOUND:
        case PlannerExecution::STOPPED:
        case PlannerExecution::INTERNAL_ERROR:
          finishResult(goal_id, goal, exec,
                       ayanami::actions::GoalStatus::ABORTED);
          clearActiveExecution(goal_id, &exec);
          return;
        default:
          break;
      }
      exec.waitForStateUpdate(std::chrono::milliseconds(20));
    }
  }

  virtual PlannerExecution::Ptr createExecution() {
    if (execution_factory) {
      return execution_factory();
    }
    return std::make_shared<PlannerExecution>(planner);
  }

protected:
  ayanami::abstract::AbstractPlanner::Ptr planner;

private:
  std::function<PlannerExecution::Ptr()> execution_factory;
  void finishResult(uint64_t goal_id, const Goal& goal,
                    const PlannerExecution& exec,
                    ayanami::actions::GoalStatus status) {
    if (goal_id == 0) return;
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
    result.path.header = goal.goal.header;
    result.path.poses = exec.getPlan().poses;
    result.session_id = goal.session_id;
    result.plan_version = goal.plan_version;
    action_server->publishResult(goal_id, status, result);
  }

  void clearActiveExecution(uint64_t goal_id,
                            const PlannerExecution* execution) {
    std::lock_guard<std::mutex> lock(goal_mutex);
    if (active_goal_id == goal_id && active_execution.get() == execution) {
      active_goal_id = 0;
      active_execution.reset();
    }
  }

  ServerPtr action_server;
  mutable std::mutex goal_mutex;
  PlannerExecution::Ptr active_execution;
  uint64_t active_goal_id{0};
  Goal active_goal;
};

} // namespace components
} // namespace ayanami
