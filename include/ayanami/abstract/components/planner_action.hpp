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
 * @brief Global path planning Action (modeled after MBF PlannerAction).
 *
 * acceptGoal returns a TaskHandle straight to the caller (mainline, BT
 * nodes); the result is also emitted to
 * NavigationCallbacks::on_get_path_result as a side path (rviz extension/
 * fallback/monitoring).
 * ExecutionState->GoalStatus mapping: FOUND_PLAN->SUCCEEDED,
 * CANCELED->CANCELED, other terminal states (NO_PLAN_FOUND/STOPPED/
 * INTERNAL_ERROR) ->ABORTED + ErrorCode.
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

  /// @brief Mainline: accept a goal and get a TaskHandle straight away.
  /// When the same execution is running and the plugin supports in-place
  /// goal updates, take the latest-wins seam (no cancel / no new
  /// execution); the old goal is settled immediately as CANCELED.
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
      if (active_execution && Base::isRunning() &&
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
    Base::start(*gh, active_execution);
    return task;
  }

  bool cancelGoal(uint64_t goal_id) { return action_server->cancelGoal(goal_id); }

  /// @brief Driver task exception fallback: publish ABORTED to guarantee TaskHandle completion (idempotent).
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

  /// @brief Inject an execution factory (for server assembly: the
  /// map-family factory returns executions carrying a map). When not
  /// injected, createExecution default-constructs the abstract execution.
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
      // The fallback exception terminal state (k_aborted) is not recognized
      // by the PlannerExecution enum and must be settled here, otherwise the
      // driver task spins forever and the slot occupancy is never released.
      if (exec.aborted()) {
        finishResult(goal_id, goal, exec,
                     ayanami::actions::GoalStatus::ABORTED);
        clearActiveExecution(goal_id, &exec);
        return;
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
                    PlannerExecution& exec,
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
    result.path = exec.takePlan();
    result.path.header = goal.goal.header;
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
