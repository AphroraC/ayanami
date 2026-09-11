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
  /// Submitting while a task is in flight follows the base preemption
  /// protocol (in-flight task canceled, new goal runs after it settles);
  /// goal REVISIONS of the in-flight task go through updateGoal() instead
  /// (no cancel, no new task).
  Task acceptGoal(const Goal& goal) {
    auto task = action_server->acceptGoal(goal);
    auto gh = action_server->getGoal(task.goal_id());
    if (!gh) {
      return task;
    }
    if (!planner) {
      Result result;
      result.error = ErrorCode::INTERNAL_ERROR;
      result.message = "Planner module is null";
      result.session_id = goal.session_id;
      result.plan_version = goal.plan_version;
      action_server->publishResult(gh->id, ayanami::actions::GoalStatus::ABORTED,
                             result);
      return task;
    }
    Base::submitGoal(*gh);
    return task;
  }

  bool cancelGoal(uint64_t goal_id) { return action_server->cancelGoal(goal_id); }

  /// @brief In-place goal update of the running planning task (the planner
  /// analogue of ControllerAction::updatePlan). No cancel / no join / no new
  /// Execution: the execution adopts the latest start/goal/tolerance via its
  /// revision-guarded pending slot and replans on its own thread. The same
  /// TaskHandle keeps running; the goal CONTENT is replaced wholesale.
  /// Returns false when the goal is no longer running (task already
  /// finished/canceled) or the execution has reached a terminal state -- the
  /// caller should re-check the finished task instead of treating the miss
  /// as a failure.
  bool updateGoal(uint64_t goal_id, const Goal& goal) {
    auto exec = Base::executionForGoal(goal_id);
    if (!exec) {
      return false;
    }
    if (!exec->setNewGoal(goal.start, goal.goal, goal.tolerance)) {
      return false;
    }
    std::lock_guard<std::mutex> lock(goal_mutex);
    if (active_goal_id == goal_id) {
      active_goal = goal;  // finishResult reports the latest stamped goal
    }
    return true;
  }

  /// @brief Settle a goal preempted from the pending slot (canceled before
  /// its driver started) so its TaskHandle never hangs.
  void finishPendingGoal(const GoalHandleType& gh) override {
    Result result;
    result.error = ErrorCode::CANCELED;
    result.message = "canceled before the planner task started";
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
    result.message = "planner action driver exception";
    result.session_id = gh.goal.session_id;
    result.plan_version = gh.goal.plan_version;
    action_server->publishResult(gh.id,
                                 ayanami::actions::GoalStatus::ABORTED, result);
    clearActiveExecution(gh.id);
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
  void runGoal(const GoalHandleType& gh, PlannerExecution& exec) override {
    action_server->setActive(gh.id);
    {
      std::lock_guard<std::mutex> lock(goal_mutex);
      active_goal_id = gh.id;
      active_goal = gh.goal;
    }
    exec.prepareGoal(gh.goal.start, gh.goal.goal, gh.goal.tolerance);
    exec.runBlocking();
    // The fallback exception terminal state (k_aborted) is not recognized
    // by the PlannerExecution enum and must be settled via the abort path,
    // otherwise the in-flight record is never released.
    if (exec.aborted()) {
      abortGoal(gh);
      clearActiveExecution(gh.id);
      return;
    }
    const auto state = exec.getState();
    ayanami::actions::GoalStatus status;
    if (action_server->isCancelRequested(gh.id) ||
        state == PlannerExecution::CANCELED) {
      status = ayanami::actions::GoalStatus::CANCELED;
    } else if (state == PlannerExecution::FOUND_PLAN) {
      status = ayanami::actions::GoalStatus::SUCCEEDED;
    } else {
      status = ayanami::actions::GoalStatus::ABORTED;
    }
    finishResult(gh.id, gh.goal, exec, status);
    clearActiveExecution(gh.id);
  }

  PlannerExecution::Ptr createExecution() override {
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
    Feedback feedback;
    feedback.message = "goal finished";
    action_server->publishFeedback(goal_id, feedback);
    action_server->publishResult(goal_id, status, result);
  }

  void clearActiveExecution(uint64_t goal_id) {
    std::lock_guard<std::mutex> lock(goal_mutex);
    if (active_goal_id == goal_id) {
      active_goal_id = 0;
    }
  }

  ServerPtr action_server;
  mutable std::mutex goal_mutex;
  uint64_t active_goal_id{0};
  Goal active_goal;
};

} // namespace components
} // namespace ayanami
