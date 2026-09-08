#pragma once


#include "ayanami/abstract/components/execution_base.hpp"
#include "ayanami/abstract/core/abstract_planner.hpp"
#include "ayanami/types.hpp"

#include <cstdint>
#include <memory>
#include <mutex>

namespace ayanami {
namespace components {

/**
 * @brief Global path planning execution (single run, modeled after MBF
 * AbstractPlannerExecution).
 *
 * Responsibility table: the plugin only performs path search; a failure is
 * mapped immediately to an ErrorCode terminal state, and retries are
 * decided by the BT (RecoveryNode) -- the execution layer does no retry/
 * patience loop.
 *
 * latest-wins goal seam: setNewGoal stores the latest start/goal/tolerance
 * into a pending slot; run() adopts the latest pending goal before each
 * plugin call, and after a plugin call returns, if the revision has
 * advanced it restarts with the latest goal (no cancel / no join / no new
 * thread).
 *
 * The internal state machine is independent of actions::GoalStatus:
 * INITIALIZED->STARTED->PLANNING->(FOUND_PLAN | NO_PLAN_FOUND | CANCELED |
 * STOPPED | INTERNAL_ERROR).
 */
class PlannerExecution : public ExecutionBase {
public:
  using Ptr = std::shared_ptr<PlannerExecution>;

  enum PlanningState {
    INITIALIZED,
    STARTED,
    PLANNING,
    FOUND_PLAN,
    NO_PLAN_FOUND,
    CANCELED,
    STOPPED,
    INTERNAL_ERROR,
  };

  explicit PlannerExecution(ayanami::abstract::AbstractPlanner::Ptr planner)
      : planner(std::move(planner)) {}

  /// @brief Record start/goal/tolerance and start the planning thread.
  using ExecutionBase::start;
  void start(const geometry_msgs::PoseStamped& start,
             const geometry_msgs::PoseStamped& goal, double tolerance) {
    this->start_pose = start;
    this->goal_pose = goal;
    this->goal_tolerance = tolerance;
    ExecutionBase::start();
  }

  /// @brief Cancel: set cancel_requested and forward to the planner plugin (cooperative).
  void cancel() override {
    cancel_requested = true;
    if (planner) planner->cancel();
    condition.notify_all();
  }

  /// @brief In-place goal update while running (latest-wins update seam).
  /// @return false means the execution has reached a terminal state or the
  /// plugin does not support it; the caller should fall back to creating a
  /// new execution.
  virtual bool setNewGoal(const geometry_msgs::PoseStamped& start,
                          const geometry_msgs::PoseStamped& goal,
                          double tolerance) {
    if (!planner || !planner->setNewGoal(start, goal, tolerance)) {
      return false;
    }
    {
      std::lock_guard<std::mutex> lock(goal_mutex);
      if (isTerminalState()) return false;
      pending_start = start;
      pending_goal = goal;
      pending_tolerance = tolerance;
      goal_revision.fetch_add(1);
    }
    condition.notify_all();
    return true;
  }

  /// @brief Current pending goal revision (for latest-wins observation/testing).
  uint64_t pendingRevision() const { return goal_revision.load(); }

  PlanningState getState() const {
    return static_cast<PlanningState>(state.load());
  }

  /// @brief The most recently computed path.
  const nav_msgs::Path& getPlan() const { return plan; }

  /// @brief Move-out: the action layer takes the path once the execution is
  /// terminal, avoiding a full path copy. Single-shot execution (the member is
  /// not reused afterwards).
  nav_msgs::Path takePlan() { return std::move(plan); }

protected:
  /// @brief Plugin call hook: by default calls the abstract signature;
  /// map-family executions override it with a typed call (the execution
  /// layer performs map access; plugins never touch map modules).
  virtual ResultT<void> callPlanner(
      const geometry_msgs::PoseStamped& start,
      const geometry_msgs::PoseStamped& goal, double tolerance,
      nav_msgs::Path& plan) {
    return planner->makePlan(start, goal, tolerance, plan);
  }

  void run() override {
    setState(STARTED, false);
    // Currently adopted goal revision (latest-wins): each loop round adopts the latest pending goal.
    uint64_t adopted_revision = 0;
    ayanami::geometry_msgs::PoseStamped cur_start = this->start_pose;
    ayanami::geometry_msgs::PoseStamped cur_goal = this->goal_pose;
    double cur_tolerance = this->goal_tolerance;
    try {
      while (true) {
        if (cancel_requested) {
          error = ErrorCode::CANCELED;
          setState(CANCELED, true);
          return;
        }
        if (should_exit) {
          error = ErrorCode::CANCELED;
          setState(STOPPED, true);
          return;
        }
        // latest-wins: if setNewGoal has pushed a new revision, adopt the latest pending goal.
        {
          std::lock_guard<std::mutex> lock(goal_mutex);
          if (goal_revision.load() != adopted_revision) {
            cur_start = pending_start;
            cur_goal = pending_goal;
            cur_tolerance = pending_tolerance;
            adopted_revision = goal_revision.load();
          }
        }
        setState(PLANNING, false);

        // in-place: the plugin writes the result member directly (single-shot
        // execution; the FOUND_PLAN terminal state makes the write visible to
        // the action reader via the state machine's happens-before edge), so
        // no buffer + move round-trip is needed.
        const ResultT<void> result =
            callPlanner(cur_start, cur_goal, cur_tolerance, plan);

        // A new goal arrived during the plugin call (revision advanced): do
        // not commit this result; replan for the latest goal on the same
        // execution thread (latest-wins, no cancel/new thread).
        {
          std::lock_guard<std::mutex> lock(goal_mutex);
          if (goal_revision.load() != adopted_revision) {
            continue;
          }
        }

        if (cancel_requested) {
          error = ErrorCode::CANCELED;
          setState(CANCELED, true);
          return;
        }
        if (result.ok) {
          // plan was written in place by the plugin (see callPlanner arg).
          error = ErrorCode::NONE;
          message = result.message;
          setState(FOUND_PLAN, true);
          return;
        }
        // Return the error code immediately on failure (retries are decided by the BT).
        error = result.error;
        message = result.message;
        setState(NO_PLAN_FOUND, true);
        return;
      }
    } catch (...) {
      error = ErrorCode::INTERNAL_ERROR;
      message = "Unknown error occurred";
      setState(INTERNAL_ERROR, true);
    }
  }

  /// @brief Whether the current state is terminal (run has exited / no longer consumes new goals).
  bool isTerminalState() const {
    const auto s = static_cast<PlanningState>(state.load());
    return s == FOUND_PLAN || s == NO_PLAN_FOUND || s == CANCELED ||
           s == STOPPED || s == INTERNAL_ERROR;
  }

protected:
  ayanami::abstract::AbstractPlanner::Ptr planner;
  geometry_msgs::PoseStamped start_pose;
  geometry_msgs::PoseStamped goal_pose;
  double goal_tolerance{0.0};
  nav_msgs::Path plan;

  // ---- latest-wins update seam state (thread-safe) ----
  mutable std::mutex goal_mutex;
  geometry_msgs::PoseStamped pending_start;
  geometry_msgs::PoseStamped pending_goal;
  double pending_tolerance{0.0};
  std::atomic<uint64_t> goal_revision{0};
};

} // namespace components
} // namespace ayanami
