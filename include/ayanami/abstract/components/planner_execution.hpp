#pragma once


#include "ayanami/abstract/components/execution_base.hpp"
#include "ayanami/abstract/components/robot_state_controller.hpp"
#include "ayanami/abstract/core/abstract_planner.hpp"
#include "ayanami/types.hpp"

#include <memory>

namespace ayanami {
namespace components {

/**
 * @brief Global path planning execution (single run, modeled after MBF
 * AbstractPlannerExecution).
 *
 * Responsibility table: the module only performs path search; a failure is
 * mapped immediately to an ErrorCode terminal state, and retries are
 * decided by the BT (RecoveryNode) -- the execution layer does no retry/
 * patience loop.
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

  /// @brief Record start/goal/tolerance and run the planning task to a
  /// terminal state on the calling thread.
  /// An empty start is resolved to the latest robot pose HERE (framework
  /// responsibility); modules always receive a fully valid start.
  void start(const geometry_msgs::PoseStamped& start,
             const geometry_msgs::PoseStamped& goal, double tolerance) {
    prepareGoal(start, goal, tolerance);
    runBlocking();
  }

  /// @brief Goal inputs for one planning task (the resident action worker
  /// calls prepareGoal + runBlocking).
  void prepareGoal(const geometry_msgs::PoseStamped& start,
                   const geometry_msgs::PoseStamped& goal, double tolerance) {
    start_pose = resolveStart(start);
    goal_pose = goal;
    goal_tolerance = tolerance;
    plan = nav_msgs::Path{};
  }

  /// @brief Cancel: set cancel_requested and forward to the planner module (cooperative).
  void cancel() override {
    cancel_requested = true;
    if (planner) planner->cancel();
    condition.notify_all();
  }

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
  /// @brief Module call hook: by default calls the abstract signature;
  /// map-family executions override it with a typed call (the execution
  /// layer performs map access; modules never touch map modules).
  virtual ResultT<void> callPlanner(
      const geometry_msgs::PoseStamped& start,
      const geometry_msgs::PoseStamped& goal, double tolerance,
      nav_msgs::Path& plan) {
    return planner->makePlan(start, goal, tolerance, plan);
  }

  void run() override {
    setState(STARTED, false);
    try {
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
      setState(PLANNING, false);

      // in-place: the module writes the result member directly (single-shot
      // execution; the FOUND_PLAN terminal state makes the write visible to
      // the action reader via the state machine's happens-before edge), so
      // no buffer + move round-trip is needed.
      const ResultT<void> result =
          callPlanner(start_pose, goal_pose, goal_tolerance, plan);

      if (cancel_requested) {
        error = ErrorCode::CANCELED;
        setState(CANCELED, true);
        return;
      }
      if (result.ok) {
        // plan was written in place by the module (see callPlanner arg).
        error = ErrorCode::NONE;
        message = result.message;
        setState(FOUND_PLAN, true);
        return;
      }
      // Return the error code immediately on failure (retries are decided by the BT).
      error = result.error;
      message = result.message;
      setState(NO_PLAN_FOUND, true);
    } catch (...) {
      error = ErrorCode::INTERNAL_ERROR;
      message = "Unknown error occurred";
      setState(INTERNAL_ERROR, true);
    }
  }

protected:
  /// @brief Interface contract helper (AbstractPlanner): an empty start
  /// (default-constructed pose) is resolved to the latest robot pose via the
  /// RobotStateController so modules never interpret sentinels themselves.
  /// An unresolvable start is passed through unchanged; the module then
  /// fails with START_INVALID and the BT recovery branch takes over.
  static geometry_msgs::PoseStamped resolveStart(
      const geometry_msgs::PoseStamped& start) {
    if (!start.header.frame_id.empty() || start.pose.position.norm() > 0.0) {
      return start;
    }
    return ayanami::components::RobotStateController::instance().getRobotPose();
  }

  ayanami::abstract::AbstractPlanner::Ptr planner;
  geometry_msgs::PoseStamped start_pose;
  geometry_msgs::PoseStamped goal_pose;
  double goal_tolerance{0.0};
  nav_msgs::Path plan;
};

} // namespace components
} // namespace ayanami
