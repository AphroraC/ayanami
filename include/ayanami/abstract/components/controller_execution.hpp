#pragma once

#include <spdlog/spdlog.h>
#include "ayanami/abstract/components/execution_base.hpp"
#include "ayanami/abstract/core/abstract_controller.hpp"
#include "ayanami/abstract/core/abstract_goal_checker.hpp"
#include "ayanami/abstract/core/abstract_progress_checker.hpp"
#include "ayanami/abstract/components/robot_state_controller.hpp"
#include "ayanami/interfaces/callbacks.hpp"
#include "ayanami/types.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>

namespace ayanami {
namespace components {

/**
 * @brief Local controller execution (periodic loop).
 *
 * Loops at controller_frequency: progress_checker judges progress ->
 * goal_checker judges arrival -> computeVelocityCommand; cmd_vel is
 * published via the NavigationCallbacks::on_velocity_command side path.
 *
 * Single source of truth for robot state:
 * RobotStateController::instance() (pose + velocity).
 *
 * Key discipline (Global Constraints): force-clear the TwistStamped before
 * every publish, fixing a historical dirty-memory bug;
 * publishStopCommand() also clears explicitly.
 *
 * Failure semantics (responsibility table): when the plugin returns a
 * failure code or an invalid command, the execution layer publishes a
 * zeroed command and keeps looping (the execution itself keeps no retry
 * count); only when patience > 0 and no valid command has appeared for
 * longer than that duration since start is a terminal state reported
 * (mirroring the nav2 controller failure_tolerance). Retry and recovery
 * decisions are always made by the BT.
 */
class ControllerExecution : public ExecutionBase {
public:
  using Ptr = std::shared_ptr<ControllerExecution>;

  enum ControllerState {
    INITIALIZED,
    STARTED,
    PLANNING,
    NO_PLAN,
    EMPTY_PLAN,
    INVALID_PLAN,
    NO_LOCAL_CMD,
    GOT_LOCAL_CMD,
    ARRIVED_GOAL,
    NO_PROGRESS,
    CANCELED,
    STOPPED,
    INTERNAL_ERROR,
    ROBOT_DISABLED,
  };

  explicit ControllerExecution(ayanami::abstract::AbstractController::Ptr controller)
      : controller(std::move(controller)) {}

  /// @brief Set the path to follow (can be updated while running, safe for continuous replanning).
  void setNewPlan(const nav_msgs::Path& plan) {
    std::lock_guard<std::mutex> guard(plan_mutex);
    new_plan = true;
    this->plan = plan;
  }

  /// @brief Submit the plan version for this goal (pending); it becomes
  /// applied only after the run loop consumes it and setPlan succeeds
  /// (getPlanVersion only reports the applied version).
  void setPlanVersion(uint64_t plan_version) {
    std::lock_guard<std::mutex> guard(plan_mutex);
    pending_plan_version = plan_version;
  }

  /// @brief Applied version: advances only after the run loop consumes a
  /// new plan and controller->setPlan succeeds (in-flight updates are not
  /// counted).
  uint64_t getPlanVersion() const {
    std::lock_guard<std::mutex> guard(plan_mutex);
    return applied_plan_version;
  }

  /// @brief In-place update of path+version while running (no cancel/join/
  /// new Execution). Consumed by the run loop on the next cycle.
  bool updatePlan(const nav_msgs::Path& plan, uint64_t plan_version) {
    std::lock_guard<std::mutex> guard(plan_mutex);
    if (isTerminalLocked()) {
      return false;
    }
    new_plan = true;
    this->plan = plan;
    pending_plan_version = plan_version;
    return true;
  }

  bool hasNewPlan() {
    std::lock_guard<std::mutex> guard(plan_mutex);
    return new_plan;
  }

  /// @brief Atomically consume path+version, preventing the update thread from interleaving between two reads.
  std::pair<nav_msgs::Path, uint64_t> takeNewPlan() {
    std::lock_guard<std::mutex> guard(plan_mutex);
    new_plan = false;
    return {plan, pending_plan_version};
  }

  void setControllerFrequency(double frequency) {
    if (frequency > 0.0) {
      controller_frequency = frequency;
    }
  }

  double getControllerFrequency() const { return controller_frequency; }

  /// @brief Patience for lacking valid commands (seconds, mirrors nav2
  /// failure_tolerance); <=0 disables it (default): failures publish a
  /// zeroed command and keep looping; terminal states are decided only by
  /// arrival/progress/cancel/stop.
  void setPatience(double seconds) {
    patience = std::chrono::duration<double>(seconds);
  }

  double getPatience() const {
    return patience.count();
  }

  /// @brief Grace window for transient robot-pose loss (seconds, from
  /// navigation.velocity_timeout). <=0 keeps the old semantics: the first
  /// failed pose query is terminal. Within the window the loop publishes
  /// zeroed commands and keeps trying, so a single dropped TF frame does
  /// not abort the whole FollowPath task.
  void setPoseGrace(double seconds) {
    pose_grace = std::chrono::duration<double>(seconds);
  }

  double getPoseGrace() const { return pose_grace.count(); }

  bool isPatienceExceeded() const {
    if (patience <= std::chrono::duration<double>(0)) return false;
    return std::chrono::steady_clock::now() - start_time > patience;
  }

  void setMaximumVelocity(double linear_x, double linear_y,
                          double angular_z) {
    max_linear_x = linear_x;
    max_linear_y = linear_y;
    max_angular_z = angular_z;
  }

  /// @brief Inject the goal-checker plugin (wired by NavigationServerBase createActions).
  void setGoalChecker(ayanami::abstract::AbstractGoalChecker::Ptr checker) {
    goal_checker = std::move(checker);
  }

  ayanami::abstract::AbstractGoalChecker::Ptr getGoalChecker() const {
    return goal_checker;
  }

  /// @brief Inject the progress-checker plugin (same as above, consumed at runtime).
  void setProgressChecker(
      ayanami::abstract::AbstractProgressChecker::Ptr checker) {
    progress_checker = std::move(checker);
  }

  ayanami::abstract::AbstractProgressChecker::Ptr getProgressChecker() const {
    return progress_checker;
  }

  ControllerState getState() const {
    return static_cast<ControllerState>(state.load());
  }

  /// @brief Override: publish a stop command with a cleared TwistStamped (safety cancel).
  void publishStopCommand() override {
    geometry_msgs::TwistStamped cmd = {};  // all fields zeroed
    ayanami::interfaces::NavigationCallbacks::on_velocity_command(cmd);
  }

  /// @brief Cancel: set cancel_requested -> forward to the controller
  /// plugin -> publish a zeroed stop command -> notify to wake
  /// waitForStateUpdate.
  void cancel() override {
    cancel_requested = true;
    if (controller) controller->cancel();
    publishStopCommand();
    condition.notify_all();
  }

protected:
  /// @brief Plugin call hook: by default calls the abstract signature;
  /// map-family executions override it per cycle with a typed call (plugins
  /// never touch map modules).
  virtual ResultT<void> callController(
      const geometry_msgs::PoseStamped& pose,
      const geometry_msgs::TwistStamped& velocity,
      geometry_msgs::TwistStamped& cmd) {
    return controller->computeVelocityCommand(pose, velocity, cmd);
  }

  void run() override {
    start_time = std::chrono::steady_clock::now();
    setState(STARTED, false);

    // Loop period from controller_frequency
    const auto period = std::chrono::microseconds(
        static_cast<long long>(1e6 / controller_frequency));

    nav_msgs::Path plan;
    if (!hasNewPlan()) {
      error = ErrorCode::NO_VALID_COMMAND;
      message = "No plan was provided";
      setState(NO_PLAN, true);
      return;
    }

    // goal/progress checkers must be injected: a missing one is an internal error; no fallback to the controller's own judgment.
    if (!goal_checker || !progress_checker) {
      error = ErrorCode::INTERNAL_ERROR;
      message = !goal_checker ? "Goal checker is not set"
                               : "Progress checker is not set";
      publishStopCommand();
      setState(INTERNAL_ERROR, true);
      return;
    }

    try {
      while (true) {
        if (cancel_requested) {
          error = ErrorCode::CANCELED;
          publishStopCommand();
          setState(CANCELED, true);
          return;
        }
        if (should_exit) {
          error = ErrorCode::CANCELED;
          publishStopCommand();
          setState(STOPPED, true);
          return;
        }

        if (!safetyCheck()) {
          publishStopCommand();
          sleepInterruptibly(period);
          continue;
        }

        // Dynamically update the path (safe for continuous replanning)
        if (hasNewPlan()) {
          uint64_t plan_version = 0;
          std::tie(plan, plan_version) = takeNewPlan();
          if (plan.poses.empty()) {
            error = ErrorCode::PATH_INVALID;
            message = "The path is empty";
            setState(EMPTY_PLAN, true);
            return;
          }
          if (!controller->setPlan(plan)) {
            error = ErrorCode::PATH_INVALID;
            message = "The controller rejected the path";
            setState(INVALID_PLAN, true);
            return;
          }
          // The version becomes applied only after setPlan succeeds (results report only applied versions).
          {
            std::lock_guard<std::mutex> guard(plan_mutex);
            applied_plan_version = plan_version;
          }
          // After a new plan is applied: snapshot the goal pose; reset the
          // goal/progress baselines only when the goal actually changed.
          // Replanning/re-smoothing toward the same goal must NOT restart
          // the progress window -- otherwise continuous replanning defeats
          // stuck detection (the checker never accumulates its time_period)
          // and recovery can never trigger.
          const geometry_msgs::PoseStamped new_goal = plan.poses.back();
          if (!sameGoalPose(new_goal, goal)) {
            goal = new_goal;
            goal_checker->reset();
            progress_checker->reset();
          }
        }

        // Single source of truth for robot state: global RobotStateController (pose + latest velocity).
        geometry_msgs::PoseStamped robot_pose;
        geometry_msgs::TwistStamped robot_velocity;
        if (!RobotStateController::instance().getRobotState(
                robot_pose, robot_velocity)) {
          const auto now = std::chrono::steady_clock::now();
          if (!pose_failure_since) {
            pose_failure_since = now;
          }
          if (pose_grace.count() <= 0.0 ||
              now - *pose_failure_since > pose_grace) {
            error = ErrorCode::TRANSFORM_UNAVAILABLE;
            message = "Could not get the robot pose";
            publishStopCommand();
            setState(INTERNAL_ERROR, true);
            return;
          }
          // Transient pose loss within the grace window: publish a zeroed
          // command and keep looping instead of aborting the task.
          publishStopCommand();
          setState(NO_LOCAL_CMD, false);
          sleepInterruptibly(period);
          continue;
        }
        pose_failure_since.reset();

        // Goal check: goal_checker is the sole arrival decision
        if (goal_checker->isGoalReached(robot_pose, goal, robot_velocity)) {
          error = ErrorCode::NONE;
          message = "Reached the goal";
          publishStopCommand();
          setState(ARRIVED_GOAL, true);
          return;
        }

        // Only declare the robot stuck after ruling out successful arrival.
        if (!progress_checker->check(robot_pose)) {
          error = ErrorCode::NO_PROGRESS;
          message = "No progress made executing the path";
          publishStopCommand();
          setState(NO_PROGRESS, true);
          return;
        }

        setState(PLANNING, false);

        // Zero-before-publish discipline: the buffer is cleared before the plugin call and overwritten on plugin success.
        geometry_msgs::TwistStamped cmd_vel = {};
        const ResultT<void> result =
            callController(robot_pose, robot_velocity, cmd_vel);

        // Cancel/stop during the plugin call: do not publish this round's
        // command (prevents leaking a stale velocity command at recovery
        // startup); CANCELED/STOPPED is settled uniformly at the top of the
        // next loop round.
        if (cancel_requested || should_exit) {
          continue;
        }

        if (result.ok) {
          if (!validVelocityCommand(cmd_vel)) {
            error = ErrorCode::NO_VALID_COMMAND;
            message = "Controller produced an invalid velocity command";
            publishStopCommand();
          } else {
            error = ErrorCode::NONE;
            message = result.message;
            setState(GOT_LOCAL_CMD, false);
            ayanami::interfaces::NavigationCallbacks::on_velocity_command(cmd_vel);
          }
        } else {
          error = result.error;
          message = result.message;
        }

        if (error != ErrorCode::NONE) {
          if (isPatienceExceeded()) {
            error = ErrorCode::TIMEOUT;
            publishStopCommand();
            setState(INTERNAL_ERROR, true);
            return;
          }
          setState(NO_LOCAL_CMD, false);
          publishStopCommand();
        }

        sleepInterruptibly(period);
      }
    } catch (...) {
      error = ErrorCode::INTERNAL_ERROR;
      message = "Unknown error occurred";
      publishStopCommand();
      setState(INTERNAL_ERROR, true);
    }
  }

// Members are protected: derived classes (e.g. CostmapControllerExecution)
// can override the callController hook to inject map access (without
// replicating the entire run loop).
protected:
  ayanami::abstract::AbstractController::Ptr controller;
  ayanami::abstract::AbstractGoalChecker::Ptr goal_checker;
  ayanami::abstract::AbstractProgressChecker::Ptr progress_checker;
  ayanami::geometry_msgs::PoseStamped goal;

  mutable std::mutex plan_mutex;
  nav_msgs::Path plan;
  bool new_plan{false};
  uint64_t pending_plan_version{0};
  uint64_t applied_plan_version{0};

  /// @brief Call with the lock held: whether the state machine has reached a terminal state (run loop has returned).
  bool isTerminalLocked() const {
    switch (static_cast<ControllerState>(state.load())) {
      case NO_PLAN:
      case EMPTY_PLAN:
      case INVALID_PLAN:
      case NO_PROGRESS:
      case ARRIVED_GOAL:
      case CANCELED:
      case STOPPED:
      case INTERNAL_ERROR:
      case ROBOT_DISABLED:
        return true;
      default:
        return false;
    }
  }

  /// @brief Periodic wait; cancel/stop interrupts immediately (does not wait out the full period).
  void sleepInterruptibly(std::chrono::microseconds period) {
    std::unique_lock<std::mutex> lock(mutex);
    condition.wait_for(lock, period, [this] {
      return cancel_requested.load() || should_exit.load();
    });
  }

  std::chrono::duration<double> pose_grace{0};
  std::optional<std::chrono::steady_clock::time_point> pose_failure_since;

  double controller_frequency{20.0};
  double max_linear_x{std::numeric_limits<double>::infinity()};
  double max_linear_y{std::numeric_limits<double>::infinity()};
  double max_angular_z{std::numeric_limits<double>::infinity()};
  std::chrono::duration<double> patience{0};
  std::chrono::steady_clock::time_point start_time;

  /// @brief Goal-change detection for checker resets: position/orientation
  /// only (header stamps change on every replan and must not count).
  static bool sameGoalPose(const geometry_msgs::PoseStamped& lhs,
                           const geometry_msgs::PoseStamped& rhs) {
    return lhs.pose.position.isApprox(rhs.pose.position) &&
           lhs.pose.orientation.isApprox(rhs.pose.orientation);
  }

  bool validVelocityCommand(
      const geometry_msgs::TwistStamped& command) const {
    const auto& linear = command.twist.linear;
    const auto& angular = command.twist.angular;
    return std::isfinite(linear.x()) && std::isfinite(linear.y()) &&
           std::isfinite(linear.z()) && std::isfinite(angular.x()) &&
           std::isfinite(angular.y()) && std::isfinite(angular.z()) &&
           std::abs(linear.x()) <= max_linear_x &&
           std::abs(linear.y()) <= max_linear_y &&
           std::abs(angular.z()) <= max_angular_z;
  }
};

} // namespace components
} // namespace ayanami
