#pragma once

#include <spdlog/spdlog.h>
#include <ayanami/utility/logging.hpp>
#include "ayanami/abstract/components/execution_base.hpp"
#include "ayanami/abstract/core/abstract_controller.hpp"
#include "ayanami/messages/actions.hpp"
#include "ayanami/abstract/core/abstract_goal_checker.hpp"
#include "ayanami/abstract/core/abstract_progress_checker.hpp"
#include "ayanami/abstract/components/robot_state_controller.hpp"
#include "ayanami/interfaces/callbacks.hpp"
#include "ayanami/types.hpp"

#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>

namespace ayanami {
namespace components {

/// @brief Content equivalence of two paths for plan-update dedup: same pose
/// count and every pose within tolerance (1 mm / ~1e-3 rad). A version bump
/// whose path content did not materially change (a converged smoother
/// re-publishing its own output) must not re-trigger controller->setPlan --
/// (DWB did a full critic reset per replan).
inline bool pathsEquivalent(const nav_msgs::Path& lhs, const nav_msgs::Path& rhs) {
  if (lhs.poses.size() != rhs.poses.size()) {
    return false;
  }
  // ABSOLUTE tolerances (1 mm position / 1e-3 quaternion component): a
  // relative isApprox would never match cells near the origin, which is
  // exactly where converged in-place refinements live.
  for (std::size_t i = 0; i < lhs.poses.size(); ++i) {
    if (!geometry_msgs::poseClose(lhs.poses[i].pose, rhs.poses[i].pose,
                                  1e-3, 1e-3)) {
      return false;
    }
  }
  return true;
}

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
 * Failure semantics (responsibility table): when the module returns a
 * failure code, an invalid command, or the safety/pose checks fail, the
 * execution layer STAYS SILENT for that cycle (no publish at all, nav2
 * controller_server semantics: it waits for the costmap without
 * publishing); the last valid command stands until the chassis-side
 * command timeout stops the robot. Only when patience > 0 and no valid
 * command has appeared for longer than that duration is a terminal state
 * reported -- terminal transitions publish exactly ONE zeroed stop pulse
 * (cancel/arrival/stuck/abort), never a zero stream (mirroring the nav2
 * controller failure_tolerance). Retry and recovery decisions are always
 * made by the BT.
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
    /// @brief Terminal: safetyCheck (map freshness) stayed false longer than
    /// controller_patience. Reported so the BT recovery branch (clear costmap
    /// / replan) takes over instead of the loop stalling in zero velocity.
    MAP_STALE,
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
  /// Content-signature dedup: a version bump whose path is equivalent to the
  /// one already held (converged smoother re-publishing its own output)
  /// advances only the version bookkeeping -- controller->setPlan (and with
  /// DWB's full critic reset) runs only on a real content change.
  bool updatePlan(const nav_msgs::Path& plan, uint64_t plan_version) {
    std::lock_guard<std::mutex> guard(plan_mutex);
    if (isTerminalLocked()) {
      return false;
    }
    pending_plan_version = plan_version;
    if (!new_plan && pathsEquivalent(plan, this->plan)) {
      // Content unchanged: semantically already applied. Advancing the
      // applied version here keeps results current (a lagging applied
      // version would fail the stale-result policy in FollowPath).
      applied_plan_version = std::max(applied_plan_version, plan_version);
      return true;
    }
    new_plan = true;
    this->plan = plan;
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

  /// @brief Patience (seconds, nav2 failure_tolerance semantics): the time
  /// without a valid command (module failure/invalid command) or with a
  /// failed safety check (stale map) that is tolerated before a terminal
  /// state is reported. 0 = fail on the first bad cycle; negative (default
  /// -1) = tolerate forever, matching the old "disabled" behavior. Retry and
  /// recovery decisions always belong to the BT.
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

  /// @brief Tracking-deviation guard (the framework-level equivalent of
  /// rose_navigation's LMPC deviation rejection): the nearest-point search
  /// always finds a reference no matter how far the robot was pushed, and a
  /// map-free tracker pulls it back along a straight line that may cross
  /// obstacles. When the robot strays farther than this from the current
  /// path, the task fails with NO_PROGRESS so the FSM replans. 0 disables
  /// (default; the rose deployment uses 0.5 m).
  void setDeviationThreshold(double meters) { deviation_threshold = meters; }

  double getDeviationThreshold() const { return deviation_threshold; }

  double getPoseGrace() const { return pose_grace.count(); }

  /// @brief nav2 failure_tolerance semantics: patience < 0 tolerates forever;
  /// patience >= 0 fails once the elapsed time since `since` (the last valid
  /// command, or the first stale safety check) exceeds it (0 = immediate).
  bool isPatienceExceeded(
      std::chrono::steady_clock::time_point since) const {
    if (patience < std::chrono::duration<double>(0)) return false;
    return std::chrono::steady_clock::now() - since > patience;
  }

  void setMaximumVelocity(double linear_x, double linear_y,
                          double angular_z) {
    max_linear_x = linear_x;
    max_linear_y = linear_y;
    max_angular_z = angular_z;
  }

  /// @brief Feedback sink (nav2 FollowPath semantics: speed /
  /// distance_remaining / estimated_time_remaining, emitted every control
  /// cycle through the action layer to the on_exe_path_feedback slot).
  using FeedbackFn =
      std::function<void(const ayanami::actions::ExePathFeedback&)>;
  void setFeedbackCallback(FeedbackFn callback) {
    feedback_callback = std::move(callback);
  }

  /// @brief Inject the goal-checker module (wired by NavigationServerBase createActions).
  void setGoalChecker(ayanami::abstract::AbstractGoalChecker::Ptr checker) {
    goal_checker = std::move(checker);
  }

  ayanami::abstract::AbstractGoalChecker::Ptr getGoalChecker() const {
    return goal_checker;
  }

  /// @brief Inject the progress-checker module (same as above, consumed at runtime).
  void setProgressChecker(
      ayanami::abstract::AbstractProgressChecker::Ptr checker) {
    progress_checker = std::move(checker);
  }

  ayanami::abstract::AbstractProgressChecker::Ptr getProgressChecker() const {
    return progress_checker;
  }

  /// @brief Force a fresh baseline for both checkers (execution reuse: a new
  /// FollowPath task on a reused execution must not inherit the previous
  /// task's checker state, even when the goal pose is unchanged -- the
  /// in-loop reset only fires on an actual goal CHANGE).
  void resetCheckers() {
    if (goal_checker) {
      goal_checker->reset();
    }
    if (progress_checker) {
      progress_checker->reset();
    }
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
  /// module -> publish a zeroed stop command -> notify to wake
  /// waitForStateUpdate.
  void cancel() override {
    cancel_requested = true;
    if (controller) controller->cancel();
    publishStopCommand();
    condition.notify_all();
  }

protected:
  /// @brief Module call hook: by default calls the abstract signature;
  /// map-family executions override it per cycle with a typed call (modules
  /// never touch map modules).
  virtual ResultT<void> callController(
      const geometry_msgs::PoseStamped& pose,
      const geometry_msgs::TwistStamped& velocity,
      geometry_msgs::TwistStamped& cmd) {
    return controller->computeVelocityCommand(pose, velocity, cmd);
  }

  void run() override {
    start_time = std::chrono::steady_clock::now();
    last_valid_time = start_time;
    // Execution reuse: stale outage windows from a previous run must not
    // leak into this one (the first stale cycle would otherwise look like a
    // long-standing outage and trip patience immediately).
    map_stale_since.reset();
    pose_failure_since.reset();
    // Execution reuse: a new run is a NEW TASK -- re-baseline the checkers
    // (the in-loop reset only fires on an actual goal CHANGE). Without this,
    // a re-follow after a NO_PROGRESS failure inherited the expired window
    // and failed again on its first cycle: the FSM's recovery loop burned
    // its whole retry budget at replan latency (6 retries in 0.6 s).
    resetCheckers();
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
      // Arrival contract: the goal checker owns it, so push its pose
      // tolerance into the controller before the cycle loop. Without this the
      // controller would hold station at its own configure-time distance,
      // which is unreachable if the checker's tolerance is tighter.
      if (controller) {
        controller->setGoalTolerances(goal_checker->getTolerances().pose);
        // Cycle-period contract: modules that integrate over time take their
        // dt from the injected cadence (1 / controller_frequency), never from
        // a period guessed in their own config.
        controller->setCyclePeriod(
            std::chrono::duration<double>(period).count());
      }
      while (true) {
        if (cancel_requested) {
          error = ErrorCode::CANCELED;
          // No stop pulse here: cancel() already published exactly one
          // (immediate, and also covers the not-running case); a second one
          // at the terminal would be redundant zero spam.
          setState(CANCELED, true);
          return;
        }
        if (should_exit) {
          error = ErrorCode::CANCELED;
          // Same as CANCELED: the cancel()/cancelAll() path published the
          // single stop pulse already.
          setState(STOPPED, true);
          return;
        }

        if (!safetyCheck()) {
          // Stale/unsafe map: stay silent this cycle (nav2 waits for the
          // costmap without publishing) and count the outage into
          // controller_patience (nav2 failure_tolerance semantics: <0 =
          // tolerate forever, 0 = first stale cycle aborts). Exceeding it is
          // a MAP_STALE terminal so the BT recovery branch (clear costmap /
          // replan) takes over instead of waiting forever.
          const auto now = std::chrono::steady_clock::now();
          if (!map_stale_since) {
            map_stale_since = now;
          }
          if (isPatienceExceeded(*map_stale_since)) {
            error = ErrorCode::MAP_STALE;
            message = "Map is not current (safety check failed)";
            publishStopCommand();
            setState(MAP_STALE, true);
            return;
          }
          sleepInterruptibly(period);
          continue;
        }
        map_stale_since.reset();

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
          // The version becomes applied only after setPlan succeeds (results
          // report only applied versions). Monotonic: a concurrent content-
          // equivalent update may have advanced the applied version past the
          // consumed one already.
          {
            std::lock_guard<std::mutex> guard(plan_mutex);
            applied_plan_version = std::max(applied_plan_version, plan_version);
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
          // The plan content changed: the next progress-metrics pass takes a
          // full nearest-point scan once (per replan, ~1Hz), then advances
          // incrementally along the trajectory (per control cycle).
          progress_index = 0;
          progress_index_full_scan = true;
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
          // Transient pose loss within the grace window: stay silent and
          // keep looping instead of aborting the task (the last valid
          // command stands; the chassis timeout is the stop net).
          setState(NO_LOCAL_CMD, false);
          sleepInterruptibly(period);
          continue;
        }
        pose_failure_since.reset();

        // Progress metrics for the cycle feedback (nav2 FollowPath
        // semantics): remaining length from the nearest path point, smoothed
        // average speed, ETA from the two.
        updateProgressMetrics(robot_pose, robot_velocity);
        emitFeedback();

        // Goal check: goal_checker is the sole arrival decision
        if (goal_checker->isGoalReached(robot_pose, goal, robot_velocity)) {
          error = ErrorCode::NONE;
          message = "Reached the goal";
          publishStopCommand();
          setState(ARRIVED_GOAL, true);
          return;
        }

        // Tracking-deviation guard: fail the task when the robot strays
        // beyond the threshold (NO_PROGRESS is recoverable -- the FSM
        // replans; a map-free tracker's straight-line pull-back may cross
        // obstacles, so waiting for the oscillation gate is too slow).
        if (deviation_threshold > 0.0 &&
            cross_track > deviation_threshold) {
          error = ErrorCode::NO_PROGRESS;
          message = "Tracking deviation exceeded the threshold";
          publishStopCommand();
          setState(NO_PROGRESS, true);
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

        // Zero-before-publish discipline: the buffer is cleared before the module call and overwritten on module success.
        geometry_msgs::TwistStamped cmd_vel = {};
        const ResultT<void> result =
            callController(robot_pose, robot_velocity, cmd_vel);

        // Cancel/stop during the module call: do not publish this round's
        // command (prevents leaking a stale velocity command at recovery
        // startup); CANCELED/STOPPED is settled uniformly at the top of the
        // next loop round.
        if (cancel_requested || should_exit) {
          continue;
        }

        if (result.ok) {
          if (!finiteVelocityCommand(cmd_vel)) {
            // Silent skip: the cycle counts into patience like any other
            // failure; no zero spam (the last valid command stands).
            error = ErrorCode::NO_VALID_COMMAND;
            message = "Controller produced a non-finite velocity command";
          } else {
            // Over-limit but finite commands are clamped into the caps (a
            // marginal overshoot must not stop the robot); NaN/inf were
            // rejected above.
            clampVelocityCommand(cmd_vel);
            error = ErrorCode::NONE;
            message = result.message;
            setState(GOT_LOCAL_CMD, false);
            // nav2 failure_tolerance semantics: the tolerance window is
            // measured from the last valid command, not from task start.
            last_valid_time = std::chrono::steady_clock::now();
            ayanami::interfaces::NavigationCallbacks::on_velocity_command(cmd_vel);
          }
        } else {
          error = result.error;
          message = result.message;
        }

        if (error != ErrorCode::NONE) {
          if (isPatienceExceeded(last_valid_time)) {
            error = ErrorCode::TIMEOUT;
            publishStopCommand();
            setState(INTERNAL_ERROR, true);
            return;
          }
          // Silent skip within patience: no publish this cycle (nav2
          // failure_tolerance waits quietly; the chassis command timeout is
          // the stop net for extended outages).
          setState(NO_LOCAL_CMD, false);
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
      case MAP_STALE:
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
  FeedbackFn feedback_callback;
  double last_speed{0.0};
  double distance_remaining{0.0};
  double avg_speed{0.0};
  double estimated_time_remaining{0.0};
  /// @brief Incremental nearest-point tracking state (see
  /// updateProgressMetrics): the index advances monotonically along the
  /// trajectory; a full scan runs once after each applied plan.
  std::size_t progress_index{0};
  bool progress_index_full_scan{false};
  /// @brief Throttle for the clamp warning (limit overshoots can recur every
  /// cycle; one warning per second is enough).
  std::chrono::steady_clock::time_point last_clamp_warn_at{};

  /// @brief nav2 failure_tolerance semantics: <0 (default) tolerates forever,
  /// 0 fails immediately, >0 is the tolerance window in seconds.
  std::chrono::duration<double> patience{-1};
  /// @brief Cross-track distance to the nearest path point of the cycle
  /// (meters; refreshed by updateProgressMetrics) and its fail threshold
  /// (<=0 disables the guard).
  double cross_track{0.0};
  double deviation_threshold{0.0};
  std::chrono::steady_clock::time_point start_time;
  /// @brief Last valid-command timestamp (patience window origin) and the
  /// first consecutive failed-safetyCheck timestamp (MAP_STALE window origin).
  std::chrono::steady_clock::time_point last_valid_time;
  std::optional<std::chrono::steady_clock::time_point> map_stale_since;

  /// @brief Update speed / distance_remaining / avg-speed metrics (call
  /// after the robot state query, before the goal check).
  ///
  /// Nearest-point tracking is INCREMENTAL along the trajectory: a full
  /// O(path) scan runs once per applied plan (progress_index_full_scan),
  /// then each cycle only scans a bounded window around the last index
  /// (small backtrack + bounded lookahead) and advances monotonically --
  /// the per-cycle cost no longer grows with the path length.
  void updateProgressMetrics(
      const geometry_msgs::PoseStamped& robot_pose,
      const geometry_msgs::TwistStamped& robot_velocity) {
    last_speed = std::hypot(robot_velocity.twist.linear.x(),
                            robot_velocity.twist.linear.y());
    if (!plan.poses.empty()) {
      const auto distance_to = [&](std::size_t index) {
        return (plan.poses[index].pose.position - robot_pose.pose.position)
            .norm();
      };
      cross_track = 0.0;
      if (progress_index_full_scan) {
        std::size_t nearest_index = 0;
        double best = std::numeric_limits<double>::max();
        for (std::size_t i = 0; i < plan.poses.size(); ++i) {
          const double d = distance_to(i);
          if (d < best) {
            best = d;
            nearest_index = i;
          }
        }
        cross_track = best;
        progress_index = nearest_index;
        progress_index_full_scan = false;
      } else {
        // Bounded window: a few points back (pose noise / small plan
        // shifts), a bounded run ahead (the robot advances along the path).
        const std::size_t back_step = 5;
        const std::size_t ahead_step = 100;
        const std::size_t window_begin =
            progress_index > back_step ? progress_index - back_step : 0;
        const std::size_t window_end =
            std::min(plan.poses.size(), progress_index + ahead_step + 1);
        std::size_t nearest_index = progress_index;
        double best = distance_to(progress_index);
        for (std::size_t i = window_begin; i < window_end; ++i) {
          const double d = distance_to(i);
          if (d < best) {
            best = d;
            nearest_index = i;
          }
        }
        cross_track = best;
        progress_index = nearest_index;
      }
    }
    double remaining = plan.poses.empty()
                           ? 0.0
                           : (plan.poses[progress_index].pose.position -
                              robot_pose.pose.position)
                                 .norm();
    for (std::size_t i = progress_index; i + 1 < plan.poses.size(); ++i) {
      remaining += (plan.poses[i + 1].pose.position -
                    plan.poses[i].pose.position)
                       .norm();
    }
    distance_remaining = remaining;
    // Exponential moving average of the measured speed drives the ETA.
    constexpr double k_alpha = 0.2;
    avg_speed = (1.0 - k_alpha) * avg_speed + k_alpha * last_speed;
    estimated_time_remaining =
        avg_speed > 1e-3 ? distance_remaining / avg_speed : 0.0;
  }

  /// @brief Publish the cycle feedback (no-op without an injected sink).
  void emitFeedback() {
    if (!feedback_callback) {
      return;
    }
    ayanami::actions::ExePathFeedback feedback;
    feedback.speed = last_speed;
    feedback.distance_remaining = distance_remaining;
    feedback.estimated_time_remaining = estimated_time_remaining;
    feedback.message = message;
    feedback_callback(feedback);
  }

  /// @brief Goal-change detection for checker resets: position/orientation
  /// only (header stamps change on every replan and must not count).
  /// Absolute tolerance (see poseClose): the relative isApprox never matched
  /// near the frame origin, so a near-origin goal would spuriously reset the
  /// goal/progress checkers on every replan.
  static bool sameGoalPose(const geometry_msgs::PoseStamped& lhs,
                           const geometry_msgs::PoseStamped& rhs) {
    return geometry_msgs::poseClose(lhs.pose, rhs.pose);
  }

  /// @brief Clamp a finite command into the velocity limits in place.
  ///
  /// Policy shift (replacing the old reject-and-stop): a command that merely
  /// exceeds a limit is CLAMPED (warn throttled), because stopping the robot
  /// for a 0.01 overshoot turns a third-party module's margin error into a
  /// patience timeout. NaN/inf components are NOT clamped -- they are
  /// rejected upstream by finiteVelocityCommand.
  void clampVelocityCommand(ayanami::geometry_msgs::TwistStamped& command) {
    const auto clamp = [](double value, double limit) {
      return value > limit ? limit : (value < -limit ? -limit : value);
    };
    auto& linear = command.twist.linear;
    auto& angular = command.twist.angular;
    const double clamped_x = clamp(linear.x(), max_linear_x);
    const double clamped_y = clamp(linear.y(), max_linear_y);
    const double clamped_w = clamp(angular.z(), max_angular_z);
    if (clamped_x != linear.x() || clamped_y != linear.y() ||
        clamped_w != angular.z()) {
      const auto now = std::chrono::steady_clock::now();
      if (now - last_clamp_warn_at >= std::chrono::seconds(1)) {
        last_clamp_warn_at = now;
        SPDLOG_LOGGER_WARN(
          ayanami::controllerLogger(),
          "ControllerExecution: velocity command exceeded the limits "
          "(vx={} vy={} wz={}, caps vx={} vy={} wz={}); clamped",
          linear.x(),
          linear.y(),
          angular.z(),
          max_linear_x,
          max_linear_y,
          max_angular_z);
      }
    }
    linear.x() = clamped_x;
    linear.y() = clamped_y;
    angular.z() = clamped_w;
  }

  /// @brief Finite check only (limits are handled by clamping): a non-finite
  /// command is unclampable garbage and is rejected (NO_VALID_COMMAND path).
  bool finiteVelocityCommand(
      const ayanami::geometry_msgs::TwistStamped& command) const {
    const auto& linear = command.twist.linear;
    const auto& angular = command.twist.angular;
    return std::isfinite(linear.x()) && std::isfinite(linear.y()) &&
           std::isfinite(linear.z()) && std::isfinite(angular.x()) &&
           std::isfinite(angular.y()) && std::isfinite(angular.z());
  }
};

} // namespace components
} // namespace ayanami
