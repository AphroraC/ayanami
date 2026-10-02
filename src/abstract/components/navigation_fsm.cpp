// FsmDriver implementation: the behavior tree's semantic successor.
//
// Every construct of the former navigation BT maps to plain code here:
// RecoveryNode -> retry budget + isRecoverable short-circuit; RecoverySequence
// -> rotating recovery cursor; RateController -> plan/smooth throttles;
// IsRobotProgressing -> position-history oscillation gate; GoalUpdated ->
// in-place goal revision; the pilot's run loop -> planLoop; FollowPath's
// async task -> controlLoop around the reused ControllerExecution.

#include "ayanami/abstract/components/navigation_fsm.hpp"

#include <ayanami/abstract/components/robot_state_controller.hpp>
#include <ayanami/utility/logging.hpp>

#include <spdlog/spdlog.h>

#include <algorithm>

namespace ayanami {
namespace components {

namespace {

/// @brief Plan-goal tolerance handed to the planner execution (the former
/// ComputePathToPose default; config surface lives with the host).
constexpr double k_plan_tolerance = 0.1;

/// @brief Recovery tick period floor (the execution's own default; kept
/// explicit so a host overriding it stays consistent).
constexpr std::chrono::milliseconds k_recovery_period{10};

std::chrono::microseconds tickPeriod(double frequency) {
  const double hz = frequency > 0.0 ? frequency : 20.0;
  return std::chrono::microseconds(
      static_cast<long long>(1e6 / hz));
}

FailureContext makeFailureContext(FailureStage stage, ErrorCode code,
                                  const std::string& module_id,
                                  const std::string& message) {
  FailureContext context;
  context.stage = stage;
  context.code = code;
  context.module_id = module_id;
  context.message = message;
  return context;
}

/// @brief Slot emission aggregate (status + family result).
template <typename Result>
actions::GoalResult<Result> makeGoalResult(actions::GoalStatus status,
                                           Result result) {
  actions::GoalResult<Result> out;
  out.status = status;
  out.result = std::move(result);
  return out;
}

}  // namespace

FsmDriver::FsmDriver(PlannerExecution::Ptr planner,
                     SmootherExecution::Ptr smoother,
                     ControllerExecution::Ptr controller,
                     std::vector<RecoveryExecution::Ptr> recoveries,
                     FsmDriverConfig config)
    : planner_(std::move(planner)),
      smoother_(std::move(smoother)),
      controller_(std::move(controller)),
      recoveries_(std::move(recoveries)),
      config_(config) {}

FsmDriver::~FsmDriver() { stop(); }

void FsmDriver::start() {
  if (running_.exchange(true)) {
    return;
  }
  setState(State::kWaitGoal);
  plan_thread_ = std::thread([this] { planLoop(); });
  control_thread_ = std::thread([this] { controlLoop(); });
}

void FsmDriver::stop() {
  running_.store(false);
  // Unblock any inline runBlocking the plan/control threads sit in: the
  // executions exit with STOPPED (should_exit wins over cancel semantics at
  // shutdown; no velocity is published by stop itself -- the host's stop
  // path already handled the chassis stop net).
  if (planner_) planner_->stop();
  if (smoother_) smoother_->stop();
  if (controller_) controller_->stop();
  for (auto& recovery : recoveries_) {
    if (recovery) recovery->stop();
  }
  goal_cv_.notify_all();
  follow_cv_.notify_all();
  if (plan_thread_.joinable()) {
    plan_thread_.join();
  }
  if (control_thread_.joinable()) {
    control_thread_.join();
  }
}

void FsmDriver::beginNavigation(const geometry_msgs::PoseStamped& goal) {
  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    pending_goal_ = goal;
  }
  goal_cv_.notify_all();
}

void FsmDriver::cancelNavigation() {
  cancel_requested_.store(true);
  // Non-blocking preemption of whatever is in flight: the plan loop observes
  // the flag at its next tick boundary and settles the attempt as CANCELED.
  if (planner_) planner_->cancel();
  if (smoother_) smoother_->cancel();
  if (controller_) controller_->cancel();
  for (auto& recovery : recoveries_) {
    if (recovery) recovery->cancel();
  }
}

// ---- plan thread ----

void FsmDriver::planLoop() {
  const auto period = tickPeriod(config_.fsm_frequency);
  while (running_.load()) {
    consumeGoalMailbox();
    switch (state()) {
      case State::kPlan:
        tickPlan();
        break;
      case State::kFollow:
        tickFollow();
        break;
      case State::kRecovering:
        tickRecovering();
        break;
      default:
        break;
    }
    std::this_thread::sleep_for(period);
  }
}

void FsmDriver::consumeGoalMailbox() {
  std::optional<geometry_msgs::PoseStamped> goal;
  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    if (pending_goal_) {
      goal = std::move(pending_goal_);
      pending_goal_.reset();
    }
  }
  if (!goal) {
    return;
  }
  if (attemptActive()) {
    // In-place revision (the former budget_reset_token + GoalUpdated
    // short-circuit): budget refresh, forced replan, controller NOT halted --
    // it keeps following the current path until the revised one lands via
    // in-place updatePlan.
    active_goal_ = *goal;
    goal_revision_.fetch_add(1, std::memory_order_acq_rel);
    retry_count_ = 0;
    failure_context_ = FailureContext{};
    force_replan_ = true;
    SPDLOG_LOGGER_INFO(
        btLogger(),
        "FSM: goal revised in place (revision {}); budget reset, replan forced",
        goalRevision());
    return;
  }
  startAttempt(*goal);
}

void FsmDriver::startAttempt(const geometry_msgs::PoseStamped& goal) {
  active_goal_ = goal;
  goal_revision_.fetch_add(1, std::memory_order_acq_rel);
  plan_version_.store(0, std::memory_order_release);
  retry_count_ = 0;
  recovery_cursor_ = 0;
  failure_context_ = FailureContext{};
  raw_path_ = nav_msgs::Path{};
  current_path_ = nav_msgs::Path{};
  raw_path_revision_ = 0;
  force_replan_ = true;
  plan_once_done_ = false;
  cancel_requested_.store(false);
  last_plan_at_ = {};
  last_smooth_at_ = {};
  position_history_.clear();
  // Execution reuse: a new attempt must not inherit the previous attempt's
  // checker baselines (the run loop only resets them on an actual goal
  // CHANGE, which a repeated same-goal attempt never produces).
  if (controller_) {
    controller_->resetCheckers();
  }
  setState(State::kPlan);
  SPDLOG_LOGGER_INFO(btLogger(),
                     "FSM: navigation attempt started (revision {}, goal "
                     "[{:.2f}, {:.2f}])",
                     goalRevision(), goal.pose.position.x(),
                     goal.pose.position.y());
}

void FsmDriver::tickPlan() {
  if (cancel_requested_.load()) {
    finishAttempt(actions::GoalStatus::CANCELED, "canceled by request");
    return;
  }
  if (!runPlanOnce(active_goal_)) {
    enterRecoveryOrTerminal();
    return;
  }
  force_replan_ = false;
  plan_once_done_ = true;
  last_plan_at_ = std::chrono::steady_clock::now();
  last_smooth_at_ = last_plan_at_;
  kickFollow();
  setState(State::kFollow);
}

bool FsmDriver::runPlanOnce(const geometry_msgs::PoseStamped& goal) {
  // 1) Plan (an empty start is resolved to the latest robot pose by the
  // execution; the module always receives a fully valid start).
  {
    actions::GetPathGoal plan_goal;
    plan_goal.goal = goal;
    plan_goal.tolerance = k_plan_tolerance;
    plan_goal.session_id = goalRevision();
    plan_goal.plan_version = planVersion();
    ayanami::interfaces::NavigationCallbacks::on_get_path_goal(plan_goal);
  }
  planner_->prepareGoal(geometry_msgs::PoseStamped{}, goal, k_plan_tolerance);
  planner_->runBlocking();
  if (planner_->getState() != PlannerExecution::FOUND_PLAN) {
    failure_context_ = makeFailureContext(FailureStage::Planner,
                                          planner_->getError(), "",
                                          planner_->getMessage());
    SPDLOG_LOGGER_WARN(btLogger(),
                       "FSM: planning failed (code {}): {}",
                       static_cast<int>(planner_->getError()),
                       planner_->getMessage());
    actions::GetPathResult plan_result;
    plan_result.error = planner_->getError();
    plan_result.message = planner_->getMessage();
    plan_result.session_id = goalRevision();
    plan_result.plan_version = planVersion();
    const auto plan_failure =
        makeGoalResult(actions::GoalStatus::ABORTED, std::move(plan_result));
    ayanami::interfaces::NavigationCallbacks::on_get_path_result(plan_failure);
    return false;
  }
  raw_path_ = planner_->takePlan();
  ++raw_path_revision_;
  {
    actions::GetPathResult plan_result;
    plan_result.error = ErrorCode::NONE;
    plan_result.message = "planned";
    plan_result.path = raw_path_;
    plan_result.session_id = goalRevision();
    plan_result.plan_version = planVersion();
    const auto plan_success =
        makeGoalResult(actions::GoalStatus::SUCCEEDED, std::move(plan_result));
    ayanami::interfaces::NavigationCallbacks::on_get_path_result(plan_success);
  }
  if (!pathValid(raw_path_)) {
    failure_context_ =
        makeFailureContext(FailureStage::Planner, ErrorCode::PATH_INVALID,
                           "", "raw path is not valid on the current map");
    return false;
  }

  // 2) Smooth (iterative refinement restarts from the raw path here; the
  // FOLLOW-tick refinement takes the current path as its input instead).
  smoother_->prepareGoal(raw_path_, config_.max_smooth_duration);
  smoother_->runBlocking();
  nav_msgs::Path smoothed;
  if (smoother_->getState() == SmootherExecution::SMOOTHED_PATH) {
    smoothed = smoother_->takeSmoothedPath();
  } else {
    // DegradeToRawPath: smoothing is an enhancement -- a failed pass falls
    // back to the raw path instead of failing the plan.
    SPDLOG_LOGGER_WARN(btLogger(),
                       "FSM: smoothing failed (code {}: {}), degrading to the "
                       "raw path",
                       static_cast<int>(smoother_->getError()),
                       smoother_->getMessage());
    smoothed = raw_path_;
  }
  {
    actions::SmoothPathResult smooth_result;
    smooth_result.error = ErrorCode::NONE;
    smooth_result.message = "smoothed";
    smooth_result.smoothed_path = smoothed;
    smooth_result.session_id = goalRevision();
    smooth_result.plan_version = planVersion();
    const auto smooth_out =
        makeGoalResult(actions::GoalStatus::SUCCEEDED, std::move(smooth_result));
    ayanami::interfaces::NavigationCallbacks::on_smooth_path_result(smooth_out);
  }
  if (!pathValid(smoothed)) {
    if (!pathValid(raw_path_)) {
      failure_context_ = makeFailureContext(
          FailureStage::Smoother, ErrorCode::PATH_INVALID, "",
          "smoothed and raw paths are both invalid on the current map");
      return false;
    }
    SPDLOG_LOGGER_WARN(btLogger(),
                       "FSM: smoothed path invalid, keeping the raw path");
    smoothed = raw_path_;
  }

  current_path_ = std::move(smoothed);
  plan_version_.fetch_add(1, std::memory_order_acq_rel);
  return true;
}

void FsmDriver::tickFollow() {
  if (cancel_requested_.load()) {
    preemptController();
    finishAttempt(actions::GoalStatus::CANCELED, "canceled by request");
    return;
  }

  // Oscillation gate (the former IsRobotProgressing): progress is lost when
  // the robot sits within oscillation_distance of where it was
  // oscillation_timeout seconds ago.
  notePositionForOscillation(RobotStateController::instance().getRobotPose());
  if (oscillationDetected()) {
    failure_context_ = makeFailureContext(
        FailureStage::Navigation, ErrorCode::NO_PROGRESS, "",
        "oscillation gate: no displacement within the timeout window");
    preemptController();
    enterRecoveryOrTerminal();
    return;
  }

  // Wait one tick for the controller to reach a terminal state.
  bool controller_done = false;
  bool controller_arrived = false;
  {
    std::unique_lock<std::mutex> lock(follow_mutex_);
    follow_cv_.wait_for(lock, tickPeriod(config_.fsm_frequency),
                        [&] { return follow_done_ || !running_.load(); });
    controller_done = follow_done_;
    controller_arrived = follow_succeeded_;
  }
  if (!running_.load()) {
    return;
  }
  if (controller_done) {
    follow_done_ = false;
    {
      actions::ExePathResult exe_result;
      exe_result.error = controller_->getError();
      exe_result.message = controller_->getMessage();
      exe_result.reached = controller_arrived;
      exe_result.session_id = goalRevision();
      exe_result.plan_version = planVersion();
      const auto exe_out = makeGoalResult(
          controller_arrived ? actions::GoalStatus::SUCCEEDED
                             : actions::GoalStatus::ABORTED,
          std::move(exe_result));
      ayanami::interfaces::NavigationCallbacks::on_exe_path_result(exe_out);
    }
    if (controller_arrived) {
      finishAttempt(actions::GoalStatus::SUCCEEDED, "reached the goal");
      return;
    }
    failure_context_ = makeFailureContext(
        FailureStage::Controller, controller_->getError(), "",
        controller_->getMessage());
    SPDLOG_LOGGER_WARN(btLogger(),
                       "FSM: follow failed (code {}): {}",
                       static_cast<int>(controller_->getError()),
                       controller_->getMessage());
    preemptController();  // terminal already; cancel() is idempotent
    enterRecoveryOrTerminal();
    return;
  }

  // Still following: supervision (throttled replanning, validity loss,
  // iterative re-smoothing). A goal revision forces the replan immediately
  // (the budget was already reset by the mailbox consumer).
  const auto now = std::chrono::steady_clock::now();
  const bool planner_due =
      config_.planner_frequency > 0.0 &&
      std::chrono::duration<double>(now - last_plan_at_).count() >=
          1.0 / config_.planner_frequency;
  const bool validity_lost = !pathValid(current_path_);
  if (force_replan_ || validity_lost || planner_due) {
    if (validity_lost) {
      SPDLOG_LOGGER_WARN(
          btLogger(), "FSM: current path turned invalid, replanning now");
    }
    if (!runPlanOnce(active_goal_)) {
      preemptController();
      enterRecoveryOrTerminal();
      return;
    }
    force_replan_ = false;
    last_plan_at_ = std::chrono::steady_clock::now();
    last_smooth_at_ = last_plan_at_;
    // In-place update (no halt, no zero velocity): the running controller
    // consumes path+version on its next cycle.
    controller_->updatePlan(current_path_, plan_version_.load());
    return;
  }

  // Iterative re-smoothing (the smoother branch's refinement): input is the
  // CURRENT smoothed path; only a fresh raw path (runPlanOnce) restarts from
  // raw. A rejected candidate keeps the current path.
  const bool smoother_due =
      config_.smoother_frequency > 0.0 &&
      std::chrono::duration<double>(now - last_smooth_at_).count() >=
          1.0 / config_.smoother_frequency;
  if (smoother_due) {
    last_smooth_at_ = now;
    smoother_->prepareGoal(current_path_, config_.max_smooth_duration);
    smoother_->runBlocking();
    if (smoother_->getState() == SmootherExecution::SMOOTHED_PATH) {
      nav_msgs::Path candidate = smoother_->takeSmoothedPath();
      if (pathValid(candidate) &&
          !candidate.poses.empty() && !current_path_.poses.empty()) {
        current_path_ = std::move(candidate);
        plan_version_.fetch_add(1, std::memory_order_acq_rel);
        controller_->updatePlan(current_path_, plan_version_.load());
      }
    } else {
      // Refinement failed: keep following the current path (an enhancement
      // pass failing is not a navigation failure).
      SPDLOG_LOGGER_WARN(btLogger(),
                         "FSM: re-smoothing failed (code {}: {}), keeping the "
                         "current path",
                         static_cast<int>(smoother_->getError()),
                         smoother_->getMessage());
    }
  }
}

void FsmDriver::tickRecovering() {
  if (cancel_requested_.load()) {
    finishAttempt(actions::GoalStatus::CANCELED, "canceled by request");
    return;
  }
  // Rotate first (the RecoverySequence cursor semantics: consecutive
  // recoveries cycle instead of repeating the first one).
  RecoveryExecution::Ptr recovery = recoveries_[recovery_cursor_];
  recovery_cursor_ = (recovery_cursor_ + 1) % recoveries_.size();
  // Motion recoveries need the robot stopped; preempting the controller at
  // recovery entry is the recovery_exclusive discipline.
  preemptController();
  recovery->setBehaviorParams(RecoveryBehaviorParams{});
  {
    actions::RecoveryGoal recovery_goal;
    recovery_goal.cause = failure_context_;
    recovery_goal.session_id = goalRevision();
    recovery_goal.plan_version = planVersion();
    ayanami::interfaces::NavigationCallbacks::on_recovery_goal(recovery_goal);
  }
  recovery->runBlocking();
  {
    actions::RecoveryResult recovery_result;
    recovery_result.error = recovery->getError();
    recovery_result.message = recovery->getMessage();
    recovery_result.session_id = goalRevision();
    recovery_result.plan_version = planVersion();
    const auto recovery_out = makeGoalResult(
        recovery->getState() == RecoveryExecution::RECOVERY_DONE
            ? actions::GoalStatus::SUCCEEDED
            : actions::GoalStatus::ABORTED,
        std::move(recovery_result));
    ayanami::interfaces::NavigationCallbacks::on_recovery_result(recovery_out);
  }
  if (recovery->getState() == RecoveryExecution::RECOVERY_DONE) {
    ++retry_count_;
    failure_context_ = FailureContext{};
    force_replan_ = true;
    SPDLOG_LOGGER_INFO(btLogger(),
                       "FSM: recovery done ({} of {} retries used); replanning",
                       retry_count_, config_.max_nav_retries);
    setState(State::kPlan);
    return;
  }
  // Recovery failure/cancel/timeout: the BT's RecoveryNode maps a failed
  // recovery child straight to overall FAILURE.
  failure_context_ = makeFailureContext(FailureStage::Recovery,
                                        recovery->getError(), "",
                                        recovery->getMessage());
  SPDLOG_LOGGER_WARN(btLogger(), "FSM: recovery failed (code {}): {}",
                     static_cast<int>(recovery->getError()),
                     recovery->getMessage());
  finishAttempt(actions::GoalStatus::ABORTED, "recovery failed");
}

void FsmDriver::enterRecoveryOrTerminal() {
  if (!isRecoverable(failure_context_.code)) {
    finishAttempt(actions::GoalStatus::ABORTED,
                  "non-recoverable failure (no recovery can fix it)");
    return;
  }
  if (recoveries_.empty() || retry_count_ >= config_.max_nav_retries) {
    finishAttempt(actions::GoalStatus::ABORTED,
                  recoveries_.empty() ? "no recovery configured"
                                      : "retry budget exhausted");
    return;
  }
  setState(State::kRecovering);
}

// ---- control thread ----

void FsmDriver::controlLoop() {
  while (running_.load()) {
    {
      std::unique_lock<std::mutex> lock(follow_mutex_);
      follow_cv_.wait(lock, [&] { return follow_request_ || !running_.load(); });
      if (!running_.load()) {
        break;
      }
      follow_request_ = false;
      follow_done_ = false;
    }
    // Blocks until the controller reaches a terminal state; a preempt
    // (cancel) lands as the CANCELED terminal and unblocks the same way.
    controller_->runBlocking();
    {
      std::lock_guard<std::mutex> lock(follow_mutex_);
      follow_succeeded_ =
          controller_->getState() == ControllerExecution::ARRIVED_GOAL;
      follow_done_ = true;
    }
    follow_cv_.notify_all();
  }
}

// ---- helpers ----

void FsmDriver::kickFollow() {
  {
    std::lock_guard<std::mutex> lock(follow_mutex_);
    controller_->setNewPlan(current_path_);
    controller_->setPlanVersion(plan_version_.load());
    follow_request_ = true;
    follow_done_ = false;
    follow_succeeded_ = false;
  }
  follow_cv_.notify_all();
}

void FsmDriver::preemptController() {
  if (controller_) {
    // cancel() publishes exactly one zeroed stop command and lets the run
    // wind down on the control thread; it is a no-op state-wise when the
    // execution already sits in a terminal state.
    controller_->cancel();
  }
}

bool FsmDriver::pathValid(const nav_msgs::Path& path) const {
  if (path.poses.empty()) {
    return false;
  }
  return path_validator_ ? path_validator_(path) : true;
}

void FsmDriver::finishAttempt(actions::GoalStatus status,
                              const std::string& why) {
  preemptController();
  actions::NavigationResult result;
  result.status = status;
  result.context = failure_context_;
  ayanami::interfaces::NavigationCallbacks::on_navigation_result(result);
  SPDLOG_LOGGER_INFO(btLogger(), "FSM: attempt finished ({}) -- {}",
                     static_cast<int>(status), why);
  // Rest in the terminal state until the next goal consumes it (the pilot's
  // resident model: the terminal outcome stays observable; startAttempt is
  // entered from every non-active state).
  switch (status) {
    case actions::GoalStatus::SUCCEEDED:
      setState(State::kSucceeded);
      break;
    case actions::GoalStatus::CANCELED:
      setState(State::kCanceled);
      break;
    default:
      setState(State::kAborted);
      break;
  }
}

void FsmDriver::setState(State next) {
  state_.store(next, std::memory_order_release);
}

void FsmDriver::notePositionForOscillation(
    const geometry_msgs::PoseStamped& pose) {
  const auto now = std::chrono::steady_clock::now();
  position_history_.emplace_back(
      now, Eigen::Vector2d(pose.pose.position.x(), pose.pose.position.y()));
  const auto window = std::chrono::duration<double>(config_.oscillation_timeout);
  while (!position_history_.empty() &&
         std::chrono::duration<double>(now - position_history_.front().first) >
             window) {
    position_history_.pop_front();
  }
}

bool FsmDriver::oscillationDetected() const {
  if (config_.oscillation_timeout <= 0.0 || config_.oscillation_distance <= 0.0 ||
      position_history_.empty()) {
    return false;
  }
  const auto now = std::chrono::steady_clock::now();
  const auto window = std::chrono::duration<double>(config_.oscillation_timeout);
  const Eigen::Vector2d& newest = position_history_.back().second;
  for (const auto& entry : position_history_) {
    if (std::chrono::duration<double>(now - entry.first) >= window &&
        (newest - entry.second).norm() <= config_.oscillation_distance) {
      return true;
    }
  }
  return false;
}

}  // namespace components
}  // namespace ayanami
