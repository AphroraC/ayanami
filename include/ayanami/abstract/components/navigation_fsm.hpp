#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "ayanami/abstract/components/controller_execution.hpp"
#include "ayanami/abstract/components/planner_execution.hpp"
#include "ayanami/abstract/components/recovery_execution.hpp"
#include "ayanami/abstract/components/smoother_execution.hpp"
#include "ayanami/abstract/core/abstract_planner.hpp"
#include "ayanami/interfaces/callbacks.hpp"
#include "ayanami/messages/actions.hpp"
#include "ayanami/messages/failure.hpp"
#include "ayanami/messages/geometry_msgs.hpp"
#include "ayanami/messages/nav_msgs.hpp"
#include "ayanami/types.hpp"

namespace ayanami {
namespace components {

/// @brief Tunables of the FSM driver (the semantic successor of the BT's
/// blackboard-injected policy keys; all of them are configure-time).
struct FsmDriverConfig {
  /// @brief FSM tick rate on the plan thread (Hz).
  double fsm_frequency{20.0};
  /// @brief Periodic replanning rate while FOLLOWING (Hz; <=0 = replan only
  /// when the current path turns invalid -- the IsPathValid trigger).
  double planner_frequency{5.0};
  /// @brief Iterative re-smoothing rate while FOLLOWING (Hz; <=0 = smooth
  /// once per raw path). Iterative refinement keeps the CURRENT smoothed
  /// path as its input; a changed raw path restarts from the raw path.
  double smoother_frequency{20.0};
  /// @brief Invalidity grace window (seconds): while the current path
  /// fails isPathValid, the SMOOTHER owns the repair (the re-smooth is
  /// forced immediately every tick); only after the path has stayed
  /// invalid this long does the planner re-plan. 0 = replan immediately
  /// on invalidity (the previous behaviour). The plan-once contract:
  /// planner_frequency < 0 + a positive grace leaves the planner out of
  /// the loop entirely during normal following.
  double replan_grace{0.0};
  /// @brief Advisory time budget per smoothing pass (seconds).
  double max_smooth_duration{2.0};
  /// @brief Outer recovery budget per attempt (goal revisions reset it).
  uint32_t max_nav_retries{6};
  /// @brief Navigation-level oscillation gate (MBF semantics): progress is
  /// declared lost when the robot stays within oscillation_distance of its
  /// position oscillation_timeout seconds ago while an attempt is active.
  double oscillation_distance{0.1};
  double oscillation_timeout{5.0};
};

/**
 * @brief Single-attempt FSM navigation driver (the BT's replacement).
 *
 * Thread model (the pilot's model, translated): a latest-wins goal mailbox
 * wakes the resident PLAN thread; a resident CONTROL thread drives the
 * injected ControllerExecution to its terminal state while the plan thread
 * supervises (validity probes, throttled replanning / iterative
 * re-smoothing, in-place plan updates, oscillation gate). Planner and
 * smoother executions run INLINE on the plan thread between ticks.
 *
 * Semantic carry-over from the former behavior tree (each maps to one BT
 * construct; all of them are now plain code):
 * - retry budget + goal-revision reset (RecoveryNode + budget_reset_token);
 * - non-recoverable short-circuit (isRecoverable classification skips the
 *   recovery branch entirely -- there is no point burning retries on an
 *   invalid goal or a lost transform);
 * - recovery rotation cursor (RecoverySequence: clear->spin->wait->backup
 *   in the tree; here a rotating index over the injected recovery list);
 * - in-place goal/plan revision (goal_revision + controller updatePlan --
 *   no halt, no zero-velocity, the running controller keeps following its
 *   path until the revised one lands);
 * - planner/smoother rate throttles (RateController);
 * - oscillation gate (IsRobotProgressing: a deque of recent positions);
 * - one navigation result per attempt (on_navigation_result).
 *
 * The driver holds no map and no module loading: the host injects one
 * execution per family (planner / smoother / controller) plus the ordered
 * recovery list, each already assembled with its module, checkers, limits
 * and patience by the NavigationServerBase-derived host.
 */
class FsmDriver {
 public:
  enum class State : std::uint8_t {
    kInit = 0,
    kWaitGoal,
    kPlan,
    kFollow,
    kRecovering,
    kSucceeded,
    kAborted,
    kCanceled,
  };

  /// @param planner / smoother / controller family executions (inline
  /// runBlocking use; the controller runs on the control thread).
  /// @param recoveries ordered recovery rotation (may be empty: failures
  /// then immediately exhaust the budget).
  FsmDriver(PlannerExecution::Ptr planner,
            SmootherExecution::Ptr smoother,
            ControllerExecution::Ptr controller,
            std::vector<RecoveryExecution::Ptr> recoveries,
            FsmDriverConfig config);
  ~FsmDriver();

  FsmDriver(const FsmDriver&) = delete;
  FsmDriver& operator=(const FsmDriver&) = delete;

  /// @brief Start the resident threads (idempotent).
  void start();

  /// @brief Stop: cancel in-flight executions, join both threads
  /// (idempotent; the destructor calls it).
  void stop();

  /// @brief Mailbox delivery of a navigation goal (latest-wins). The first
  /// goal of an idle driver starts an attempt; a goal arriving while an
  /// attempt is in flight is an IN-PLACE revision (goal_revision advance +
  /// budget reset + forced replan; the running controller is not halted).
  void beginNavigation(const geometry_msgs::PoseStamped& goal);

  /// @brief Request cancellation of the active attempt (idempotent; the
  /// terminal result is CANCELED).
  void cancelNavigation();

  /// @brief Inject the path-validity probe (the planner module's
  /// isPathValid, wired by the host -- the planner is the map reader).
  /// Default (unset): a path is valid iff non-empty.
  void setPathValidator(
      std::function<bool(const ayanami::nav_msgs::Path&)> validator) {
    path_validator_ = std::move(validator);
  }

  // ---- observation (tests / telemetry) ----
  State state() const { return state_.load(std::memory_order_relaxed); }
  bool attemptActive() const {
    const State s = state();
    return s == State::kPlan || s == State::kFollow || s == State::kRecovering;
  }
  std::uint64_t goalRevision() const {
    return goal_revision_.load(std::memory_order_relaxed);
  }
  std::uint64_t planVersion() const {
    return plan_version_.load(std::memory_order_relaxed);
  }
  const FsmDriverConfig& config() const { return config_; }

 private:
  // ---- plan thread (the FSM tick loop) ----
  void planLoop();

  /// @brief Drain the goal mailbox: a fresh attempt on an idle driver, an
  /// in-place revision (budget reset + forced replan, controller untouched)
  /// while an attempt is active.
  void consumeGoalMailbox();

  /// @brief Non-consuming mailbox probe (the recovery-branch interrupt
  /// predicate): true while a fresh goal is waiting to be consumed.
  bool goalPending();

  /// @brief Reset the per-attempt state and enter kPlan.
  void startAttempt(const geometry_msgs::PoseStamped& goal);

  /// @brief One kPlan tick: plan -> validity -> smooth (with the
  /// DegradeToRawPath fallback) -> validity; kick the controller on success.
  void tickPlan();

  /// @brief One planning pass. Returns true when a followable path landed
  /// in current_path_; failure context is written to failure_context_.
  bool runPlanOnce(const geometry_msgs::PoseStamped& goal);

  /// @brief One kFollow tick: terminal check, oscillation gate, throttled
  /// replan / validity-loss replan / iterative re-smoothing with in-place
  /// controller updates.
  void tickFollow();

  /// @brief One kRecovering tick: rotate the cursor, run the recovery
  /// inline; success replans, failure aborts the attempt.
  void tickRecovering();

  /// @brief Recovery decision after a failure: non-recoverable codes abort
  /// immediately, an empty list or an exhausted budget aborts, otherwise
  /// enter kRecovering.
  void enterRecoveryOrTerminal();

  /// @brief Follow-context failure entry: account the net displacement since
  /// the last recovery entry (the futility guard) and either abort a
  /// confirmed stuck loop or hand over to enterRecoveryOrTerminal.
  void enterRecoveryOrTerminalAfterFollowFailure();

  /// @brief Futility accounting: compare the robot position against the
  /// last recovery entry; a cycle with < 5 cm of displacement increments
  /// stuck_cycles_, real movement resets it.
  void accountRecoveryCycleProgress();

  // ---- control thread ----
  void controlLoop();

  // ---- helpers ----
  /// @brief Hand current_path_ to the control thread and start one
  /// controller run (setNewPlan + version + follow handshake).
  void kickFollow();

  /// @brief Non-blocking controller preemption (one zeroed stop pulse; the
  /// run winds down on the control thread).
  void preemptController();

  /// @brief Path validity via the injected probe (default: non-empty).
  bool pathValid(const ayanami::nav_msgs::Path& path) const;

  /// @brief Emit the session-level navigation result once and settle at
  /// kWaitGoal.
  void finishAttempt(actions::GoalStatus status, const std::string& why);

  void setState(State next);

  /// @brief Push the latest robot position into the oscillation history and
  /// age out entries older than oscillation_timeout.
  void notePositionForOscillation(const geometry_msgs::PoseStamped& pose);

  /// @brief True when the robot stayed within oscillation_distance of its
  /// position from oscillation_timeout seconds ago.
  bool oscillationDetected() const;

  // ---- injected executions ----
  PlannerExecution::Ptr planner_;
  SmootherExecution::Ptr smoother_;
  ControllerExecution::Ptr controller_;
  std::vector<RecoveryExecution::Ptr> recoveries_;
  /// @brief Path-validity probe (planner module's isPathValid; unset =
  /// non-empty check).
  std::function<bool(const ayanami::nav_msgs::Path&)> path_validator_;

  FsmDriverConfig config_;
  std::atomic<State> state_{State::kInit};

  // ---- goal mailbox ----
  std::mutex goal_mutex_;
  std::condition_variable goal_cv_;
  std::optional<geometry_msgs::PoseStamped> pending_goal_;
  geometry_msgs::PoseStamped active_goal_;
  std::atomic<std::uint64_t> goal_revision_{0};
  std::atomic<std::uint64_t> plan_version_{0};
  bool force_replan_{false};

  // ---- follow bridge ----
  std::mutex follow_mutex_;
  std::condition_variable follow_cv_;
  bool follow_request_{false};
  bool follow_done_{true};
  bool follow_succeeded_{false};

  // ---- per-attempt state ----
  nav_msgs::Path raw_path_;
  nav_msgs::Path current_path_;
  std::uint64_t raw_path_revision_{0};
  FailureContext failure_context_;
  uint32_t retry_count_{0};
  std::size_t recovery_cursor_{0};
  /// @brief Robot position at the last recovery entry (net-displacement
  /// futility guard): a recovery cycle that ends in another follow failure
  /// WITHOUT the robot having moved is a stuck loop the placeholder
  /// recovery cannot break -- two such cycles abort the attempt with a
  /// diagnosis instead of burning the whole budget.
  Eigen::Vector2d recovery_entry_pos_{Eigen::Vector2d::Zero()};
  bool recovery_entry_valid_{false};
  uint32_t stuck_cycles_{0};
  std::chrono::steady_clock::time_point last_plan_at_{};
  std::chrono::steady_clock::time_point last_smooth_at_{};
  /// @brief When the current path first failed isPathValid (reset on
  /// recovery); drives the replan_grace window.
  std::optional<std::chrono::steady_clock::time_point> invalid_since_;
  bool plan_once_done_{false};

  // ---- oscillation gate ----
  std::deque<std::pair<std::chrono::steady_clock::time_point,
                       Eigen::Vector2d>>
      position_history_;

  std::atomic<bool> cancel_requested_{false};
  std::atomic<bool> running_{false};
  std::thread plan_thread_;
  std::thread control_thread_;
};

}  // namespace components
}  // namespace ayanami
