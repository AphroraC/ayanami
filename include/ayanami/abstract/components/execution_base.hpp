#pragma once

#include "ayanami/messages/failure.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <string>

namespace ayanami {
namespace components {

/**
 * @brief Abstract execution base class (modeled after MBF
 * AbstractExecutionBase, components/).
 *
 * SINGLE execution model: the caller's thread drives the task inline via
 * runBlocking() (preRun()->run()->postRun(); the resident action worker
 * calls it once per goal) -- there is no execution thread, no spawn, no
 * join. The internal state machine (atomic<int>) + condition_variable
 * publish state updates to control threads (waitForStateUpdate is
 * event-driven; cancel/stop/setState all notify). Derived classes override
 * the run()/preRun()/postRun()/safetyCheck()/publishStopCommand() hooks
 * (map layer overrides, mirroring the MBF §2.2 pattern). Tests that need a
 * concurrent execution spawn their OWN std::thread around runBlocking().
 *
 * This class's internal state machine is independent of
 * actions::GoalStatus (5 states); only the action layer performs the
 * ExecutionState->GoalStatus mapping (active merged).
 */
class ExecutionBase {
public:
  ExecutionBase() = default;

  virtual ~ExecutionBase();

  /// @brief runTask fallback terminal value (does not collide with any
  /// derived state-machine enum): set uniformly by the runTask catch when
  /// an exception escapes run/preRun/postRun. Action driver polling must
  /// treat it as terminal (same for the runImplCommon pre-check and the
  /// PlannerAction custom loop), otherwise slot occupancy and resource
  /// protocols such as the recovery mutex will deadlock.
  static constexpr int k_aborted{250};

  /// @brief Whether currently in the fallback exception terminal state (see k_aborted).
  bool aborted() const { return state.load() == k_aborted; }

  /// @brief Execute INLINE on the caller's thread: the resident action
  /// worker calls this once per goal. Blocks until run() reaches a terminal
  /// state; the cancel/stop flags are reset for the run.
  void runBlocking();

  /// @brief Request run to exit (rigorous); run exits with the STOPPED state.
  virtual void stop();

  /// @brief Cancel the execution: set cancel_requested and call the virtual publishStopCommand (safe stop).
  virtual void cancel();

  /// @brief Wait on the condition for a state update; returns whether it was woken before the timeout.
  bool waitForStateUpdate(std::chrono::microseconds const& duration);

  /// @brief Current internal state (atomic read).
  int getState() const { return state.load(); }

  /// @brief Unified error code of the most recent module call (NONE=success).
  ErrorCode getError() const { return error; }

  /// @brief The most recent human-readable message.
  std::string getMessage() const { return message; }

protected:
  /// @brief Execution body (thread entry), overridden by derived classes.
  virtual void run() {}

  /// @brief Pre-execution hook (overridden by map-layer checkActivate etc.).
  virtual void preRun() {}

  /// @brief Post-execution hook (overridden by map-layer checkDeactivate etc.).
  virtual void postRun() {}

  /// @brief Safety check (overridden by map-layer isCurrent etc.). When false, derived classes should publish a stop command.
  virtual bool safetyCheck() { return true; }

  /// @brief Publish a stop command (empty by default; the controller overrides it to zero cmd_vel).
  virtual void publishStopCommand() {}

  /// @brief Update the state and notify the condition when signalling.
  void setState(int state, bool signalling = true) {
    this->state = state;
    if (signalling) condition.notify_all();
  }

  std::condition_variable condition;
  std::mutex mutex;
  std::atomic<bool> should_exit{false};
  std::atomic<bool> cancel_requested{false};
  std::atomic<int> state{0};
  ErrorCode error{ErrorCode::NONE};
  std::string message;

private:
  /// @brief Task body: preRun -> run -> postRun.
  void runTask();
};

} // namespace components
} // namespace ayanami
