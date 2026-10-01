#pragma once

#include "ayanami/abstract/components/action_server.hpp"

#include <spdlog/spdlog.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <utility>
#include <vector>

namespace ayanami {
namespace components {

/**
 * @brief Action base: one RESIDENT worker thread + one reused execution,
 * running goals strictly sequentially (latest-wins mailbox).
 *
 * Single-navigation discipline: each action family runs at most one task at
 * a time, so there is exactly one worker thread per ACTION (one per family
 * module) and one execution object per action, created lazily on the first
 * goal and reused across tasks (no driver-thread/execution-thread pair per
 * task; the worker calls ExecutionBase::runBlocking inline, mirroring the
 * nav2 controller_server single-loop model).
 *
 * Goal flow: submitGoal() parks the goal in a one-slot latest-wins
 * mailbox (a goal dropped from the slot is settled CANCELED immediately);
 * an in-flight execution is cooperatively canceled so the worker reaches
 * the newest goal promptly -- its own goal then settles CANCELED through
 * the normal finish path. The worker pops the newest goal, hands it to
 * runGoal() (derived: prepare execution -> runBlocking -> map terminal
 * state -> publish result), and only then takes the next goal: strict
 * serialization, the module never re-enters.
 *
 * Cancellation: TaskHandle::cancel / cancelGoal(id) forward to the
 * in-flight execution directly (a zeroed stop command is published
 * synchronously by the execution's cancel()); cancelAll() additionally
 * waits until the worker is idle (synchronous stop/destructor path).
 * In-place updates (updatePlan/updateGoal) address the in-flight execution
 * by goal id via executionForGoal().
 *
 * Stall policy (the watchdog second line): ExecutionBase has no execution
 * thread, so a module call that never returns occupies the worker forever
 * and no cancel can interrupt it. cancelAll() therefore waits BOUNDED
 * (kStallWait); on expiry the action is marked stalled -- further goals are
 * settled CANCELED without dispatch -- and stopWorker() moves the stuck
 * thread into a process-lifetime leak container instead of joining (a
 * joinable std::thread destructor would terminate; joining would hang
 * forever). Residual risk, documented: if the stuck module EVER returns
 * after the action object was destroyed, the leaked thread touches freed
 * memory -- treated as a module violating the cancel contract in an already
 * fatal situation; process exit reclaims the thread.
 */
template <typename Goal, typename Execution>
class ActionBase {
 public:
  using GoalHandleType = GoalHandle<Goal>;
  using ExecutionPtr = typename Execution::Ptr;

  ActionBase() = default;
  virtual ~ActionBase() { stopWorker(); }

  /// @brief Submit one goal (latest-wins mailbox, see class comment).
  void submitGoal(const GoalHandleType& goal_handle) {
    if (stalled_.load()) {
      // The worker is wedged inside a module call that ignored cancel: no
      // new goal can ever run. Settle immediately through the derived
      // CANCELED publisher so the TaskHandle never hangs.
      finishPendingGoal(goal_handle);
      return;
    }
    ensureWorker();
    std::optional<GoalHandleType> dropped;
    ExecutionPtr exec;
    {
      std::lock_guard<std::mutex> lock(task_mutex);
      if (pending_goal_) {
        dropped = *pending_goal_;
      }
      pending_goal_ = goal_handle;
      if (in_flight_) {
        exec = execution_;
      }
    }
    if (dropped) {
      // Settled on the caller thread while the object is fully alive.
      finishPendingGoal(*dropped);
    }
    if (exec) {
      exec->cancel();
    }
    worker_cv.notify_all();
  }

  /// @brief Cancel the in-flight execution (forwards to the module cancel).
  void cancel() { preemptAll(); }

  /// @brief Cancel a goal: a still-pending goal is settled CANCELED before
  /// its driver ever started; the in-flight goal's execution is canceled
  /// (its result publishes through the normal finish path on the worker).
  /// Returns false when the goal is neither pending nor in flight.
  bool cancelGoal(uint64_t goal_id) {
    std::optional<GoalHandleType> dropped;
    ExecutionPtr exec;
    {
      std::lock_guard<std::mutex> lock(task_mutex);
      if (pending_goal_ && pending_goal_->id == goal_id) {
        dropped = *pending_goal_;
        pending_goal_.reset();
      } else if (in_flight_ && in_flight_goal_id == goal_id) {
        exec = execution_;
      }
    }
    if (dropped) {
      finishPendingGoal(*dropped);
      return true;
    }
    if (exec) {
      exec->cancel();
      return true;
    }
    return false;
  }

  /// @brief Non-blocking preemption: settle the pending goal, cancel the
  /// in-flight execution and return. Used by runtime paths (halt / recovery
  /// start) that must not block on a slow module call.
  void preemptAll() {
    std::optional<GoalHandleType> dropped;
    ExecutionPtr exec;
    {
      std::lock_guard<std::mutex> lock(task_mutex);
      if (pending_goal_) {
        dropped = *pending_goal_;
        pending_goal_.reset();
      }
      if (in_flight_) {
        exec = execution_;
      }
    }
    if (dropped) {
      finishPendingGoal(*dropped);
    }
    if (exec) {
      exec->cancel();
    }
  }

  /// @brief Bound on how long cancelAll() tolerates an unresponsive module
  /// (seconds). Expiry marks the action stalled -- see the class comment.
  static constexpr std::chrono::seconds kStallWait{2};

  /// @brief Cancel everything and WAIT (bounded) for the in-flight goal to
  /// finish (destructor / stop path where module lifetime requires a
  /// synchronous shutdown). A module that ignores cancel keeps the worker
  /// occupied: after kStallWait the action is marked stalled and this
  /// returns, so one broken module cannot hang the whole server shutdown.
  void cancelAll() {
    preemptAll();
    std::unique_lock<std::mutex> lock(task_mutex);
    const bool idle =
        idle_cv.wait_for(lock, kStallWait, [this] { return !in_flight_; });
    if (!idle) {
      stalled_.store(true);
      spdlog::critical(
          "ActionBase: cancelAll timed out after {} s -- a module ignored "
          "cancel; the action is marked stalled (further goals are rejected) "
          "and its worker thread will be leaked at shutdown",
          kStallWait.count());
    }
  }

  bool isRunning() const {
    std::lock_guard<std::mutex> lock(task_mutex);
    return in_flight_;
  }

 protected:
  /// @brief Run one goal to its published terminal result on the worker
  /// thread. Derived classes: setActive -> prepare the (reused) execution
  /// for this goal -> execution.runBlocking() -> map the terminal state ->
  /// publish the result (and clear per-goal bookkeeping).
  virtual void runGoal(const GoalHandleType& goal_handle,
                       Execution& execution) = 0;

  /// @brief Settle a goal that was dropped from the mailbox (replaced by a
  /// newer goal or canceled) before its driver ever started. Derived
  /// classes publish a CANCELED result so the TaskHandle never hangs.
  virtual void finishPendingGoal(const GoalHandleType& goal_handle) {
    (void)goal_handle;
  }

  /// @brief Driver-task exception fallback: publish ABORTED to guarantee
  /// TaskHandle completion (not publishing would hang waiters, and resource
  /// protocols such as the recovery mutex released in onComplete would
  /// deadlock as well). Called when runGoal throws.
  virtual void abortGoal(const GoalHandleType& goal_handle) {
    (void)goal_handle;
  }

  /// @brief Find the in-flight execution by goal_id (for in-place updates,
  /// e.g. controller path/version updates, planner goal updates). Returns
  /// nullptr when the goal is not the one in flight.
  ExecutionPtr executionForGoal(uint64_t goal_id) const {
    std::lock_guard<std::mutex> lock(task_mutex);
    if (in_flight_ && in_flight_goal_id == goal_id) {
      return execution_;
    }
    return ExecutionPtr{};
  }

  /// @brief Lazily create the single reused execution (first goal).
  ExecutionPtr ensureExecution() {
    std::lock_guard<std::mutex> lock(task_mutex);
    if (!execution_) {
      execution_ = createExecution();
    }
    return execution_;
  }

  /// @brief Create the execution object (derived: map-aware variants).
  virtual ExecutionPtr createExecution() = 0;

 private:
  /// @brief Start the resident worker on the first submit (lazy: tests that
  /// only construct actions do not pay a thread).
  void ensureWorker() {
    std::lock_guard<std::mutex> lock(worker_lifecycle_mutex);
    if (worker.joinable()) {
      return;
    }
    worker = std::thread([this] { workerLoop(); });
  }

  /// @brief Stop the resident worker: cancel everything, wait for the
  /// in-flight goal to finish, then join. No virtual calls here -- the
  /// derived destructor has already run cancelAll() while the object was
  /// still complete.
  void stopWorker() {
    {
      std::lock_guard<std::mutex> lock(worker_lifecycle_mutex);
      worker_stop = true;
    }
    if (stalled_.load()) {
      // The worker is wedged inside a module call: joining would hang
      // forever and destroying a joinable std::thread would terminate.
      // Move the thread object into a process-lifetime container (never
      // destroyed -> no terminate) and let process exit reclaim the stuck
      // thread. See the class comment for the documented residual risk.
      static auto* leaked_threads = new std::vector<std::thread>();
      {
        std::lock_guard<std::mutex> lock(worker_lifecycle_mutex);
        if (worker.joinable()) {
          leaked_threads->push_back(std::move(worker));
          spdlog::critical(
              "ActionBase: stalled worker thread leaked (module ignored "
              "cancel); process exit will reclaim it");
        }
      }
      return;
    }
    {
      std::lock_guard<std::mutex> lock(task_mutex);
      if (pending_goal_) {
        // Destructor path only (well-behaved owners cancelAll() while the
        // derived action is still alive). A goal still parked here can no
        // longer be settled: the virtual finishPendingGoal would dispatch to
        // THIS base's no-op after derived destruction, so its TaskHandle
        // would hang silently -- say so loudly instead.
        spdlog::error(
            "ActionBase: a pending goal is dropped at worker stop without a "
            "result; cancelAll() must run before destruction");
      }
      pending_goal_.reset();
      if (in_flight_ && execution_) {
        execution_->cancel();
      }
    }
    worker_cv.notify_all();
    if (worker.joinable()) {
      worker.join();
    }
  }

  /// @brief Worker body: pop the newest goal, run it inline, publish,
  /// repeat. Strictly sequential -- the module never re-enters.
  void workerLoop() {
    while (true) {
      GoalHandleType gh;
      {
        std::unique_lock<std::mutex> lock(task_mutex);
        worker_cv.wait(lock, [this] { return pending_goal_ || worker_stop; });
        if (!pending_goal_) {
          break;  // stop requested with an empty mailbox
        }
        gh = *pending_goal_;
        pending_goal_.reset();
        in_flight_ = true;
        in_flight_goal_id = gh.id;
        if (!execution_) {
          execution_ = createExecution();
        }
      }
      try {
        runGoal(gh, *execution_);
      } catch (...) {
        // runGoal must not throw: the in-flight record and resource
        // protocols (recovery mutex) released via the result completion
        // would deadlock. abortGoal guarantees a published result.
        abortGoal(gh);
      }
      {
        std::lock_guard<std::mutex> lock(task_mutex);
        if (in_flight_ && in_flight_goal_id == gh.id) {
          in_flight_ = false;
          in_flight_goal_id = 0;
        }
        idle_cv.notify_all();
      }
    }
  }

  /// @brief The single reused execution (created lazily on the first goal).
  ExecutionPtr execution_;
  /// @brief Goal parked while another goal is in flight (one slot: newer
  /// goals replace it; a dropped goal is settled CANCELED immediately).
  std::optional<GoalHandleType> pending_goal_;
  bool in_flight_{false};
  uint64_t in_flight_goal_id{0};
  mutable std::mutex task_mutex;
  /// @brief Set by cancelAll() when the in-flight module ignored cancel for
  /// kStallWait: the action rejects all further goals and its worker thread
  /// is leaked at shutdown (see the class comment).
  std::atomic<bool> stalled_{false};
  /// @brief Worker mailbox/idle signaling.
  std::condition_variable worker_cv;
  std::condition_variable idle_cv;
  bool worker_stop{false};
  std::mutex worker_lifecycle_mutex;
  std::thread worker;
};

} // namespace components
} // namespace ayanami
