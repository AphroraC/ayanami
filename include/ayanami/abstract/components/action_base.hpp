#pragma once

#include "ayanami/abstract/components/action_server.hpp"

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <thread>
#include <utility>

namespace ayanami {
namespace components {

/**
 * @brief Action base class: one in-flight task + strict preemption semantics.
 *
 * Single-navigation discipline: each action family runs at most one task at
 * a time (one configured plugin per family; recoveries are per-plugin action
 * instances), so the old MBF-style concurrency-slot map has collapsed into a
 * single in-flight record. Each task owns one driver thread (a full runImpl
 * execution, spawned by start() and reaped by the task teardown) plus the
 * execution's own state-machine thread -- the threading model mirrors
 * ExecutionBase: one thread per long task, naturally bounded without a
 * process-wide pool.
 *
 * When a new goal arrives while a task is in flight, the old execution is
 * first preempted (cancel) and the old driver task (including the old
 * execution state machine) must have fully exited (join) before the new task
 * may enter the plugin -- a family's plugin never re-enters (latest-wins
 * strict serialization / max_running==1).
 *
 * Concurrency control: a handoff mutex serializes the
 * start/cancel/cancelGoal transaction "cancel + wait for the old task to
 * exit -> start the new one". The handoff lock only briefly takes task_mutex
 * and never waits for the old task while holding task_mutex (task_cv.wait
 * releases it), so old-task teardown (run() still takes task_mutex at its
 * end) cannot deadlock with start. The task_done flag + task_cv turn the old
 * join(std::thread) into an event wait: run() sets task_done and notifies
 * before returning; start/cancelAll wait on that flag.
 *
 * Mainline control flow: the derived Action::acceptGoal returns a TaskHandle
 * straight to the caller (BT node), bypassing NavigationCallbacks.
 */
template <typename Goal, typename Execution>
class ActionBase {
 public:
  using GoalHandleType = GoalHandle<Goal>;
  using ExecutionPtr = typename Execution::Ptr;

  ActionBase() = default;
  virtual ~ActionBase() { cancelAll(); }

  /// @brief Start one execution. If a task is already in flight, first
  /// preempt it (cancel) and wait for the old driver task to fully exit
  /// (equivalent to joining the old thread) before submitting the new one.
  /// The execution uses shared_ptr to match InFlight::execution's type (the
  /// createExecution factory returns shared_ptr).
  void start(const GoalHandleType& goal_handle, ExecutionPtr exec);

  /// @brief Cancel the in-flight execution (forwards to the plugin cancel).
  void cancel();

  /// @brief Cancel and wait for the in-flight task to finish (used by the
  /// destructor and preemption teardown).
  void cancelAll();

  /// @brief Cancel the in-flight execution only if it owns the given goal
  /// (TaskHandle::cancel forwarded via the server). Returns false when the
  /// goal is not the one in flight.
  bool cancelGoal(uint64_t goal_id);

  bool isRunning() const;

 protected:
  /// @brief Derived classes implement the execution driver: configure the
  /// execution + poll the state and dispatch to finish.
  /// Default empty implementation (when the pure base is not overridden by
  /// a derived class, it merely starts the execution and waits for it to
  /// finish).
  virtual void runImpl(const GoalHandleType& goal_handle,
                       Execution& execution) {
    (void)goal_handle;
    (void)execution;
  }

  /// @brief runImpl exception fallback: the derived class publishes an
  /// ABORTED result, guaranteeing TaskHandle completion (not publishing
  /// would hang waiters, and resource protocols such as the recovery mutex
  /// released in onComplete would deadlock as well). Idempotent no-op for
  /// an already-finished goal. Default no-op (pure test stubs may skip
  /// overriding).
  virtual void abortGoal(const GoalHandleType& goal_handle) {
    (void)goal_handle;
  }

  /**
   * @brief Common polling skeleton for runImpl (DRY consolidation).
   *
   * Derived Actions no longer each copy the while{wait;switch} skeleton;
   * instead they inject a StateMapper (lambda/functor capturing the derived
   * server/finishResult): each round the skeleton first calls the mapper,
   * then waitForStateUpdate to wait for the execution state machine to
   * advance (event-driven: the base condition variable is notified on
   * setState/cancel/stop/task teardown, waking immediately; the short
   * timeout is only a fallback).
   * The mapper is responsible for:
   *   - forwarding cancel (server->isCancelRequested -> exec.cancel()),
   *   - mapping the Execution internal state-machine enum + failure +
   *     message onto GoalStatus,
   *   - calling finish (server->publishResult) to complete the goal,
   * returning true once finished (skeleton exits), false to keep polling.
   */
  template <typename StateMapper>
  void runImplCommon(const GoalHandleType& gh, Execution& exec,
                     StateMapper mapper) {
    while (true) {
      // The fallback exception terminal state takes priority over the
      // mapper: an execution that escaped run/preRun with an exception stays
      // in k_aborted, unrecognized by any derived state-machine enum -- if
      // not settled here the driver task spins forever, and the in-flight
      // record plus resource protocols such as the recovery mutex deadlock.
      if (exec.aborted()) {
        abortGoal(gh);
        return;
      }
      if (mapper(gh, exec)) {
        return;
      }
      exec.waitForStateUpdate(std::chrono::milliseconds(20));
    }
  }

  /// @brief Driver task entry (runs on the driver thread): runImpl -> wait
  /// for the execution state machine to finish -> release the in-flight
  /// record, set task_done and publish the completion signal (for
  /// start/cancelAll join).
  void run(const GoalHandleType& goal_handle, ExecutionPtr exec);

 private:
  struct InFlight {
    ExecutionPtr execution;
    /// @brief This task's driver thread (joined and reaped on exit).
    std::thread worker;
    Goal goal;
    bool in_use{false};
    uint64_t goal_id{0};
    /// @brief Whether the driver task has fully exited (set at the end of
    /// run; precondition for join).
    bool task_done{true};
  };

  /// @brief Single in-flight record: a family's plugin never re-enters.
  InFlight task;
  mutable std::mutex task_mutex;  ///< protects the task record fields
  /// @brief Serializes the start/cancel/cancelGoal transaction.
  std::mutex handoff_mutex;
  /// @brief Driver-task completion notification (paired with task_mutex;
  /// the predicate is task_done).
  std::condition_variable task_cv;

 protected:
  /// @brief Find the in-flight execution by goal_id (for in-place updates,
  /// e.g. controller path/version updates). Returns nullptr when the goal is
  /// not the one in flight.
  ExecutionPtr executionForGoal(uint64_t goal_id) const {
    std::lock_guard<std::mutex> lock(task_mutex);
    if (task.in_use && task.goal_id == goal_id) {
      return task.execution;
    }
    return ExecutionPtr{};
  }
};

template <typename Goal, typename Execution>
void ActionBase<Goal, Execution>::start(const GoalHandleType& goal_handle,
                                        ExecutionPtr exec) {
  // Take the handoff lock: the preemption transaction is strictly
  // serialized against concurrent cancel/cancelGoal.
  std::lock_guard<std::mutex> handoff_lock(handoff_mutex);

  ExecutionPtr old_exec;
  {
    std::lock_guard<std::mutex> lock(task_mutex);
    if (task.in_use) {
      old_exec = task.execution;
      task.in_use = false;  // Occupancy removed: concurrent cancel during the preemption window no longer hits the old execution
    }
  }
  // Strict preemption: first cancel the old execution to interrupt its blocking plugin calls.
  if (old_exec) {
    old_exec->cancel();
  }
  {
    // Event-driven wait for the old driver task to fully exit (equivalent
    // to joining the old driver thread): the old task is ActionBase::run and
    // only counts as finished at its end (after exec->join has waited for
    // the old execution state machine to fully exit, task_done is then set);
    // only then may the new task enter the plugin (never re-enters).
    // The exiting thread no longer touches task_mutex, so joining under
    // the lock cannot deadlock.
    std::unique_lock<std::mutex> lock(task_mutex);
    task_cv.wait(lock, [this] { return task.task_done; });
    if (task.worker.joinable()) {
      task.worker.join();
    }
    task.execution = exec;
    task.goal = goal_handle.goal;
    task.goal_id = goal_handle.id;
    task.task_done = false;  // New driver task in flight
    task.in_use = true;
    // Driver task = one temporary thread ("one full runImpl execution");
    // the threading model mirrors ExecutionBase, task lifecycle is managed
    // by task_done + join.
    task.worker = std::thread([this, goal_handle, exec]() {
      run(goal_handle, exec);
    });
  }
}

template <typename Goal, typename Execution>
void ActionBase<Goal, Execution>::cancel() {
  std::lock_guard<std::mutex> handoff_lock(handoff_mutex);
  ExecutionPtr exec;
  {
    std::lock_guard<std::mutex> lock(task_mutex);
    if (!task.in_use) {
      return;
    }
    exec = task.execution;
  }
  if (exec) {
    exec->cancel();
  }
}

template <typename Goal, typename Execution>
bool ActionBase<Goal, Execution>::cancelGoal(uint64_t goal_id) {
  // Take the handoff lock, then re-check that the in-flight goal matches and cancel.
  std::lock_guard<std::mutex> handoff_lock(handoff_mutex);
  ExecutionPtr exec;
  {
    std::lock_guard<std::mutex> lock(task_mutex);
    if (task.in_use && task.goal_id == goal_id) {
      exec = task.execution;
    }
  }
  if (!exec) {
    return false;
  }
  exec->cancel();
  return true;
}

template <typename Goal, typename Execution>
void ActionBase<Goal, Execution>::cancelAll() {
  ExecutionPtr to_cancel;
  {
    std::lock_guard<std::mutex> lock(task_mutex);
    if (task.in_use && task.execution) {
      to_cancel = task.execution;
    }
    // Occupancy removed: after cancelAll the task teardown no longer hits
    // the old execution; the execution reference is reset in the unified
    // teardown after its task exits.
    task.in_use = false;
  }
  if (to_cancel) {
    to_cancel->cancel();
  }
  // Event-driven wait for the driver task to fully exit; join and reap the driver thread.
  std::unique_lock<std::mutex> lock(task_mutex);
  task_cv.wait(lock, [this] { return task.task_done; });
  if (task.worker.joinable()) {
    task.worker.join();
  }
  task.execution.reset();
}

template <typename Goal, typename Execution>
bool ActionBase<Goal, Execution>::isRunning() const {
  std::lock_guard<std::mutex> lock(task_mutex);
  return task.in_use;
}

template <typename Goal, typename Execution>
void ActionBase<Goal, Execution>::run(const GoalHandleType& goal_handle,
                                      ExecutionPtr exec) {
  if (!exec) {
    // Null execution (defensive path): clear the occupancy and publish the completion signal, otherwise waiters would hang.
    std::lock_guard<std::mutex> lock(task_mutex);
    if (task.in_use && task.goal_id == goal_handle.id) {
      task.in_use = false;
    }
    task.task_done = true;
    task_cv.notify_all();
    return;
  }
  try {
    runImpl(goal_handle, *exec);
  } catch (const std::exception& e) {
    // runImpl does not throw internally; the fallback publishes ABORTED to
    // guarantee TaskHandle completion (waiters and the onComplete resource
    // protocol do not hang).
    abortGoal(goal_handle);
    (void)e;
  } catch (...) {
    abortGoal(goal_handle);
  }
  exec->join();
  // Teardown (before this driver thread exits): if the task is still
  // occupied, mark it idle and release the execution; regardless of
  // occupancy (in_use is already false when preempted by a new
  // start/cancelAll), publish the completion signal so start/cancelAll can
  // join and reap this thread.
  {
    std::lock_guard<std::mutex> lock(task_mutex);
    if (task.in_use && task.goal_id == goal_handle.id) {
      task.in_use = false;
      task.execution.reset();
    }
    task.task_done = true;
  }
  task_cv.notify_all();
}

} // namespace components
} // namespace ayanami
