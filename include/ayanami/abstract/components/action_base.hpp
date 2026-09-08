#pragma once

#include "ayanami/abstract/components/action_server.hpp"

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
 * @brief Action base class: one in-flight task + strict preemption semantics.
 *
 * Single-navigation discipline: each action family runs at most one task at
 * a time (one configured plugin per family; recoveries are per-plugin action
 * instances), so the old MBF-style concurrency-slot map has collapsed into a
 * single in-flight record. Each task owns one driver thread (a full runImpl
 * execution); a family's plugin never re-enters (latest-wins strict
 * serialization / max_running==1).
 *
 * Preemption is asynchronous (mbf/nav2 style): start() on a busy family
 * marks the running execution canceled (cancel() already publishes the
 * zeroed stop command synchronously), parks the new goal in a one-slot
 * pending record and returns immediately. When the in-flight driver task
 * fully exits (its execution joined), the teardown chains the pending goal
 * onto a fresh driver thread -- so the tick thread never blocks on a slow
 * plugin call, while the plugin still never re-enters. Use cancelAll() for
 * the synchronous stop/destructor path (cancel + wait for everything).
 *
 * Concurrency control: a handoff mutex serializes start/cancel transactions
 * and the teardown chaining decision; it is held only briefly and never
 * across a join. The task_done flag + task_cv turn joins into event waits.
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

  /// @brief Submit one execution. Non-blocking: if a task is in flight the
  /// running execution is canceled (asynchronously torn down) and the new
  /// goal is parked in the pending slot; the driver for it is chained by the
  /// exiting task's teardown. A goal parked here that is itself replaced or
  /// canceled before starting is settled via finishPendingGoal().
  void start(const GoalHandleType& goal_handle, ExecutionPtr exec) {
    std::lock_guard<std::mutex> handoff_lock(handoff_mutex);
    std::vector<GoalHandleType> dropped;
    bool spawn_now = false;
    {
      std::lock_guard<std::mutex> lock(task_mutex);
      if (pending_start) {
        dropped.push_back(pending_start->goal_handle);
        pending_start.reset();
      }
      if (task.in_use) {
        // Mark the old execution canceled (it publishes its stop command
        // synchronously inside cancel()); its teardown chains the new goal.
        if (task.execution) {
          task.execution->cancel();
        }
        pending_start = PendingStart{goal_handle, std::move(exec)};
      } else {
        // Reap a finished-but-unjoined predecessor first: assigning a new
        // thread over a joinable handle would terminate. The predecessor
        // already published task_done, so it holds no lock anymore and this
        // join cannot deadlock.
        if (task.worker.joinable()) {
          task.worker.join();
        }
        task.execution = exec;
        task.goal = goal_handle.goal;
        task.goal_id = goal_handle.id;
        task.task_done = false;
        task.in_use = true;
        spawn_now = true;
      }
    }
    for (const auto& handle : dropped) {
      finishPendingGoal(handle);
    }
    if (spawn_now) {
      task.worker = std::thread([this, goal_handle, exec]() {
        run(goal_handle, exec);
      });
    }
  }

  /// @brief Cancel the in-flight execution (forwards to the plugin cancel).
  void cancel() {
    std::lock_guard<std::mutex> handoff_lock(handoff_mutex);
    std::vector<GoalHandleType> dropped;
    ExecutionPtr exec;
    {
      std::lock_guard<std::mutex> lock(task_mutex);
      if (pending_start) {
        dropped.push_back(pending_start->goal_handle);
        pending_start.reset();
      }
      if (task.in_use) {
        exec = task.execution;
      }
    }
    for (const auto& handle : dropped) {
      finishPendingGoal(handle);
    }
    if (exec) {
      exec->cancel();
    }
  }

  /// @brief Cancel the in-flight execution only if it owns the given goal
  /// (TaskHandle::cancel forwarded via the server). Also settles a pending
  /// goal with the same id (canceled before its driver ever started).
  /// Returns false when the goal is not the one in flight/pending.
  bool cancelGoal(uint64_t goal_id) {
    std::lock_guard<std::mutex> handoff_lock(handoff_mutex);
    std::optional<GoalHandleType> dropped;
    ExecutionPtr exec;
    {
      std::lock_guard<std::mutex> lock(task_mutex);
      if (pending_start && pending_start->goal_handle.id == goal_id) {
        dropped = pending_start->goal_handle;
        pending_start.reset();
      } else if (task.in_use && task.goal_id == goal_id) {
        exec = task.execution;
      }
    }
    if (dropped) {
      finishPendingGoal(*dropped);
      return true;
    }
    if (!exec) {
      return false;
    }
    exec->cancel();
    return true;
  }

  /// @brief Non-blocking preemption: settle the pending goal, mark the
  /// in-flight execution canceled and return. Used by runtime paths running
  /// on the BT tick thread (halt / recovery start) that must not block on a
  /// slow plugin call.
  void preemptAll() {
    std::lock_guard<std::mutex> handoff_lock(handoff_mutex);
    std::vector<GoalHandleType> dropped;
    ExecutionPtr exec;
    {
      std::lock_guard<std::mutex> lock(task_mutex);
      if (pending_start) {
        dropped.push_back(pending_start->goal_handle);
        pending_start.reset();
      }
      if (task.in_use) {
        exec = task.execution;
      }
    }
    for (const auto& handle : dropped) {
      finishPendingGoal(handle);
    }
    if (exec) {
      exec->cancel();
    }
  }

  /// @brief Cancel and wait for the in-flight task to finish (used by the
  /// destructor and the stop/teardown path, where plugin lifetime requires
  /// a synchronous shutdown).
  void cancelAll() {
    preemptAll();
    std::unique_lock<std::mutex> lock(task_mutex);
    task_cv.wait(lock, [this] { return task.task_done; });
    if (task.worker.joinable()) {
      task.worker.join();
    }
    task.execution.reset();
  }

  bool isRunning() const {
    std::lock_guard<std::mutex> lock(task_mutex);
    return task.in_use;
  }

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

  /// @brief Settle a goal that was preempted from the pending slot (replaced
  /// by a newer goal or canceled) before its driver ever started. Derived
  /// classes publish a CANCELED result so the TaskHandle never hangs.
  virtual void finishPendingGoal(const GoalHandleType& goal_handle) {
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
  /// record; if a pending goal was parked meanwhile, chain it onto a fresh
  /// driver thread (strict serialization: the new goal enters the plugin
  /// only after this task fully exits).
  void run(const GoalHandleType& goal_handle, ExecutionPtr exec) {
    if (!exec) {
      // Null execution (defensive path): clear the occupancy and publish the
      // completion signal, otherwise waiters would hang.
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
    // Teardown: release the occupancy, then either chain a pending goal onto
    // a fresh driver thread or settle this task as done. The handoff mutex
    // serializes the chaining decision against concurrent
    // start/cancel/cancelGoal; it is never held across a join.
    std::optional<PendingStart> next;
    std::thread retiring_worker;
    {
      std::lock_guard<std::mutex> handoff_lock(handoff_mutex);
      std::lock_guard<std::mutex> lock(task_mutex);
      if (task.in_use && task.goal_id == goal_handle.id) {
        task.in_use = false;
        task.execution.reset();
      }
      next = std::move(pending_start);
      pending_start.reset();
      if (next) {
        retiring_worker = std::move(task.worker);
        task.execution = next->execution;
        task.goal = next->goal_handle.goal;
        task.goal_id = next->goal_handle.id;
        task.in_use = true;
        task.task_done = false;  // the chained task is in flight now
        task.worker = std::thread([this, gh = next->goal_handle,
                                   chained = next->execution]() mutable {
          run(gh, chained);
        });
      } else {
        task.task_done = true;
      }
    }
    task_cv.notify_all();
    // The retiring handle is this thread itself (it is about to exit);
    // detaching avoids the joinable-destructor terminate.
    if (retiring_worker.joinable()) {
      retiring_worker.detach();
    }
    (void)next;
  }

 private:
  struct PendingStart {
    GoalHandleType goal_handle;
    ExecutionPtr execution;
  };

  struct InFlight {
    ExecutionPtr execution;
    /// @brief This task's driver thread (joined and reaped on exit).
    std::thread worker;
    Goal goal;
    bool in_use{false};
    uint64_t goal_id{0};
    /// @brief Whether the driver task has fully exited and nothing was
    /// chained after it (precondition for cancelAll's join).
    bool task_done{true};
  };

  /// @brief Single in-flight record: a family's plugin never re-enters.
  InFlight task;
  /// @brief Goal parked while a preemption waits for the in-flight task to
  /// exit (one slot: newer goals replace it).
  std::optional<PendingStart> pending_start;
  mutable std::mutex task_mutex;  ///< protects the task/pending records
  /// @brief Serializes the start/cancel/cancelGoal/preempt transactions and
  /// the teardown chaining decision.
  std::mutex handoff_mutex;
  /// @brief Driver-task completion notification (paired with task_mutex;
  /// the predicate is task_done).
  std::condition_variable task_cv;

 protected:
  /// @brief Find the in-flight execution by goal_id (for in-place updates,
  /// e.g. controller path/version updates). Returns nullptr when the goal is
  /// not the one in flight (a still-pending goal cannot be updated).
  ExecutionPtr executionForGoal(uint64_t goal_id) const {
    std::lock_guard<std::mutex> lock(task_mutex);
    if (task.in_use && task.goal_id == goal_id) {
      return task.execution;
    }
    return ExecutionPtr{};
  }
};

} // namespace components
} // namespace ayanami
