#pragma once

#include "ayanami/messages/actions.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace ayanami {
namespace components {

/**
 * @brief Lifecycle of a single goal (goal_id + goal content + current
 * GoalStatus).
 *
 * States follow actions::GoalStatus (the ACTIVE common state merges
 * Accepted/Executing).
 */
template <typename Goal>
struct GoalHandle {
  uint64_t id{0};
  Goal goal{};
  ayanami::actions::GoalStatus status{ayanami::actions::GoalStatus::PENDING};
  bool cancel_requested{false};
};

/// @brief Feedback wrapper (goal_id + feedback).
template <typename Feedback>
struct GoalFeedback {
  uint64_t goal_id{0};
  Feedback feedback{};
};

/**
 * @brief Shared completion state of a TaskHandle (promise/future + the
 * onComplete slot).
 *
 * The completion payload is actions::GoalResult<Result> (status + result
 * aggregate). After the goal is removed from the ActionServer map, this
 * object is still held by the TaskHandle, so the result can be read safely.
 */
template <typename Result>
struct TaskState {
  using Completion = ayanami::actions::GoalResult<Result>;

  TaskState() : future(promise.get_future().share()) {}

  bool complete(Completion completion) {
    std::vector<std::function<void(const Completion&)>> listeners;
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (completed) {
        return false;
      }
      completed = true;
      value = std::move(completion);
      listeners.swap(callbacks);
    }
    // Listeners run BEFORE the future is published: side-path emits and
    // resource releases (e.g. the recovery exclusive flag) must land before a
    // poller can observe ready()==true. Publishing the future first opens a
    // window where the BT poll loop sees the finished task and re-requests a
    // family whose resource is still flagged busy (spurious RESOURCE_BUSY).
    for (auto& listener : listeners) {
      listener(value);
    }
    promise.set_value(value);
    return true;
  }

  void onComplete(std::function<void(const Completion&)> cb) {
    if (!cb) {
      return;
    }
    Completion c;
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (!completed) {
        callbacks.push_back(std::move(cb));
        return;
      }
      c = value;
    }
    cb(c);
  }

  mutable std::mutex mutex;
  bool completed{false};
  Completion value{};
  std::promise<Completion> promise;
  std::shared_future<Completion> future;
  std::vector<std::function<void(const Completion&)>> callbacks;
};

/**
 * @brief Core<->caller handle used directly by BT nodes (mainline control
 * flow).
 *
 * Mainline: a BT node calls requestPath etc. to obtain a TaskHandle, and
 * its completion callback calls emitWakeUpSignal() to wake the BT loop;
 * this class does not go through NavigationCallbacks.
 * The side path (rviz extension/visualization/fallback) is emitted to
 * NavigationCallbacks separately by the Action layer.
 */
template <typename Result>
class TaskHandle {
public:
  using Completion = ayanami::actions::GoalResult<Result>;

  TaskHandle() = default;

  TaskHandle(uint64_t goal_id, std::shared_ptr<TaskState<Result>> state,
             std::function<bool()> cancel)
      : goal_handle_id(goal_id),
        state(std::move(state)),
        cancel_fn(std::move(cancel)) {}

  /// @brief Whether bound to a real goal.
  bool valid() const noexcept {
    return goal_handle_id != 0 && static_cast<bool>(state);
  }

  uint64_t goal_id() const noexcept { return goal_handle_id; }

  /// @brief Whether completed (future ready).
  bool ready() const {
    return valid() && state->future.wait_for(std::chrono::seconds(0)) ==
                          std::future_status::ready;
  }

  /// @brief Overall goal status (blocks until completion; the call blocks when ready==false).
  ayanami::actions::GoalStatus status() const {
    return state->future.get().status;
  }

  /// @brief The Result payload of each action family (blocks until completion).
  const Result& result() const { return state->future.get().result; }

  /// @brief Register a completion callback (invoked immediately if already completed).
  void onComplete(std::function<void()> cb) const {
    if (!cb || !state) {
      return;
    }
    state->onComplete([cb](const Completion&) { cb(); });
  }

  /// @brief Request cancellation (sets cancel_requested, forwarded to the execution by the Action layer).
  bool cancel() const { return cancel_fn && cancel_fn(); }

private:
  uint64_t goal_handle_id{0};
  std::shared_ptr<TaskState<Result>> state;
  std::function<bool()> cancel_fn;
};

/**
 * @brief Thread-safe action protocol server (modeled after MBF
 * AbstractActionServer).
 *
 * The internal state machine follows actions::GoalStatus: Pending -> Active
 * (the ACTIVE common state merges Accepted/Executing) ->
 * Succeeded/Aborted/Canceled. Finished goals are removed from the map (no
 * accumulation, mirroring legacy CompletedGoalsDoNotAccumulate); TaskHandle
 * holds a shared future, so the result remains readable after the goal is
 * removed.
 */
template <typename Goal, typename Result, typename Feedback>
class ActionServer {
public:
  using GoalHandleType = GoalHandle<Goal>;
  using GoalResultType = ayanami::actions::GoalResult<Result>;
  using Task = TaskHandle<Result>;
  using GoalCallback = std::function<void(const GoalHandleType&)>;
  using CancelCallback = std::function<void(uint64_t)>;
  using ResultCallback = std::function<void(const GoalResultType&)>;
  using FeedbackCallback = std::function<void(const GoalFeedback<Feedback>&)>;

  ActionServer() = default;
  ~ActionServer() { cancelAll(); }

  // ---- Callback injection (derived Actions hook side-path emits / forward to executions) ----
  void setGoalCallback(GoalCallback cb) { goal_callback = std::move(cb); }
  void setCancelCallback(CancelCallback cb) { cancel_callback = std::move(cb); }
  void setResultCallback(ResultCallback cb) { result_callback = std::move(cb); }
  void setFeedbackCallback(FeedbackCallback cb) {
    feedback_callback = std::move(cb);
  }

  /// @brief Accept a goal: create the goal state + TaskHandle (with future), invoke the goal side-path callback.
  Task acceptGoal(const Goal& goal) {
    const uint64_t goal_id = next_goal_id.fetch_add(1);
    GoalHandleType handle{goal_id, goal, ayanami::actions::GoalStatus::PENDING,
                          false};
    auto task_state = std::make_shared<TaskState<Result>>();
    {
      std::lock_guard<std::mutex> lock(mutex);
      goals.emplace(goal_id, StoredGoal{handle, task_state});
    }
    if (goal_callback) {
      goal_callback(handle);
    }
    return Task(goal_id, task_state,
                [this, goal_id]() { return cancelGoal(goal_id); });
  }

  /// @brief Cancel a goal: set cancel_requested and invoke the cancel
  /// side-path callback (forwarded to the execution by the Action layer).
  /// Returns false if the goal does not exist.
  bool cancelGoal(uint64_t goal_id) {
    {
      std::lock_guard<std::mutex> lock(mutex);
      auto iter = goals.find(goal_id);
      if (iter == goals.end()) {
        return false;
      }
      iter->second.handle.cancel_requested = true;
    }
    if (cancel_callback) {
      cancel_callback(goal_id);
    }
    return true;
  }

  /// @brief Cancel all unfinished goals.
  void cancelAll() {
    std::vector<uint64_t> ids;
    {
      std::lock_guard<std::mutex> lock(mutex);
      ids.reserve(goals.size());
      for (const auto& kv : goals) {
        ids.push_back(kv.first);
      }
    }
    for (const auto id : ids) {
      cancelGoal(id);
    }
  }

  // ---- State transitions (ACTIVE common state: Accepted/Executing merged) ----
  bool setActive(uint64_t goal_id) {
    return updateStatus(goal_id, ayanami::actions::GoalStatus::ACTIVE);
  }

  bool setExecuting(uint64_t goal_id) {
    return updateStatus(goal_id, ayanami::actions::GoalStatus::ACTIVE);
  }

  /// @brief Finish a goal (terminal). Returns false if the goal does not exist or is already terminal.
  bool publishResult(uint64_t goal_id, ayanami::actions::GoalStatus status,
                     const Result& result) {
    ResultCallback callback;
    std::shared_ptr<TaskState<Result>> task;
    GoalResultType gr;
    {
      std::lock_guard<std::mutex> lock(mutex);
      auto iter = goals.find(goal_id);
      if (iter == goals.end()) {
        return false;
      }
      if (isTerminal(iter->second.handle.status)) {
        return false;
      }
      iter->second.handle.status = status;
      task = iter->second.task;
      gr = GoalResultType{status, result};
      goals.erase(iter);
      callback = result_callback;
    }
    task->complete(gr);
    if (callback) {
      callback(gr);
    }
    return true;
  }

  /// @brief Publish feedback (via the feedback side-path callback).
  void publishFeedback(uint64_t goal_id, const Feedback& feedback) {
    if (feedback_callback) {
      feedback_callback(GoalFeedback<Feedback>{goal_id, feedback});
    }
  }

  bool hasGoal(uint64_t goal_id) const {
    std::lock_guard<std::mutex> lock(mutex);
    return goals.find(goal_id) != goals.end();
  }

  /// @brief Number of currently non-terminal goals (mirrors legacy activeGoalCount).
  std::size_t activeGoalCount() const {
    std::lock_guard<std::mutex> lock(mutex);
    return goals.size();
  }

  bool isCancelRequested(uint64_t goal_id) const {
    std::lock_guard<std::mutex> lock(mutex);
    auto iter = goals.find(goal_id);
    return iter != goals.end() && iter->second.handle.cancel_requested;
  }

  /// @brief Get a copy of the goal handle (used by ActionBase::start).
  std::optional<GoalHandleType> getGoal(uint64_t goal_id) const {
    std::lock_guard<std::mutex> lock(mutex);
    auto iter = goals.find(goal_id);
    if (iter == goals.end()) {
      return std::nullopt;
    }
    return iter->second.handle;
  }

private:
  struct StoredGoal {
    GoalHandleType handle;
    std::shared_ptr<TaskState<Result>> task;
  };

  bool updateStatus(uint64_t goal_id, ayanami::actions::GoalStatus status) {
    std::lock_guard<std::mutex> lock(mutex);
    auto iter = goals.find(goal_id);
    if (iter == goals.end()) {
      return false;
    }
    if (isTerminal(iter->second.handle.status)) {
      return false;
    }
    iter->second.handle.status = status;
    return true;
  }

  static bool isTerminal(ayanami::actions::GoalStatus status) {
    using GS = ayanami::actions::GoalStatus;
    return status == GS::SUCCEEDED || status == GS::ABORTED ||
           status == GS::CANCELED;
  }

  std::atomic<uint64_t> next_goal_id{1};
  mutable std::mutex mutex;
  std::map<uint64_t, StoredGoal> goals;

  GoalCallback goal_callback;
  CancelCallback cancel_callback;
  ResultCallback result_callback;
  FeedbackCallback feedback_callback;
};

} // namespace components
} // namespace ayanami
