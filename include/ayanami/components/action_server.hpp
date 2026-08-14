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
 * @brief 单个目标的生命周期 (goal_id + goal 内容 + 当前 GoalStatus)。
 *
 * 状态沿用 actions::GoalStatus (Ruling #2: ACTIVE 通态合并 Accepted/Executing)。
 */
template <typename Goal>
struct GoalHandle {
  uint64_t id{0};
  Goal goal{};
  ayanami::actions::GoalStatus status{ayanami::actions::GoalStatus::PENDING};
  bool cancel_requested{false};
};

/// @brief 反馈封装 (goal_id + feedback)。
template <typename Feedback>
struct GoalFeedback {
  uint64_t goal_id{0};
  Feedback feedback{};
};

/**
 * @brief TaskHandle 的共享完成状态 (promise/future + onComplete 槽)。
 *
 * 完成负载是 actions::GoalResult<Result> (status + result 聚合, spec §3.3)。
 * goal 从 ActionServer map 移出后, 本对象仍被 TaskHandle 持有, 可安全读结果。
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
    promise.set_value(value);
    for (auto& listener : listeners) {
      listener(value);
    }
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
 * @brief BT 节点直连的核心↔调用者句柄 (spec §3.4 主流控制流)。
 *
 * 主流: BT 节点调 requestPath 等拿 TaskHandle, completion 回调里
 * emitWakeUpSignal() 唤醒 BT 循环; 本类不走 NavigationCallbacks。
 * 旁路 (rviz 扩展/可视化/回退) 由 Action 层单独 emit 到 NavigationCallbacks。
 */
template <typename Result>
class TaskHandle {
public:
  using Completion = ayanami::actions::GoalResult<Result>;

  TaskHandle() = default;

  TaskHandle(uint64_t goal_id, std::shared_ptr<TaskState<Result>> state,
             std::function<bool()> cancel)
      : goal_id_(goal_id), state_(std::move(state)), cancel_(std::move(cancel)) {}

  /// @brief 是否绑定到真实 goal。
  bool valid() const noexcept {
    return goal_id_ != 0 && static_cast<bool>(state_);
  }

  uint64_t goal_id() const noexcept { return goal_id_; }

  /// @brief 是否已完成 (future ready)。
  bool ready() const {
    return valid() && state_->future.wait_for(std::chrono::seconds(0)) ==
                          std::future_status::ready;
  }

  /// @brief 整体目标状态 (阻塞到完成; ready()==false 时调用会阻塞)。
  ayanami::actions::GoalStatus status() const {
    return state_->future.get().status;
  }

  /// @brief 各 action 族的 Result 负载 (阻塞到完成)。
  const Result& result() const { return state_->future.get().result; }

  /// @brief 注册完成回调 (已完成则立即调用)。
  void onComplete(std::function<void()> cb) const {
    if (!cb || !state_) {
      return;
    }
    state_->onComplete([cb](const Completion&) { cb(); });
  }

  /// @brief 请求取消 (置 cancel_requested, 由 Action 层透传到 execution)。
  bool cancel() const { return cancel_ && cancel_(); }

private:
  uint64_t goal_id_{0};
  std::shared_ptr<TaskState<Result>> state_;
  std::function<bool()> cancel_;
};

/**
 * @brief 线程安全 action 协议服务器 (对标 MBF AbstractActionServer, spec §5)。
 *
 * 内部状态机沿用 actions::GoalStatus (Ruling #2): Pending → Active (ACTIVE
 * 通态合并 Accepted/Executing) → Succeeded/Aborted/Canceled。goal 完成后从
 * map 移除 (不累积, 对标 legacy CompletedGoalsDoNotAccumulate); TaskHandle
 * 持共享 future, goal 移出后仍可读取结果。
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

  // ---- 回调注入 (派生 Action 接旁路 emit / execution 透传) ----
  void setGoalCallback(GoalCallback cb) { goal_callback_ = std::move(cb); }
  void setCancelCallback(CancelCallback cb) { cancel_callback_ = std::move(cb); }
  void setResultCallback(ResultCallback cb) { result_callback_ = std::move(cb); }
  void setFeedbackCallback(FeedbackCallback cb) {
    feedback_callback_ = std::move(cb);
  }

  /// @brief 接受目标: 建 goal 状态 + TaskHandle (含 future), 调 goal 旁路回调。
  Task acceptGoal(const Goal& goal) {
    const uint64_t goal_id = next_goal_id_.fetch_add(1);
    GoalHandleType handle{goal_id, goal, ayanami::actions::GoalStatus::PENDING,
                          false};
    auto task_state = std::make_shared<TaskState<Result>>();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      goals_.emplace(goal_id, StoredGoal{handle, task_state});
    }
    if (goal_callback_) {
      goal_callback_(handle);
    }
    return Task(goal_id, task_state,
                [this, goal_id]() { return cancelGoal(goal_id); });
  }

  /// @brief 取消目标: 置 cancel_requested 并调 cancel 旁路回调
  /// (Action 层透传到 execution)。返回 false 若 goal 不存在。
  bool cancelGoal(uint64_t goal_id) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      auto iter = goals_.find(goal_id);
      if (iter == goals_.end()) {
        return false;
      }
      iter->second.handle.cancel_requested = true;
    }
    if (cancel_callback_) {
      cancel_callback_(goal_id);
    }
    return true;
  }

  /// @brief 取消所有未完成目标。
  void cancelAll() {
    std::vector<uint64_t> ids;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      ids.reserve(goals_.size());
      for (const auto& kv : goals_) {
        ids.push_back(kv.first);
      }
    }
    for (const auto id : ids) {
      cancelGoal(id);
    }
  }

  // ---- 状态迁移 (Ruling #2 ACTIVE 通态: Accepted/Executing 合并) ----
  bool setActive(uint64_t goal_id) {
    return updateStatus(goal_id, ayanami::actions::GoalStatus::ACTIVE);
  }

  bool setExecuting(uint64_t goal_id) {
    return updateStatus(goal_id, ayanami::actions::GoalStatus::ACTIVE);
  }

  /// @brief 完成目标 (terminal)。返回 false 若 goal 不存在或已 terminal。
  bool publishResult(uint64_t goal_id, ayanami::actions::GoalStatus status,
                     const Result& result) {
    ResultCallback callback;
    std::shared_ptr<TaskState<Result>> task;
    GoalResultType gr;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      auto iter = goals_.find(goal_id);
      if (iter == goals_.end()) {
        return false;
      }
      if (isTerminal(iter->second.handle.status)) {
        return false;
      }
      iter->second.handle.status = status;
      task = iter->second.task;
      gr = GoalResultType{status, result};
      goals_.erase(iter);
      callback = result_callback_;
    }
    task->complete(gr);
    if (callback) {
      callback(gr);
    }
    return true;
  }

  /// @brief 发布反馈 (经 feedback 旁路回调)。
  void publishFeedback(uint64_t goal_id, const Feedback& feedback) {
    if (feedback_callback_) {
      feedback_callback_(GoalFeedback<Feedback>{goal_id, feedback});
    }
  }

  bool hasGoal(uint64_t goal_id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return goals_.find(goal_id) != goals_.end();
  }

  /// @brief 当前非 terminal 的 goal 数量 (对标 legacy activeGoalCount)。
  std::size_t activeGoalCount() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return goals_.size();
  }

  bool isCancelRequested(uint64_t goal_id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto iter = goals_.find(goal_id);
    return iter != goals_.end() && iter->second.handle.cancel_requested;
  }

  /// @brief 取 goal 句柄副本 (ActionBase::start 用)。
  std::optional<GoalHandleType> getGoal(uint64_t goal_id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto iter = goals_.find(goal_id);
    if (iter == goals_.end()) {
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
    std::lock_guard<std::mutex> lock(mutex_);
    auto iter = goals_.find(goal_id);
    if (iter == goals_.end()) {
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

  std::atomic<uint64_t> next_goal_id_{1};
  mutable std::mutex mutex_;
  std::map<uint64_t, StoredGoal> goals_;

  GoalCallback goal_callback_;
  CancelCallback cancel_callback_;
  ResultCallback result_callback_;
  FeedbackCallback feedback_callback_;
};

} // namespace components
} // namespace ayanami
