#pragma once

#include "ayanami/components/action_server.hpp"

#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

namespace ayanami {
namespace components {

/**
 * @brief Action 基类: 并发槽 + 抢占语义 (对标 MBF AbstractActionBase, spec §5)。
 *
 * 每个并发槽 (slot) 持一个 execution + 一个专用槽线程。槽线程在 run() 里
 * 驱动 runImpl() (子类配置 execution 并轮询其状态机分派到 finish), 之后
 * join execution 线程。同槽新 goal 到达时 preempt 旧 execution (cancel) →
 * join 旧槽线程 → 再起新线程 (MBF 抢占 / legacy latest_wins 语义)。
 *
 * 主流控制流: 派生 Action::acceptGoal 返回 TaskHandle 直连调用者 (BT 节点),
 * 不走 NavigationCallbacks (spec §3.4)。
 */
template <typename Goal, typename Execution>
class ActionBase {
public:
  using GoalHandleType = GoalHandle<Goal>;
  using ExecutionPtr = typename Execution::Ptr;

  ActionBase() = default;
  virtual ~ActionBase() { cancelAll(); }

  /**
   * @brief 在指定并发槽启动一次执行。同槽已有 running 则 preempt(旧) +
   * join(旧线程) 再起新线程。execution 用 shared_ptr 以匹配槽成员
   * Execution::Ptr 类型 (createExecution 工厂返回 shared_ptr)。
   */
  void start(const GoalHandleType& goal_handle, ExecutionPtr exec,
             uint8_t slot = 0);

  /// @brief 取消指定槽的 execution (透传到插件 cancel)。
  void cancel(uint8_t slot);

  /// @brief 取消并 join 所有槽 (析构与抢占收尾用)。
  void cancelAll();

  /// @brief 取消指定 goal 所属槽的 execution (TaskHandle::cancel 经 server 透传)。
  bool cancelGoal(uint64_t goal_id);

  bool isRunning(uint8_t slot) const;

protected:
  /// @brief 子类实现执行驱动: 配置 execution + 轮询状态分派到 finish。
  /// 默认空实现 (纯基类不经由派生覆写时仅启动 execution 并 join)。
  virtual void runImpl(const GoalHandleType& goal_handle,
                       Execution& execution) {
    (void)goal_handle;
    (void)execution;
  }

  /// @brief 槽线程入口: runImpl → join execution 线程 → 释放槽。
  void run(const GoalHandleType& goal_handle, ExecutionPtr exec);

private:
  struct ConcurrencySlot {
    ExecutionPtr execution;
    Goal goal;
    std::unique_ptr<std::thread> thread;
    bool in_use{false};
    uint64_t goal_id{0};
  };

  std::map<uint8_t, ConcurrencySlot> slots_;
  mutable std::mutex slots_mutex_;
};

template <typename Goal, typename Execution>
void ActionBase<Goal, Execution>::start(const GoalHandleType& goal_handle,
                                        ExecutionPtr exec, uint8_t slot) {
  std::unique_ptr<std::thread> old_thread;
  ExecutionPtr old_exec;
  {
    std::lock_guard<std::mutex> lock(slots_mutex_);
    auto& s = slots_[slot];
    if (s.in_use) {
      old_exec = s.execution;
      old_thread = std::move(s.thread);
    }
    s.execution = exec;
    s.goal = goal_handle.goal;
    s.goal_id = goal_handle.id;
    s.in_use = true;
    s.thread = std::make_unique<std::thread>([this, goal_handle, exec]() {
      run(goal_handle, exec);
    });
  }
  // 释放锁后再 preempt + join, 避免与旧槽线程的收尾锁交互死锁
  if (old_exec) {
    old_exec->cancel();
  }
  if (old_thread && old_thread->joinable()) {
    old_thread->join();
  }
}

template <typename Goal, typename Execution>
void ActionBase<Goal, Execution>::cancel(uint8_t slot) {
  ExecutionPtr exec;
  {
    std::lock_guard<std::mutex> lock(slots_mutex_);
    auto iter = slots_.find(slot);
    if (iter == slots_.end() || !iter->second.in_use) {
      return;
    }
    exec = iter->second.execution;
  }
  if (exec) {
    exec->cancel();
  }
}

template <typename Goal, typename Execution>
bool ActionBase<Goal, Execution>::cancelGoal(uint64_t goal_id) {
  ExecutionPtr exec;
  {
    std::lock_guard<std::mutex> lock(slots_mutex_);
    for (auto& kv : slots_) {
      auto& s = kv.second;
      if (s.in_use && s.goal_id == goal_id) {
        exec = s.execution;
        break;
      }
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
  std::vector<ExecutionPtr> to_cancel;
  std::vector<std::unique_ptr<std::thread>> to_join;
  {
    std::lock_guard<std::mutex> lock(slots_mutex_);
    for (auto& kv : slots_) {
      auto& s = kv.second;
      if (!s.in_use) {
        continue;
      }
      if (s.execution) {
        to_cancel.push_back(s.execution);
      }
      if (s.thread) {
        to_join.push_back(std::move(s.thread));
      }
      s.thread.reset();
      s.in_use = false;
    }
  }
  for (auto& e : to_cancel) {
    e->cancel();
  }
  for (auto& t : to_join) {
    if (t && t->joinable()) {
      t->join();
    }
  }
}

template <typename Goal, typename Execution>
bool ActionBase<Goal, Execution>::isRunning(uint8_t slot) const {
  std::lock_guard<std::mutex> lock(slots_mutex_);
  auto iter = slots_.find(slot);
  return iter != slots_.end() && iter->second.in_use;
}

template <typename Goal, typename Execution>
void ActionBase<Goal, Execution>::run(const GoalHandleType& goal_handle,
                                      ExecutionPtr exec) {
  if (!exec) {
    return;
  }
  try {
    runImpl(goal_handle, *exec);
  } catch (...) {
    // runImpl 内部不抛; 兜底避免槽线程异常逃逸
  }
  exec->join();
  // 本线程就是槽线程自身: 先 detach 再 reset, 避免销毁 joinable 的 std::thread
  std::lock_guard<std::mutex> lock(slots_mutex_);
  for (auto& kv : slots_) {
    auto& s = kv.second;
    if (s.in_use && s.goal_id == goal_handle.id) {
      s.in_use = false;
      s.execution.reset();
      if (s.thread && s.thread->joinable()) {
        s.thread->detach();
      }
      s.thread.reset();
      break;
    }
  }
}

} // namespace components
} // namespace ayanami
