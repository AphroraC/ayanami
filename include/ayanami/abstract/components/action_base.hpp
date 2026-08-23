#pragma once

#include "ayanami/abstract/components/action_server.hpp"

#include <chrono>
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
 * join execution 线程。同槽新 goal 到达时必须先 preempt 旧 execution (cancel)
 * 并完整 join 旧槽线程, 等旧 execution 完全退出后才允许启动新线程, 因此同槽
 * 插件绝不会重入 (latest_wins 强串行 / max_running==1); 不同 slot 仍可并发。
 *
 * 并发控制: 每个 slot 一个独立的 per-slot handoff mutex (slot_handoffs_), 把
 * start/cancel/cancelGoal 对同 slot 的 "cancel+join 旧 → 启动新" 事务串行化。
 * handoff 锁只短暂持有 slots_mutex_ 取指针, 绝不在持 slots_mutex_ 时 join 旧
 * 线程, 故旧线程收尾 (run() 末尾仍锁 slots_mutex_) 不会与 start 互相死锁。
 * slot_handoffs_ 的 value 用 std::unique_ptr<std::mutex>: std::mutex 不可移动,
 * 直接放进 map value 会破坏元素可移动性/可复制性, unique_ptr 则保持 value 安全。
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
   * @brief 在指定并发槽启动一次执行。同槽已有 running 则先 preempt(旧) +
   * join(旧线程) 至其完全退出, 之后才启动新线程; 不同 slot 互不影响可并发。
   * execution 用 shared_ptr 以匹配槽成员 Execution::Ptr 类型 (createExecution
   * 工厂返回 shared_ptr)。
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

  /**
   * @brief runImpl 公共轮询骨架 (Ruling #4 DRY 收敛)。
   *
   * 派生 Action 不再各自复制 while{wait;switch} 骨架, 而是注入 StateMapper
   * (lambda/functor, 捕获派生 server_/finishResult): 每轮由骨架先调 mapper,
   * 再 waitForStateUpdate(50ms) 等 execution 状态机推进。mapper 负责:
   *   - 透传 cancel (server_->isCancelRequested → exec.cancel()),
   *   - 把 Execution 内部状态机枚举 + failure + message 映射到 GoalStatus,
   *   - 调 finish (server_->publishResult) 完成 goal,
   * 返回 true 表示已 finish (骨架退出), false 继续轮询。
   */
  template <typename StateMapper>
  void runImplCommon(const GoalHandleType& gh, Execution& exec,
                     StateMapper mapper) {
    while (true) {
      if (mapper(gh, exec)) {
        return;
      }
      exec.waitForStateUpdate(std::chrono::milliseconds(50));
    }
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

  /// @brief per-slot handoff 锁: 串行化同 slot 的 start/cancel/cancelGoal，
  /// value 用 unique_ptr 保持 map 元素可移动 (std::mutex 本身不可移动)。
  std::map<uint8_t, std::unique_ptr<std::mutex>> slot_handoffs_;
  std::map<uint8_t, ConcurrencySlot> slots_;
  mutable std::mutex slots_mutex_;

  /// @brief 取 slot 的 handoff 锁指针 (不存在则创建)。调用方必须已持 slots_mutex_。
  std::mutex* handoffLockFor(uint8_t slot) {
    auto iter = slot_handoffs_.emplace(slot, std::make_unique<std::mutex>()).first;
    return iter->second.get();
  }
};

template <typename Goal, typename Execution>
void ActionBase<Goal, Execution>::start(const GoalHandleType& goal_handle,
                                        ExecutionPtr exec, uint8_t slot) {
  // 取 per-slot handoff 锁: 同 slot 的抢占事务严格串行, 不同 slot 不受影响。
  std::mutex* handoff;
  {
    std::lock_guard<std::mutex> lock(slots_mutex_);
    handoff = handoffLockFor(slot);
  }
  std::lock_guard<std::mutex> handoff_lock(*handoff);

  std::unique_ptr<std::thread> old_thread;
  ExecutionPtr old_exec;
  {
    std::lock_guard<std::mutex> lock(slots_mutex_);
    auto& s = slots_[slot];
    if (s.in_use) {
      old_exec = s.execution;
      old_thread = std::move(s.thread);
      s.in_use = false;  // 占位移除: 抢占窗口内并发 cancel 不再命中旧 execution
    }
  }
  // 严格抢占: 先 cancel 旧 execution 并 join 旧槽线程至完全退出 (绝不持
  // slots_mutex_ 时 join, 旧线程收尾要锁 slots_mutex_)。
  if (old_exec) {
    old_exec->cancel();
  }
  if (old_thread && old_thread->joinable()) {
    old_thread->join();
  }
  // 旧 execution 已完全退出, 这时才允许新的进入插件 (同槽插件绝不重入)。
  {
    std::lock_guard<std::mutex> lock(slots_mutex_);
    auto& s = slots_[slot];
    s.execution = exec;
    s.goal = goal_handle.goal;
    s.goal_id = goal_handle.id;
    s.in_use = true;
    s.thread = std::make_unique<std::thread>([this, goal_handle, exec]() {
      run(goal_handle, exec);
    });
  }
}

template <typename Goal, typename Execution>
void ActionBase<Goal, Execution>::cancel(uint8_t slot) {
  std::mutex* handoff;
  {
    std::lock_guard<std::mutex> lock(slots_mutex_);
    handoff = handoffLockFor(slot);
  }
  std::lock_guard<std::mutex> handoff_lock(*handoff);
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
  // 先定位 goal 所属 slot 并取其 handoff 锁, 再在持锁下复核并 cancel。
  std::mutex* handoff = nullptr;
  {
    std::lock_guard<std::mutex> lock(slots_mutex_);
    for (auto& kv : slots_) {
      auto& s = kv.second;
      if (s.in_use && s.goal_id == goal_id) {
        handoff = handoffLockFor(kv.first);
        break;
      }
    }
  }
  if (!handoff) {
    return false;
  }
  std::lock_guard<std::mutex> handoff_lock(*handoff);
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
