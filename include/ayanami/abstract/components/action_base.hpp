#pragma once

#include "ayanami/abstract/components/action_server.hpp"

#include <chrono>
#include <condition_variable>
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
 * @brief Action 基类: 并发槽 + 抢占语义 (对标 MBF AbstractActionBase)。
 *
 * 每个并发槽 (slot) 持一个 execution + 一条"驱动线程" (runImpl 一次完整执行,
 * 由 start() 现场创建、退出后由本槽回收)。线程模型与 ExecutionBase 同构:
 * 每个长任务一线程 —— 单导航并发上限 = 四族 action 各 ≤1 驱动线程 + ≤1
 * execution 状态机线程, 天然有界, 无需进程级线程池。
 *
 * 同槽新 goal 到达时必须先 preempt 旧 execution (cancel) 并等旧驱动任务
 * (含旧 execution 状态机) 完全退出 (join) 后才允许提交新任务, 因此同槽插件
 * 绝不会重入 (latest_wins 强串行 / max_running==1); 不同 slot 各自独立并行。
 *
 * 并发控制: 每个 slot 一个独立的 per-slot handoff mutex (slot_handoffs), 把
 * start/cancel/cancelGoal 对同 slot 的 "cancel+等旧任务结束 → 启动新任务" 事务
 * 串行化。handoff 锁只短暂持有 slots_mutex 取指针, 绝不在持 slots_mutex 时等待
 * 旧任务 (slots_cv.wait 会释放锁), 故旧任务收尾 (run() 末尾仍锁 slots_mutex)
 * 不会与 start 互相死锁。
 * slot_handoffs 的 value 用 std::unique_ptr<std::mutex>: std::mutex 不可移动,
 * 直接放进 map value 会破坏元素可移动性/可复制性, unique_ptr 则保持 value 安全。
 * 槽内 task_done 标志 + slots_cv 把旧实现的 join(std::thread) 换成等价的事件
 * 等待: 任务 run() 返回前置 task_done 并 notify, start/cancelAll 等待该标志。
 *
 * 主流控制流: 派生 Action::acceptGoal 返回 TaskHandle 直连调用者 (BT 节点),
 * 不走 NavigationCallbacks 。
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
   * 等旧驱动任务完全退出 (等价 join 旧线程), 之后才提交新任务; 不同 slot 互不
   * 影响可并行。execution 用 shared_ptr 以匹配槽成员 Execution::Ptr 类型
   * (createExecution 工厂返回 shared_ptr)。
   */
  void start(const GoalHandleType& goal_handle, ExecutionPtr exec,
             uint8_t slot = 0);

  /// @brief 取消指定槽的 execution (透传到插件 cancel)。
  void cancel(uint8_t slot);

  /// @brief 取消并等待所有槽任务结束 (析构与抢占收尾用)。
  void cancelAll();

  /// @brief 取消指定 goal 所属槽的 execution (TaskHandle::cancel 经 server 透传)。
  bool cancelGoal(uint64_t goal_id);

  bool isRunning(uint8_t slot) const;

protected:
  /// @brief 子类实现执行驱动: 配置 execution + 轮询状态分派到 finish。
  /// 默认空实现 (纯基类不经由派生覆写时仅启动 execution 并等待其结束)。
  virtual void runImpl(const GoalHandleType& goal_handle,
                       Execution& execution) {
    (void)goal_handle;
    (void)execution;
  }

  /// @brief runImpl 异常兜底: 派生发布 ABORTED 结果, 保证 TaskHandle 完成
  /// (不发布会使等待方悬挂, recovery 互斥等 onComplete 释放的资源协议也会
  /// 卡死)。对已完成 goal 幂等 no-op。默认 no-op (纯测试桩可不覆写)。
  virtual void abortGoal(const GoalHandleType& goal_handle) {
    (void)goal_handle;
  }

  /**
   * @brief runImpl 公共轮询骨架 (DRY 收敛)。
   *
   * 派生 Action 不再各自复制 while{wait;switch} 骨架, 而是注入 StateMapper
   * (lambda/functor, 捕获派生 server/finishResult): 每轮由骨架先调 mapper,
   * 再 waitForStateUpdate 等 execution 状态机推进 (事件驱动: base 的 condition
   * 在 setState/cancel/stop/任务收尾通知, notify 立即唤醒; 短超时仅作兜底)。
   * mapper 负责:
   *   - 透传 cancel (server->isCancelRequested → exec.cancel()),
   *   - 把 Execution 内部状态机枚举 + failure + message 映射到 GoalStatus,
   *   - 调 finish (server->publishResult) 完成 goal,
   * 返回 true 表示已 finish (骨架退出), false 继续轮询。
   */
  template <typename StateMapper>
  void runImplCommon(const GoalHandleType& gh, Execution& exec,
                     StateMapper mapper) {
    while (true) {
      // 兜底异常终态优先于 mapper: run/preRun 逃逸异常的 execution 停在
      // k_aborted, 不被任何派生状态机枚举识别 —— 不在此结算则驱动任务死轮询,
      // 槽占用与 recovery 互斥等资源协议死锁。
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

  /// @brief 驱动任务入口 (本槽驱动线程上跑): runImpl → 等 execution 状态机
  /// 结束 → 释放槽、置 task_done 并发布完成信号 (供 start/cancelAll join)。
  void run(const GoalHandleType& goal_handle, ExecutionPtr exec, uint8_t slot);

private:
  struct ConcurrencySlot {
    ExecutionPtr execution;
    /// @brief 本槽驱动线程 (runImpl 一次完整执行; 退出后由本槽 join 回收)。
    std::thread worker;
    Goal goal;
    bool in_use{false};
    uint64_t goal_id{0};
    /// @brief 本槽最近一次驱动任务是否已完全退出 (run 末尾置位, join 前置)。
    bool task_done{true};
  };

  /// @brief per-slot handoff 锁: 串行化同 slot 的 start/cancel/cancelGoal，
  /// value 用 unique_ptr 保持 map 元素可移动 (std::mutex 本身不可移动)。
  std::map<uint8_t, std::unique_ptr<std::mutex>> slot_handoffs;
  std::map<uint8_t, ConcurrencySlot> slots;
  mutable std::mutex slots_mutex;
  /// @brief 槽驱动任务完成通知 (与 slots_mutex 配套, 谓词为各槽 task_done)。
  std::condition_variable slots_cv;

  /// @brief 取 slot 的 handoff 锁指针 (不存在则创建)。调用方必须已持 slots_mutex。
  std::mutex* handoffLockFor(uint8_t slot) {
    auto iter = slot_handoffs.emplace(slot, std::make_unique<std::mutex>()).first;
    return iter->second.get();
  }

 protected:
  /// @brief 按 goal_id 找当前在跑 slot 的 execution (用于 in-place 更新, 如
  /// controller 路径/版本更新)。找不到返回 nullptr。
  ExecutionPtr executionForGoal(uint64_t goal_id) const {
    std::lock_guard<std::mutex> lock(slots_mutex);
    for (const auto& kv : slots) {
      const auto& s = kv.second;
      if (s.in_use && s.goal_id == goal_id) {
        return s.execution;
      }
    }
    return ExecutionPtr{};
  }
};

template <typename Goal, typename Execution>
void ActionBase<Goal, Execution>::start(const GoalHandleType& goal_handle,
                                        ExecutionPtr exec, uint8_t slot) {
  // 取 per-slot handoff 锁: 同 slot 的抢占事务严格串行, 不同 slot 不受影响。
  std::mutex* handoff;
  {
    std::lock_guard<std::mutex> lock(slots_mutex);
    handoff = handoffLockFor(slot);
  }
  std::lock_guard<std::mutex> handoff_lock(*handoff);

  ExecutionPtr old_exec;
  {
    std::lock_guard<std::mutex> lock(slots_mutex);
    auto& s = slots[slot];
    if (s.in_use) {
      old_exec = s.execution;
      s.in_use = false;  // 占位移除: 抢占窗口内并发 cancel 不再命中旧 execution
    }
  }
  // 严格抢占: 先 cancel 旧 execution 打断其阻塞的插件调用。
  if (old_exec) {
    old_exec->cancel();
  }
  {
    // 事件驱动等待旧驱动任务完全退出 (等价 join 旧槽线程): 旧任务即
    // ActionBase::run, 其末尾 (含 exec->join 等旧 execution 状态机完全退出
    // 之后再置 task_done) 才算结束, 之后才允许新的进入插件 (同槽绝不重入)。
    // 退出中的线程不再触碰 slots_mutex, 持锁 join 无死锁。
    std::unique_lock<std::mutex> lock(slots_mutex);
    auto& s = slots[slot];
    slots_cv.wait(lock, [&s] { return s.task_done; });
    if (s.worker.joinable()) {
      s.worker.join();
    }
    s.execution = exec;
    s.goal = goal_handle.goal;
    s.goal_id = goal_handle.id;
    s.task_done = false;  // 新驱动任务在飞
    s.in_use = true;
    // 驱动任务 = 本槽一条临时线程 ("runImpl 一次完整执行"); 线程模型与
    // ExecutionBase 同构, 槽级生命周期由 task_done + join 管理。
    s.worker = std::thread([this, goal_handle, exec, slot]() {
      run(goal_handle, exec, slot);
    });
  }
}

template <typename Goal, typename Execution>
void ActionBase<Goal, Execution>::cancel(uint8_t slot) {
  std::mutex* handoff;
  {
    std::lock_guard<std::mutex> lock(slots_mutex);
    handoff = handoffLockFor(slot);
  }
  std::lock_guard<std::mutex> handoff_lock(*handoff);
  ExecutionPtr exec;
  {
    std::lock_guard<std::mutex> lock(slots_mutex);
    auto iter = slots.find(slot);
    if (iter == slots.end() || !iter->second.in_use) {
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
    std::lock_guard<std::mutex> lock(slots_mutex);
    for (auto& kv : slots) {
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
    std::lock_guard<std::mutex> lock(slots_mutex);
    for (auto& kv : slots) {
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
  {
    std::lock_guard<std::mutex> lock(slots_mutex);
    for (auto& kv : slots) {
      auto& s = kv.second;
      if (!s.in_use) {
        continue;
      }
      if (s.execution) {
        to_cancel.push_back(s.execution);
      }
      // 占位移除: cancelAll 后任务收尾不再命中旧 execution; execution 引用
      // 在其任务退出后的统一收尾里 reset。
      s.in_use = false;
    }
  }
  for (auto& e : to_cancel) {
    e->cancel();
  }
  // 事件驱动等待所有槽驱动任务完全退出, join 回收全部驱动线程。
  std::unique_lock<std::mutex> lock(slots_mutex);
  slots_cv.wait(lock, [this] {
    for (const auto& kv : slots) {
      if (!kv.second.task_done) {
        return false;
      }
    }
    return true;
  });
  for (auto& kv : slots) {
    if (kv.second.worker.joinable()) {
      kv.second.worker.join();
    }
    kv.second.execution.reset();
  }
}

template <typename Goal, typename Execution>
bool ActionBase<Goal, Execution>::isRunning(uint8_t slot) const {
  std::lock_guard<std::mutex> lock(slots_mutex);
  auto iter = slots.find(slot);
  return iter != slots.end() && iter->second.in_use;
}

template <typename Goal, typename Execution>
void ActionBase<Goal, Execution>::run(const GoalHandleType& goal_handle,
                                      ExecutionPtr exec, uint8_t slot) {
  if (!exec) {
    // 空 execution (防御路径): 清理槽占用并发布完成信号, 否则等待方将悬挂。
    std::lock_guard<std::mutex> lock(slots_mutex);
    auto& s = slots[slot];
    if (s.in_use && s.goal_id == goal_handle.id) {
      s.in_use = false;
    }
    s.task_done = true;
    slots_cv.notify_all();
    return;
  }
  try {
    runImpl(goal_handle, *exec);
  } catch (const std::exception& e) {
    // runImpl 内部不抛; 兜底发布 ABORTED 保证 TaskHandle 完成 (等待方与
    // onComplete 资源协议不悬挂)。
    abortGoal(goal_handle);
    (void)e;
  } catch (...) {
    abortGoal(goal_handle);
  }
  exec->join();
  // 收尾 (本驱动线程退出前): 若仍占有该槽则置空闲并释放 execution; 无论是否
  // 仍占有 (被新 start/cancelAll 抢占时已 in_use=false), 都发布本槽完成信号
  // 供 start/cancelAll join 回收本线程。
  {
    std::lock_guard<std::mutex> lock(slots_mutex);
    auto& s = slots[slot];
    if (s.in_use && s.goal_id == goal_handle.id) {
      s.in_use = false;
      s.execution.reset();
    }
    s.task_done = true;
  }
  slots_cv.notify_all();
}

} // namespace components
} // namespace ayanami