#pragma once

#include "ayanami/messages/failure.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>

namespace ayanami {
namespace components {

/**
 * @brief 抽象执行基类 (对标 MBF AbstractExecutionBase, spec §5 components/)。
 *
 * 用专用线程跑可能耗时的导航任务; 内部状态机 (atomic<int>) + condition_variable
 * 向控制线程发布状态更新。派生类覆写 run()/preRun()/postRun()/safetyCheck()/
 * publishStopCommand() 钩子 (map 层覆写, 对标 MBF §2.2 模式)。
 *
 * Ruling #2: 本类内部状态机独立于 actions::GoalStatus (5 态); Task 9 action
 * 层才做 ExecutionState→GoalStatus 映射 (active 合并)。
 */
class ExecutionBase {
public:
  ExecutionBase() = default;

  virtual ~ExecutionBase();

  /// @brief 开线程跑虚 run()。若线程已在跑, 先 stop+join 再重启。
  virtual void start();

  /// @brief 请求退出线程 (rigorous), 线程以 STOPPED 状态退出。
  virtual void stop();

  /// @brief 取消执行: 置 cancel_ 并调虚 publishStopCommand() (安全停止)。
  virtual void cancel();

  /// @brief 等待执行线程结束。
  void join();

  /// @brief 在 condition_ 上等待状态更新, 返回是否在超时前被唤醒。
  bool waitForStateUpdate(std::chrono::microseconds const& duration);

  /// @brief 当前内部状态 (原子读)。
  int getState() const { return state_.load(); }

  /// @brief 最近一次插件调用的统一失败码 (None=成功)。
  FailureCode getFailure() const { return failure_; }

  /// @brief 最近一次的人类可读消息。
  std::string getMessage() const { return message_; }

protected:
  /// @brief 执行主体 (线程入口), 派生覆写。
  virtual void run() {}

  /// @brief 执行前钩子 (map 层 checkActivate 等覆写)。
  virtual void preRun() {}

  /// @brief 执行后钩子 (map 层 checkDeactivate 等覆写)。
  virtual void postRun() {}

  /// @brief 安全检查 (map 层 isCurrent 等覆写)。false 时派生应发布停止命令。
  virtual bool safetyCheck() { return true; }

  /// @brief 发布停止命令 (默认空; controller 覆写为清零 cmd_vel)。
  virtual void publishStopCommand() {}

  /// @brief 更新状态并在 signalling 时通知 condition_。
  void setState(int state, bool signalling = true) {
    state_ = state;
    if (signalling) condition_.notify_all();
  }

  std::thread thread_;
  std::condition_variable condition_;
  std::mutex mutex_;
  std::atomic<bool> should_exit_{false};
  std::atomic<bool> cancel_{false};
  std::atomic<int> state_{0};
  FailureCode failure_{FailureCode::None};
  std::string message_;
};

} // namespace components
} // namespace ayanami
