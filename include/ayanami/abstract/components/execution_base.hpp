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
 * run() 状态机以"任务"方式提交到进程级 WorkerPool, 由池内常驻 worker 执行
 * preRun()→run()→postRun(); 不再自建专用 std::thread。内部状态机 (atomic<int>)
 * + condition_variable 向控制线程发布状态更新 (waitForStateUpdate 事件驱动,
 * cancel/stop/setState 均 notify)。派生类覆写 run()/preRun()/postRun()/
 * safetyCheck()/publishStopCommand() 钩子 (map 层覆写, 对标 MBF §2.2 模式)。
 *
 * Ruling #2: 本类内部状态机独立于 actions::GoalStatus (5 态); Task 9 action
 * 层才做 ExecutionState→GoalStatus 映射 (active 合并)。
 */
class ExecutionBase {
public:
  ExecutionBase() = default;

  virtual ~ExecutionBase();

  /// @brief 在 execution 自有线程中异步执行。
  /// 长时 controller/recovery 不占用 action driver 的 worker，避免嵌套
  /// WorkerPool 任务造成 starvation。
  virtual void start();

  /// @brief 请求退出 run() (rigorous), run() 以 STOPPED 状态退出。
  virtual void stop();

  /// @brief 取消执行: 置 cancel_requested 并调虚 publishStopCommand() (安全停止)。
  virtual void cancel();

  /// @brief 等待 run() 任务完全结束 (等价旧实现的 join 执行线程)。
  void join();

  /// @brief 在 condition 上等待状态更新, 返回是否在超时前被唤醒。
  bool waitForStateUpdate(std::chrono::microseconds const& duration);

  /// @brief 当前内部状态 (原子读)。
  int getState() const { return state.load(); }

  /// @brief 最近一次插件调用的统一错误码 (NONE=成功)。
  ErrorCode getError() const { return error; }

  /// @brief 最近一次的人类可读消息。
  std::string getMessage() const { return message; }

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

  /// @brief 更新状态并在 signalling 时通知 condition。
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
  /// @brief WorkerPool 任务体: preRun → run → postRun, 结束后解挂 running。
  void runTask();

  /// @brief 是否仍有 run 任务在跑 (mutex 保护, 供 start/join 谓词等待)。
  bool running{false};
  std::thread execution_thread;
};

} // namespace components
} // namespace ayanami
