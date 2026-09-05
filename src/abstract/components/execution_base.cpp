#include "ayanami/abstract/components/execution_base.hpp"

namespace ayanami {
namespace components {

ExecutionBase::~ExecutionBase() {
  // 若用户忘了 stop(), 强制停止并等待 run() 任务结束 (等价旧 join 线程)。
  stop();
  join();
}

void ExecutionBase::start() {
  // 与旧行为等价: 若上一次 run 任务仍在跑, 先等其结束再重启。
  join();

  should_exit = false;
  cancel_requested = false;
  {
    std::lock_guard<std::mutex> lock(mutex);
    running = true;
  }
  execution_thread = std::thread([this]() { runTask(); });
}

void ExecutionBase::runTask() {
  preRun();
  try {
    run();
  } catch (...) {
    // run()/preRun()/postRun() 逃逸异常的最后防线: 置统一兜底终态 + 安全
    // 停止。若停在中间态, action 驱动任务将永远轮询不到终态 —— 槽占用、
    // recovery 互斥 (recovery_exclusive) 等资源协议随之死锁, 整条导航挂死。
    error = ErrorCode::INTERNAL_ERROR;
    message = "Execution aborted by uncaught exception";
    publishStopCommand();
    setState(k_aborted, true);
  }
  postRun();
  {
    std::lock_guard<std::mutex> lock(mutex);
    running = false;
  }
  // 唤醒 join()/waitForStateUpdate 等控制线程 (事件驱动收尾)。
  condition.notify_all();
}

void ExecutionBase::stop() {
  should_exit = true;
  condition.notify_all();
}

void ExecutionBase::cancel() {
  cancel_requested = true;
  publishStopCommand();
  condition.notify_all();
}

void ExecutionBase::join() {
  if (execution_thread.joinable()) {
    execution_thread.join();
  }
}

bool ExecutionBase::waitForStateUpdate(std::chrono::microseconds const& duration) {
  std::unique_lock<std::mutex> lock(mutex);
  auto status = condition.wait_for(lock, duration);
  return status == std::cv_status::no_timeout;
}

} // namespace components
} // namespace ayanami
