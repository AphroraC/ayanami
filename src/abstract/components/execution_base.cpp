#include "ayanami/abstract/components/execution_base.hpp"

#include "ayanami/abstract/components/worker_pool.hpp"

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
  // 状态机任务提交到进程级 WorkerPool, 不再自建 std::thread。
  WorkerPool::instance().submit([this]() { runTask(); });
}

void ExecutionBase::runTask() {
  preRun();
  try {
    run();
  } catch (...) {
    message = "Unknown error occurred";
    condition.notify_all();
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
  std::unique_lock<std::mutex> lock(mutex);
  condition.wait(lock, [this] { return !running; });
}

bool ExecutionBase::waitForStateUpdate(std::chrono::microseconds const& duration) {
  std::unique_lock<std::mutex> lock(mutex);
  auto status = condition.wait_for(lock, duration);
  return status == std::cv_status::no_timeout;
}

} // namespace components
} // namespace ayanami