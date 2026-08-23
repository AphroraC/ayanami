#include "ayanami/abstract/components/execution_base.hpp"

namespace ayanami {
namespace components {

ExecutionBase::~ExecutionBase() {
  if (thread_.joinable()) {
    // 若用户忘了 stop(), 强制停止并等待线程结束
    stop();
    thread_.join();
  }
}

void ExecutionBase::start() {
  if (thread_.joinable()) {
    stop();
    thread_.join();
  }

  should_exit_ = false;
  cancel_ = false;
  thread_ = std::thread([this]() {
    preRun();
    try {
      run();
    } catch (...) {
      message_ = "Unknown error occurred";
      condition_.notify_all();
    }
    postRun();
  });
}

void ExecutionBase::stop() {
  should_exit_ = true;
  condition_.notify_all();
}

void ExecutionBase::cancel() {
  cancel_ = true;
  publishStopCommand();
  condition_.notify_all();
}

void ExecutionBase::join() {
  if (thread_.joinable()) {
    thread_.join();
  }
}

bool ExecutionBase::waitForStateUpdate(std::chrono::microseconds const& duration) {
  std::unique_lock<std::mutex> lock(mutex_);
  auto status = condition_.wait_for(lock, duration);
  return status == std::cv_status::no_timeout;
}

} // namespace components
} // namespace ayanami
