#include "ayanami/abstract/components/execution_base.hpp"

namespace ayanami {
namespace components {

ExecutionBase::~ExecutionBase() {
  // If the user forgot to stop, force stop and wait for the run task to finish (equivalent to the old join thread).
  stop();
  join();
}

void ExecutionBase::start() {
  // Equivalent to the old behavior: if the previous run task is still running, wait for it to finish before restarting.
  join();

  should_exit = false;
  cancel_requested = false;
  {
    std::lock_guard<std::mutex> lock(mutex);
    running = true;
  }
  execution_thread = std::thread([this]() { runTask(); });
}

void ExecutionBase::runBlocking() {
  // Inline variant of start(): the caller's thread drives preRun -> run ->
  // postRun directly (execution reuse: the resident action worker calls this
  // per goal instead of spawning an execution thread per task). The flag
  // reset mirrors start(), plus a state reset so observers never see the
  // previous run's terminal state before this run's STARTED.
  should_exit = false;
  cancel_requested = false;
  state.store(0);
  {
    std::lock_guard<std::mutex> lock(mutex);
    running = true;
  }
  runTask();
}

void ExecutionBase::runTask() {
  preRun();
  try {
    run();
  } catch (...) {
    // Last line of defense for exceptions escaping run/preRun/postRun: set
    // the unified fallback terminal state + safe stop. If left in an
    // intermediate state, the action driver task would never observe a
    // terminal state -- slot occupancy, the recovery mutex
    // (recovery_exclusive) and other resource protocols would deadlock,
    // hanging the entire navigation.
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
  // Wake control threads waiting in join/waitForStateUpdate (event-driven teardown).
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
