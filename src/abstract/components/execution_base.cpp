#include "ayanami/abstract/components/execution_base.hpp"

namespace ayanami {
namespace components {

ExecutionBase::~ExecutionBase() {
  // Safety net for subclasses destroyed mid-run: request exit (a run that is
  // between cycles observes should_exit; an inline caller has already
  // returned, so there is nothing to wait for).
  stop();
}

void ExecutionBase::runBlocking() {
  // Inline execution: the caller's thread drives preRun -> run -> postRun
  // (the resident action worker calls this per goal). The flag reset plus a
  // state reset ensure observers never see the previous run's terminal
  // state before this run's STARTED.
  should_exit = false;
  cancel_requested = false;
  state.store(0);
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
  // Wake control threads waiting in waitForStateUpdate (event-driven
  // observation).
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

bool ExecutionBase::waitForStateUpdate(std::chrono::microseconds const& duration) {
  std::unique_lock<std::mutex> lock(mutex);
  auto status = condition.wait_for(lock, duration);
  return status == std::cv_status::no_timeout;
}

} // namespace components
} // namespace ayanami
