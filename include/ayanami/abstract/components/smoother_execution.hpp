#pragma once

#include "ayanami/abstract/components/execution_base.hpp"
#include "ayanami/abstract/core/abstract_smoother.hpp"
#include "ayanami/types.hpp"

#include <memory>
#include <utility>

namespace ayanami {
namespace components {

/**
 * @brief Path smoothing execution (single in-place run).
 *
 * Responsibility table: the module only performs smoothing; a failure is
 * mapped immediately to an ErrorCode terminal state, and retries are
 * decided by the BT.
 * In-place semantics: the path is passed by value and the module modifies
 * it in place; on success getSmoothedPath() returns the smoothed path.
 *
 * State machine: INITIALIZED->STARTED->SMOOTHING->SMOOTHED_PATH (or
 * NO_SMOOTHED_PATH/CANCELED/STOPPED/INTERNAL_ERROR).
 */
class SmootherExecution : public ExecutionBase {
public:
  using Ptr = std::shared_ptr<SmootherExecution>;

  enum SmootherState {
    INITIALIZED,
    STARTED,
    SMOOTHING,
    SMOOTHED_PATH,
    NO_SMOOTHED_PATH,
    CANCELED,
    STOPPED,
    INTERNAL_ERROR,
  };

  explicit SmootherExecution(ayanami::abstract::AbstractSmoother::Ptr smoother)
      : smoother(std::move(smoother)) {}

  /// @brief Record the inputs (taking the path by value) and run the
  /// smoothing task to a terminal state on the calling thread.
  void smoothPath(nav_msgs::Path path, double max_duration) {
    prepareGoal(std::move(path), max_duration);
    runBlocking();
  }

  /// @brief Goal inputs for one smoothing task WITHOUT spawning a thread
  /// (execution reuse: the resident action worker calls prepareGoal +
  /// runBlocking).
  void prepareGoal(nav_msgs::Path path, double max_duration) {
    this->path = std::move(path);
    this->max_duration = max_duration;
  }

  /// @brief Cancel: set cancel_requested and forward to the smoother module (cooperative).
  void cancel() override {
    cancel_requested = true;
    if (smoother) smoother->cancel();
    condition.notify_all();
  }

  SmootherState getState() const {
    return static_cast<SmootherState>(state.load());
  }

  /// @brief The smoothing result (in-place: on success, the smoothed version of the input path).
  const nav_msgs::Path& getSmoothedPath() const { return path; }

  /// @brief Move-out: the action layer takes the smoothed path once the
  /// execution is terminal, avoiding a full path copy. Single-shot execution.
  nav_msgs::Path takeSmoothedPath() { return std::move(path); }

protected:
  /// @brief Module call hook: by default calls the abstract signature;
  /// map-family executions override it with a typed call (the execution
  /// layer performs map access).
  virtual ResultT<void> callSmoother(nav_msgs::Path& path,
                                     double max_duration) {
    return smoother->smoothPath(path, max_duration);
  }

  void run() override {
    setState(STARTED, false);
    try {
      if (cancel_requested) {
        error = ErrorCode::CANCELED;
        setState(CANCELED, true);
        return;
      }
      if (should_exit) {
        error = ErrorCode::CANCELED;
        setState(STOPPED, true);
        return;
      }
      setState(SMOOTHING, false);

      const ResultT<void> result =
          callSmoother(path, max_duration);

      if (cancel_requested) {
        error = ErrorCode::CANCELED;
        setState(CANCELED, true);
        return;
      }
      if (result.ok) {
        // In-place semantics: the module already modified the path member in place; getSmoothedPath is directly valid.
        error = ErrorCode::NONE;
        message = result.message;
        setState(SMOOTHED_PATH, true);
        return;
      }
      // Return the error code immediately on failure (retries are decided by the BT).
      error = result.error;
      message = result.message;
      setState(NO_SMOOTHED_PATH, true);
      return;
    } catch (...) {
      error = ErrorCode::INTERNAL_ERROR;
      message = "Unknown error occurred";
      setState(INTERNAL_ERROR, true);
    }
  }

protected:
  ayanami::abstract::AbstractSmoother::Ptr smoother;
  nav_msgs::Path path;
  double max_duration{0.0};
};

} // namespace components
} // namespace ayanami
