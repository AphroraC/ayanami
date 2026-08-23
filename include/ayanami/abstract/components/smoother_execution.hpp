#pragma once

#include "ayanami/abstract/components/execution_base.hpp"
#include "ayanami/abstract/core/abstract_smoother.hpp"

#include <chrono>
#include <memory>

namespace ayanami {
namespace components {

/**
 * @brief 路径平滑执行 (对标 MBF AbstractPlanRefinerExecution, spec §5.1)。
 *
 * 单次 smoothPath + 状态机: INITIALIZED→STARTED→SMOOTHING→SMOOTHED_PATH
 * (或 MAX_RETRIES/PAT_EXCEEDED/NO_SMOOTHED_PATH/CANCELED/STOPPED/INTERNAL_ERROR)。
 */
class SmootherExecution : public ExecutionBase {
public:
  using Ptr = std::shared_ptr<SmootherExecution>;

  enum SmootherState {
    INITIALIZED,
    STARTED,
    SMOOTHING,
    SMOOTHED_PATH,
    MAX_RETRIES,
    PAT_EXCEEDED,
    NO_SMOOTHED_PATH,
    CANCELED,
    STOPPED,
    INTERNAL_ERROR,
  };

  explicit SmootherExecution(ayanami::abstract::AbstractSmoother::Ptr smoother)
      : smoother_(std::move(smoother)) {}

  /// @brief 记录入参并启动平滑线程。
  using ExecutionBase::start;
  void smoothPath(const nav_msgs::Path& path, double max_duration,
                  bool check_for_collision) {
    path_ = path;
    max_duration_ = max_duration;
    check_for_collision_ = check_for_collision;
    ExecutionBase::start();
  }

  /// @brief 取消: 置 cancel_ 并透传到 smoother 插件 (virtual cancel)。
  void cancel() override {
    cancel_ = true;
    if (smoother_) smoother_->cancel();
    condition_.notify_all();
  }

  SmootherState getState() const {
    return static_cast<SmootherState>(state_.load());
  }

  /// @brief 最近一次平滑结果。
  const nav_msgs::Path& getSmoothedPath() const { return smoothed_path_; }

  void setMaxRetries(int retries) { max_retries_ = retries; }

  void setPatience(double seconds) {
    patience_ = std::chrono::duration<double>(seconds);
  }

  bool isPatienceExceeded() const {
    if (patience_ <= std::chrono::duration<double>(0)) return false;
    return std::chrono::steady_clock::now() - start_time_ > patience_;
  }

protected:
  void run() override {
    setState(STARTED, false);
    start_time_ = std::chrono::steady_clock::now();
    int retries = 0;
    try {
      while (true) {
        if (cancel_) {
          failure_ = FailureCode::Canceled;
          setState(CANCELED, true);
          return;
        }
        if (should_exit_) {
          failure_ = FailureCode::Canceled;
          setState(STOPPED, true);
          return;
        }
        setState(SMOOTHING, false);

        failure_ = smoother_->smoothPath(path_, smoothed_path_, max_duration_,
                                     check_for_collision_, message_);
        bool success = failure_ == FailureCode::None;

        if (cancel_ && !isPatienceExceeded()) {
          failure_ = FailureCode::Canceled;
          setState(CANCELED, true);
          return;
        } else if (success) {
          setState(SMOOTHED_PATH, true);
          return;
        } else if (max_retries_ > 0 && ++retries > max_retries_) {
          if (failure_ == FailureCode::None) failure_ = FailureCode::Timeout;
          setState(MAX_RETRIES, true);
          return;
        } else if (isPatienceExceeded()) {
          failure_ = FailureCode::Timeout;
          setState(PAT_EXCEEDED, true);
          return;
        } else if (max_retries_ == 0) {
          setState(NO_SMOOTHED_PATH, true);
          return;
        } else {
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
      }
    } catch (...) {
      failure_ = FailureCode::InternalError;
      message_ = "Unknown error occurred";
      setState(INTERNAL_ERROR, true);
    }
  }

private:
  ayanami::abstract::AbstractSmoother::Ptr smoother_;
  nav_msgs::Path path_;
  nav_msgs::Path smoothed_path_;
  double max_duration_{0.0};
  bool check_for_collision_{false};
  int max_retries_{-1};
  std::chrono::duration<double> patience_{0};
  std::chrono::steady_clock::time_point start_time_;
};

} // namespace components
} // namespace ayanami
