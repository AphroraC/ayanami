#pragma once

#include "ayanami/abstract/components/execution_base.hpp"
#include "ayanami/abstract/core/abstract_smoother.hpp"

#include <chrono>
#include <memory>
#include <thread>

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
      : smoother(std::move(smoother)) {}

  /// @brief 记录入参并启动平滑线程。
  using ExecutionBase::start;
  void smoothPath(const nav_msgs::Path& path, double max_duration,
                  bool check_for_collision) {
    this->path = path;
    this->max_duration = max_duration;
    this->check_for_collision = check_for_collision;
    ExecutionBase::start();
  }

  /// @brief 取消: 置 cancel_requested 并透传到 smoother 插件 (virtual cancel_requested)。
  void cancel() override {
    cancel_requested = true;
    if (smoother) smoother->cancel();
    condition.notify_all();
  }

  SmootherState getState() const {
    return static_cast<SmootherState>(state.load());
  }

  /// @brief 最近一次平滑结果。
  const nav_msgs::Path& getSmoothedPath() const { return smoothed_path; }

  void setMaxRetries(int retries) { max_retries = retries; }

  void setPatience(double seconds) {
    patience = std::chrono::duration<double>(seconds);
  }

  bool isPatienceExceeded() const {
    if (patience <= std::chrono::duration<double>(0)) return false;
    return std::chrono::steady_clock::now() - start_time > patience;
  }

protected:
  /// @brief 插件调用钩子: 默认调 abstract 签名; 地图族 execution 覆写为
  /// 带快照的 typed 调用 (在执行层完成快照获取, 插件不接触地图)。
  virtual FailureCode callSmoother(
      const nav_msgs::Path& in, nav_msgs::Path& out, double max_duration,
      bool check_for_collision, std::string& message) {
    return smoother->smoothPath(in, out, max_duration, check_for_collision,
                                 message);
  }

  void run() override {
    setState(STARTED, false);
    start_time = std::chrono::steady_clock::now();
    int retries = 0;
    try {
      while (true) {
        if (cancel_requested) {
          failure = FailureCode::Canceled;
          setState(CANCELED, true);
          return;
        }
        if (should_exit) {
          failure = FailureCode::Canceled;
          setState(STOPPED, true);
          return;
        }
        setState(SMOOTHING, false);

        failure = callSmoother(path, smoothed_path, max_duration,
                             check_for_collision, message);
        bool success = failure == FailureCode::None;

        if (cancel_requested && !isPatienceExceeded()) {
          failure = FailureCode::Canceled;
          setState(CANCELED, true);
          return;
        } else if (success) {
          setState(SMOOTHED_PATH, true);
          return;
        } else if (max_retries > 0 && ++retries > max_retries) {
          if (failure == FailureCode::None) failure = FailureCode::Timeout;
          setState(MAX_RETRIES, true);
          return;
        } else if (isPatienceExceeded()) {
          failure = FailureCode::Timeout;
          setState(PAT_EXCEEDED, true);
          return;
        } else if (max_retries == 0) {
          setState(NO_SMOOTHED_PATH, true);
          return;
        } else {
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
      }
    } catch (...) {
      failure = FailureCode::InternalError;
      message = "Unknown error occurred";
      setState(INTERNAL_ERROR, true);
    }
  }

protected:
  ayanami::abstract::AbstractSmoother::Ptr smoother;
  nav_msgs::Path path;
  nav_msgs::Path smoothed_path;
  double max_duration{0.0};
  bool check_for_collision{false};
  int max_retries{-1};
  std::chrono::duration<double> patience{0};
  std::chrono::steady_clock::time_point start_time;
};

} // namespace components
} // namespace ayanami
