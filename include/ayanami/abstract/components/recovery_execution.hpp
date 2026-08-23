#pragma once

#include "ayanami/abstract/components/execution_base.hpp"
#include "ayanami/abstract/core/abstract_recovery.hpp"

#include <chrono>
#include <memory>

namespace ayanami {
namespace components {

/**
 * @brief 恢复行为执行 (对标 MBF AbstractRecoveryExecution, spec §5.1)。
 *
 * 单次 runRecovery + 状态机: INITIALIZED→STARTED→RECOVERING→RECOVERY_DONE
 * (或 CANCELED/STOPPED/INTERNAL_ERROR)。
 */
class RecoveryExecution : public ExecutionBase {
public:
  using Ptr = std::shared_ptr<RecoveryExecution>;

  enum RecoveryState {
    INITIALIZED,
    STARTED,
    RECOVERING,
    RECOVERY_DONE,
    RECOVERY_FAILED,
    CANCELED,
    STOPPED,
    INTERNAL_ERROR,
    PAT_EXCEEDED,
  };

  explicit RecoveryExecution(ayanami::abstract::AbstractRecovery::Ptr recovery)
      : recovery_(std::move(recovery)) {}

  RecoveryState getState() const {
    return static_cast<RecoveryState>(state_.load());
  }

  /// @brief 取消: 置 cancel_ 并透传到 recovery 插件 (virtual cancel)。
  void cancel() override {
    cancel_ = true;
    if (recovery_) recovery_->cancel();
    condition_.notify_all();
  }

  /// @brief 恢复耐心 (秒); <=0 关闭。取消时等待该时长后仍不返回则超时。
  void setRecoveryPatience(double seconds) {
    recovery_patience_ = std::chrono::duration<double>(seconds);
  }

  bool isPatienceExceeded() const {
    if (recovery_patience_ <= std::chrono::duration<double>(0)) return false;
    return std::chrono::steady_clock::now() - start_time_ > recovery_patience_;
  }

protected:
  void run() override {
    // 注意: 不在 run() 内 reset cancel_。start() 期已复位, 这里只读 cancel_;
    // 若在 run() 开头复位会把 start() 后刚发生的 cancel() 丢掉 (TOCTOU)。
    setState(STARTED, false);
    start_time_ = std::chrono::steady_clock::now();
    setState(RECOVERING, false);
    try {
      failure_ = recovery_->runRecovery(message_);
      if (cancel_) {
        failure_ = FailureCode::Canceled;
        setState(CANCELED, true);
      } else if (isPatienceExceeded()) {
        failure_ = FailureCode::Timeout;
        setState(PAT_EXCEEDED, true);
      } else if (failure_ != FailureCode::None) {
        setState(RECOVERY_FAILED, true);
      } else {
        setState(RECOVERY_DONE, true);
      }
    } catch (...) {
      failure_ = FailureCode::InternalError;
      message_ = "Unknown error occurred";
      setState(INTERNAL_ERROR, true);
    }
  }

private:
  ayanami::abstract::AbstractRecovery::Ptr recovery_;
  std::chrono::duration<double> recovery_patience_{0};
  std::chrono::steady_clock::time_point start_time_;
};

} // namespace components
} // namespace ayanami
