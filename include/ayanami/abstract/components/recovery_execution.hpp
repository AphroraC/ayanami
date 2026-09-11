#pragma once

#include "ayanami/abstract/components/execution_base.hpp"
#include "ayanami/abstract/core/abstract_recovery.hpp"
#include "ayanami/interfaces/callbacks.hpp"
#include "ayanami/messages/geometry_msgs.hpp"
#include "ayanami/messages/recovery_params.hpp"
#include "ayanami/types.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>

#include <spdlog/spdlog.h>

namespace ayanami {
namespace components {

/**
 * @brief Recovery behavior execution (unified protocol: start -> tick loop
 * -> finish).
 *
 * State machine: INITIALIZED->STARTED->RECOVERING->RECOVERY_DONE (or
 * RECOVERY_FAILED/CANCELED/STOPPED/INTERNAL_ERROR/PAT_EXCEEDED).
 *
 * This execution is the sole velocity command publisher during recovery:
 * the module tick() only returns the desired velocity for this cycle
 * (optional), and the execution layer validates finite + max_vel before
 * publishing; non-motion recoveries (velocity=nullopt) publish nothing.
 * All terminal paths -- success/failure/cancel/timeout/exception --
 * publish zero velocity first.
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
      : recovery(std::move(recovery)) {}

  /// @brief Inject per-run override params (extracted from RecoveryGoal by RecoveryAction::acceptGoal).
  void setBehaviorParams(const ayanami::RecoveryBehaviorParams& params) {
    behavior_params = params;
  }

  RecoveryState getState() const {
    return static_cast<RecoveryState>(state.load());
  }

  /// @brief Cancel: set cancel_requested and forward to the recovery module (cooperative).
  void cancel() override {
    cancel_requested = true;
    if (recovery) recovery->cancel();
    publishStopCommand();
    condition.notify_all();
  }

  /// @brief Recovery patience (seconds); <=0 disables.
  void setRecoveryPatience(double seconds) {
    recovery_patience = std::chrono::duration<double>(seconds);
  }

  double getRecoveryPatience() const {
    return recovery_patience.count();
  }

  bool isPatienceExceeded() const {
    if (recovery_patience <= std::chrono::duration<double>(0)) return false;
    return std::chrono::steady_clock::now() - start_time > recovery_patience;
  }

  /// @brief Cycle period (default 10ms, aligned with the module sim_cycle_dt). Only >0 takes effect.
  void setCyclePeriod(std::chrono::microseconds period) {
    if (period.count() > 0) cycle_period = period;
  }

  /// @brief Velocity limit injection (navigation.max_vel_{x,y,theta} single source of truth).
  void setMaximumVelocity(double linear_x, double linear_y,
                          double angular_z) {
    max_linear_x = linear_x;
    max_linear_y = linear_y;
    max_angular_z = angular_z;
  }

  /// @brief Override: publish zero velocity after clearing (the sole publisher's safe stop path).
  void publishStopCommand() override {
    ayanami::geometry_msgs::TwistStamped cmd = {};  // all fields zeroed
    ayanami::interfaces::NavigationCallbacks::on_velocity_command(cmd);
  }

protected:
  /// @brief Module call hook: by default calls the abstract signature;
  /// map-family executions override it with a typed call (the execution
  /// layer performs map access; modules never touch map modules).
  virtual ResultT<void> callRecoveryTick(RecoveryCycleData& cycle) {
    return recovery->tick(cycle);
  }

  void run() override {
    // cancel_requested/should_exit are reset by runBlocking() before the
    // task (resetting them here would drop a cancel that landed between the
    // reset and this point, TOCTOU).
    setState(STARTED, false);
    start_time = std::chrono::steady_clock::now();
    // Module start/setBehaviorParams get the same exception-fallback
    // protection as tick: if a start exception escapes run, the state
    // machine would stop in the intermediate STARTED state and the action
    // driver task would never observe a terminal state -> recovery mutex
    // deadlock (the runTask fallback is the last line of defense, not a
    // dependency).
    try {
      if (recovery) {
        recovery->setBehaviorParams(behavior_params);
        // The cadence this loop will drive the module at: pushed before start()
        // so a module can size its run up front, instead of reading a framework
        // parameter out of its own config file.
        // cycle_period is microseconds -- convert, or the module is told its
        // cadence is ~10000 seconds and sizes every run to a single frame.
        recovery->setCyclePeriod(
            std::chrono::duration<double>(cycle_period).count());
        recovery->start();
      }
      setState(RECOVERING, false);
      while (true) {
        if (cancel_requested) {
          error = ErrorCode::CANCELED;
          publishStopCommand();
          setState(CANCELED, true);
          break;
        }
        if (should_exit) {
          error = ErrorCode::CANCELED;
          publishStopCommand();
          setState(STOPPED, true);
          break;
        }
        if (isPatienceExceeded()) {
          error = ErrorCode::TIMEOUT;
          publishStopCommand();
          setState(PAT_EXCEEDED, true);
          break;
        }

        RecoveryCycleData cycle;
        const ResultT<void> outcome = callRecoveryTick(cycle);
        // If a cancel happens during the cycle call, cancellation wins (for blocking recoveries / modules that forward cancels).
        if (cancel_requested) {
          error = ErrorCode::CANCELED;
          publishStopCommand();
          setState(CANCELED, true);
          break;
        }

        if (!outcome.ok) {
          // Recovery failed/canceled: error=CANCELED -> CANCELED state, otherwise RECOVERY_FAILED.
          error = outcome.error;
          message = outcome.message;
          publishStopCommand();
          setState(outcome.error == ErrorCode::CANCELED ? CANCELED
                                                        : RECOVERY_FAILED,
                   true);
          break;
        }

        // Sole publish: after validation the cycle's suggested velocity is
        // clamped into the max_vel limits (policy aligned with the
        // controller: an over-limit command is clamped + warned, never
        // stopped on); non-motion recoveries (nullopt) publish nothing.
        if (cycle.velocity) {
          publishRecoveryVelocity(*cycle.velocity);
        }

        switch (cycle.status) {
          case RecoveryStatus::RUNNING:
            sleepInterruptibly(cycle_period);
            continue;
          case RecoveryStatus::SUCCESS:
            error = ErrorCode::NONE;
            message = outcome.message;
            publishStopCommand();
            setState(RECOVERY_DONE, true);
            break;
          case RecoveryStatus::CANCELED:
            error = ErrorCode::CANCELED;
            publishStopCommand();
            setState(CANCELED, true);
            break;
          case RecoveryStatus::FAILED:
          default:
            error = outcome.error == ErrorCode::NONE ? ErrorCode::RECOVERY_FAILED
                                                     : outcome.error;
            message = outcome.message;
            publishStopCommand();
            setState(RECOVERY_FAILED, true);
            break;
        }
        break;  // terminal state, exit the loop
      }
    } catch (...) {
      error = ErrorCode::INTERNAL_ERROR;
      message = "Unknown error occurred";
      publishStopCommand();
      setState(INTERNAL_ERROR, true);
    }
    if (recovery) {
      // Swallow cleanup exceptions in place (log them): the terminal state
      // is already settled; a cleanup failure must not rewrite the result
      // nor escape run to trigger the k_aborted override (first-terminal-
      // state-wins discipline).
      try {
        recovery->finish();
      } catch (const std::exception& e) {
        spdlog::warn("RecoveryExecution: recovery finish() threw: {}", e.what());
      } catch (...) {
        spdlog::warn("RecoveryExecution: recovery finish() threw (unknown)");
      }
    }
  }

  /// @brief Publish the suggested velocity after clamping into the max_vel
  /// limits; a non-finite command is unclampable garbage and is zeroed
  /// instead (never reaches the robot).
  void publishRecoveryVelocity(
      const ayanami::geometry_msgs::TwistStamped& suggestion) {
    ayanami::geometry_msgs::TwistStamped cmd = suggestion;
    auto& linear = cmd.twist.linear;
    auto& angular = cmd.twist.angular;
    const bool finite = std::isfinite(linear.x()) && std::isfinite(linear.y()) &&
                        std::isfinite(linear.z()) &&
                        std::isfinite(angular.x()) &&
                        std::isfinite(angular.y()) &&
                        std::isfinite(angular.z());
    if (!finite) {
      // Keep the frame, zero the remaining fields (invalid input never
      // reaches the robot).
      cmd.twist = ayanami::geometry_msgs::Twist{};
    } else {
      const auto clamp = [](double value, double limit) {
        return value > limit ? limit : (value < -limit ? -limit : value);
      };
      const double clamped_x = clamp(linear.x(), max_linear_x);
      const double clamped_y = clamp(linear.y(), max_linear_y);
      const double clamped_w = clamp(angular.z(), max_angular_z);
      if (clamped_x != linear.x() || clamped_y != linear.y() ||
          clamped_w != angular.z()) {
        const auto now = std::chrono::steady_clock::now();
        if (now - last_clamp_warn_at >= std::chrono::seconds(1)) {
          last_clamp_warn_at = now;
          spdlog::warn(
              "RecoveryExecution: velocity suggestion exceeded the limits "
              "(vx={} wz={}; caps vx={} wz={}); clamped",
              linear.x(), angular.z(), max_linear_x, max_angular_z);
        }
      }
      linear.x() = clamped_x;
      linear.y() = clamped_y;
      angular.z() = clamped_w;
    }
    ayanami::interfaces::NavigationCallbacks::on_velocity_command(cmd);
  }

  /// @brief Periodic wait; cancel/stop interrupts immediately (does not wait out the full period).
  void sleepInterruptibly(std::chrono::microseconds period) {
    std::unique_lock<std::mutex> lock(mutex);
    condition.wait_for(lock, period, [this] {
      return cancel_requested.load() || should_exit.load();
    });
  }

protected:
  ayanami::abstract::AbstractRecovery::Ptr recovery;
  ayanami::RecoveryBehaviorParams behavior_params;
  std::chrono::duration<double> recovery_patience{0};
  std::chrono::steady_clock::time_point start_time;
  std::chrono::microseconds cycle_period{10000};
  double max_linear_x{std::numeric_limits<double>::infinity()};
  double max_linear_y{std::numeric_limits<double>::infinity()};
  double max_angular_z{std::numeric_limits<double>::infinity()};
  /// @brief Throttle for the clamp warning (see publishRecoveryVelocity).
  std::chrono::steady_clock::time_point last_clamp_warn_at{};
};

} // namespace components
} // namespace ayanami
