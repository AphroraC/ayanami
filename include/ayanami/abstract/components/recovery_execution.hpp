#pragma once

#include "ayanami/abstract/components/execution_base.hpp"
#include "ayanami/abstract/core/abstract_recovery.hpp"
#include "ayanami/interfaces/callbacks.hpp"
#include "ayanami/messages/geometry_msgs.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <thread>

namespace ayanami {
namespace components {

/**
 * @brief 恢复行为执行 (对标 MBF AbstractRecoveryExecution, spec §5.1)。
 *
 * 按固定周期驱动恢复插件 (start → onCycleUpdate 反复 → end), 状态机:
 * INITIALIZED→STARTED→RECOVERING→RECOVERY_DONE (或 CANCELED/STOPPED/
 * INTERNAL_ERROR/PAT_EXCEEDED)。
 *
 * 自 Task (recovery 发布收口): 本执行是恢复期间的唯一 velocity command
 * 发布者。插件只返回每周期期望速度, 执行层在这里做 finite + max_vel_x/y/theta
 * 安全校验并发布; 成功/失败/取消/超时/异常等所有终态路径一律先发布零速度。
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

  /// @brief 注入按次覆盖参数 (RecoveryAction::acceptGoal 从 RecoveryGoal 提取)。
  void setBehaviorParams(const ayanami::RecoveryBehaviorParams& params) {
    behavior_params = params;
  }

  RecoveryState getState() const {
    return static_cast<RecoveryState>(state.load());
  }

  /// @brief 取消: 置 cancel_requested 并透传到 recovery 插件 (virtual cancel_requested)。
  void cancel() override {
    cancel_requested = true;
    if (recovery) recovery->cancel();
    publishStopCommand();
    condition.notify_all();
  }

  /// @brief 恢复耐心 (秒); <=0 关闭。取消时等待该时长后仍不返回则超时。
  void setRecoveryPatience(double seconds) {
    recovery_patience = std::chrono::duration<double>(seconds);
  }

  bool isPatienceExceeded() const {
    if (recovery_patience <= std::chrono::duration<double>(0)) return false;
    return std::chrono::steady_clock::now() - start_time > recovery_patience;
  }

  /// @brief 周期时长 (默认 10ms, 与插件 sim_cycle_dt 对齐)。>0 才生效。
  void setCyclePeriod(std::chrono::microseconds period) {
    if (period.count() > 0) cycle_period = period;
  }

  /// @brief 速度上限注入 (navigation.max_vel_{x,y,theta} 单一来源)。
  void setMaximumVelocity(double linear_x, double linear_y,
                          double angular_z) {
    max_linear_x = linear_x;
    max_linear_y = linear_y;
    max_angular_z = angular_z;
  }

  /// @brief 覆写: 清零后发布零速度 (唯一发布者的安全停止路径)。
  void publishStopCommand() override {
    ayanami::geometry_msgs::TwistStamped cmd = {};  // 全字段清零
    ayanami::interfaces::NavigationCallbacks::on_velocity_command(cmd);
  }

protected:
  /// @brief 插件调用钩子: 默认调 abstract 周期签名; 地图族 execution 覆写为
  /// 带快照的 typed 调用 (在执行层完成快照获取, 插件不接触地图)。
  virtual ayanami::abstract::RecoveryCycleResult callRecoveryCycle(
      std::string& message) {
    return recovery->onCycleUpdate(message);
  }

  void run() override {
    // 注意: 不在 run() 内 reset cancel_requested。start() 期已复位, 这里只读 cancel_requested;
    // 若在 run() 开头复位会把 start() 后刚发生的 cancel() 丢掉 (TOCTOU)。
    setState(STARTED, false);
    start_time = std::chrono::steady_clock::now();
    if (recovery) {
      recovery->setBehaviorParams(behavior_params);
      recovery->start();
    }
    setState(RECOVERING, false);
    try {
      while (true) {
        if (cancel_requested) {
          failure = FailureCode::Canceled;
          publishStopCommand();
          setState(CANCELED, true);
          break;
        }
        if (should_exit) {
          failure = FailureCode::Canceled;
          publishStopCommand();
          setState(STOPPED, true);
          break;
        }
        if (isPatienceExceeded()) {
          failure = FailureCode::Timeout;
          publishStopCommand();
          setState(PAT_EXCEEDED, true);
          break;
        }

        const auto outcome = callRecoveryCycle(message);
        // 若周期调用期间发生取消, 以取消为准 (阻塞恢复/透传取消的插件)。
        if (cancel_requested) {
          failure = FailureCode::Canceled;
          publishStopCommand();
          setState(CANCELED, true);
          break;
        }

        // 唯一发布: 校验 (finite + max_vel) 后发布本周期建议速度。
        publishRecoveryVelocity(outcome.velocity);

        switch (outcome.status) {
          case ayanami::abstract::RecoveryCycleResult::Status::RUNNING:
            std::this_thread::sleep_for(cycle_period);
            continue;
          case ayanami::abstract::RecoveryCycleResult::Status::SUCCESS:
            failure = FailureCode::None;
            publishStopCommand();
            setState(RECOVERY_DONE, true);
            break;
          case ayanami::abstract::RecoveryCycleResult::Status::CANCELED:
            failure = FailureCode::Canceled;
            publishStopCommand();
            setState(CANCELED, true);
            break;
          case ayanami::abstract::RecoveryCycleResult::Status::FAILED:
          default:
            failure = outcome.failure;
            publishStopCommand();
            setState(RECOVERY_FAILED, true);
            break;
        }
        break;  // 终态退出循环
      }
    } catch (...) {
      failure = FailureCode::InternalError;
      message = "Unknown error occurred";
      publishStopCommand();
      setState(INTERNAL_ERROR, true);
    }
    if (recovery) recovery->end();
  }

  /// @brief 校验 (finite + max_vel 上限) 后发布建议速度; 非法命令清零。
  void publishRecoveryVelocity(
      const ayanami::geometry_msgs::TwistStamped& suggestion) {
    ayanami::geometry_msgs::TwistStamped cmd = suggestion;
    if (!validVelocityCommand(cmd)) {
      // 保留 frame, 其余字段清零 (非法输入不落机器人)。
      cmd.twist = ayanami::geometry_msgs::Twist{};
    }
    ayanami::interfaces::NavigationCallbacks::on_velocity_command(cmd);
  }

  bool validVelocityCommand(
      const ayanami::geometry_msgs::TwistStamped& command) const {
    const auto& linear = command.twist.linear;
    const auto& angular = command.twist.angular;
    return std::isfinite(linear.x()) && std::isfinite(linear.y()) &&
           std::isfinite(linear.z()) && std::isfinite(angular.x()) &&
           std::isfinite(angular.y()) && std::isfinite(angular.z()) &&
           std::abs(linear.x()) <= max_linear_x &&
           std::abs(linear.y()) <= max_linear_y &&
           std::abs(angular.z()) <= max_angular_z;
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
};

} // namespace components
} // namespace ayanami
