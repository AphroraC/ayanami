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
 * @brief 恢复行为执行 (统一协议: start → tick 循环 → finish)。
 *
 * 状态机: INITIALIZED→STARTED→RECOVERING→RECOVERY_DONE (或 RECOVERY_FAILED/
 * CANCELED/STOPPED/INTERNAL_ERROR/PAT_EXCEEDED)。
 *
 * 本执行是恢复期间的唯一 velocity command 发布者: 插件 tick() 只返回本周期
 * 期望速度 (optional), 执行层校验 finite + max_vel 后发布; 非运动恢复
 * (velocity=nullopt) 不发布。成功/失败/取消/超时/异常等所有终态路径一律先
 * 发布零速度。
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

  /// @brief 取消: 置 cancel_requested 并透传到 recovery 插件 (协作式)。
  void cancel() override {
    cancel_requested = true;
    if (recovery) recovery->cancel();
    publishStopCommand();
    condition.notify_all();
  }

  /// @brief 恢复耐心 (秒); <=0 关闭。
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
  /// @brief 插件调用钩子: 默认调 abstract 签名; 地图族 execution 覆写为
  /// typed 调用 (执行层完成地图访问, 插件不接触地图模块)。
  virtual RecoveryResultT callRecoveryTick() { return recovery->tick(); }

  void run() override {
    // 注意: 不在 run() 内 reset cancel_requested。start() 期已复位, 这里只读;
    // 若在 run() 开头复位会把 start() 后刚发生的 cancel() 丢掉 (TOCTOU)。
    setState(STARTED, false);
    start_time = std::chrono::steady_clock::now();
    // 插件 start()/setBehaviorParams() 与 tick 同受异常兜底保护: start() 抛
    // 异常若逃逸 run(), 状态机将停在 STARTED 中间态, action 驱动任务永远
    // 轮询不到终态 → recovery 互斥死锁 (runTask 兜底是最后防线, 不作依赖)。
    try {
      if (recovery) {
        recovery->setBehaviorParams(behavior_params);
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

        const RecoveryResultT outcome = callRecoveryTick();
        // 若周期调用期间发生取消, 以取消为准 (阻塞恢复/透传取消的插件)。
        if (cancel_requested) {
          error = ErrorCode::CANCELED;
          publishStopCommand();
          setState(CANCELED, true);
          break;
        }

        if (!outcome.ok) {
          // 恢复失败/被取消: error=CANCELED → CANCELED 状态, 其余 RECOVERY_FAILED。
          error = outcome.error;
          message = outcome.message;
          publishStopCommand();
          setState(outcome.error == ErrorCode::CANCELED ? CANCELED
                                                        : RECOVERY_FAILED,
                   true);
          break;
        }

        // 唯一发布: 校验 (finite + max_vel) 后发布本周期建议速度;
        // 非运动恢复 (nullopt) 不发布。
        if (outcome.data.velocity) {
          publishRecoveryVelocity(*outcome.data.velocity);
        }

        switch (outcome.data.status) {
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
        break;  // 终态退出循环
      }
    } catch (...) {
      error = ErrorCode::INTERNAL_ERROR;
      message = "Unknown error occurred";
      publishStopCommand();
      setState(INTERNAL_ERROR, true);
    }
    if (recovery) {
      // 清理异常就地吞掉 (记日志): 终态已结算, 清理失败不得改写结果, 也不得
      // 逃逸 run() 触发 k_aborted 覆盖 (首个终态优先纪律)。
      try {
        recovery->finish();
      } catch (const std::exception& e) {
        spdlog::warn("RecoveryExecution: recovery finish() threw: {}", e.what());
      } catch (...) {
        spdlog::warn("RecoveryExecution: recovery finish() threw (unknown)");
      }
    }
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

  /// @brief 周期等待; cancel/stop 立即打断 (不等满周期)。
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
};

} // namespace components
} // namespace ayanami
