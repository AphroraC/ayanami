#pragma once

#include "ayanami/messages/failure.hpp"
#include "ayanami/messages/geometry_msgs.hpp"
#include "ayanami/messages/recovery_params.hpp"
#include "ayanami/utility/extension_module.hpp"

#include <memory>
#include <string>

namespace ayanami {
namespace abstract {

/// @brief 周期驱动恢复的单个周期结果。RecoveryExecution 按周期调用
/// onCycleUpdate() 取得本周期期望速度与状态; 插件只返回建议速度, 绝不自行
/// 发布速度命令 (唯一发布者为 RecoveryExecution)。
struct RecoveryCycleResult {
  enum class Status { RUNNING, SUCCESS, FAILED, CANCELED };
  Status status = Status::RUNNING;
  /// @brief 本周期期望速度建议 (由执行层校验 finite + max_vel 并发布)。
  ayanami::geometry_msgs::TwistStamped velocity;
  /// @brief FAILED 时的统一失败码 (决策用)。默认 RecoveryFailed。
  FailureCode failure = FailureCode::RecoveryFailed;
};

/**
 * @brief 恢复行为插件抽象 (spec §5.1)。
 *
 * 注意: 恢复类命名一律 recovery。MBF 语义: 恢复是插件化、可取消的动作。
 *
 * 周期驱动接口 (对标 Nav2 TimedBehavior): RecoveryExecution 按固定周期调用
 * onCycleUpdate() 驱动插件, 并作为唯一 velocity command 发布者。插件不自行
 * sleep 循环、不直接调 NavigationCallbacks::on_velocity_command; spin/back_up
 * 等需要持续输出的恢复只返回每周期期望速度。
 */
class AbstractRecovery : public virtual ExtensionModule {
public:
  using Ptr = std::shared_ptr<AbstractRecovery>;

  virtual ~AbstractRecovery() = default;

  /**
   * @brief 执行一次恢复动作 (一次性接口, 兼容单发插件)。
   * @param[out] message 失败时的人类可读诊断 (仅诊断, 决策只看 failure)。
   * @return FailureCode: None=成功, 其余为统一失败码。
   */
  virtual FailureCode runRecovery(std::string& message) {
    message = "recovery does not provide a map-independent entry point";
    return FailureCode::InternalError;
  }

  /// @brief 每次执行开始时调用, 复位运行态 (框架在 start() 后驱动周期)。
  virtual void start() {}

  /// @brief 执行前注入按次覆盖参数 (RecoveryExecution 每次 run 开始前调用)。
  /// 默认忽略; 需要按次覆盖的恢复 (spin/back_up/wait) 覆写: nullopt 字段
  /// 表示保留 configure 期默认值, 禁止把 nullopt 解释为 0。
  virtual void setBehaviorParams(const ayanami::RecoveryBehaviorParams& params) {
    (void)params;
  }

  /// @brief 单个周期。框架按固定周期重复调用, 驱动到非 RUNNING 为止。
  /// 默认单发: 调 runRecovery() 一次并映射为 SUCCESS/FAILED (携带原失败码)。
  virtual RecoveryCycleResult onCycleUpdate(std::string& message) {
    RecoveryCycleResult r;
    const FailureCode fc = runRecovery(message);
    if (fc == FailureCode::None) {
      r.status = RecoveryCycleResult::Status::SUCCESS;
    } else {
      r.status = RecoveryCycleResult::Status::FAILED;
      r.failure = fc;
    }
    return r;
  }

  /// @brief 每次执行结束时调用 (成功/失败/取消/异常后), 用于清理。
  virtual void end() {}

  virtual void configure() {}
  virtual void activate() {}
  virtual void deactivate() {}
  virtual void cleanup() {}

  /// @brief 取消进行中的恢复。默认不支持, 返回 false。
  virtual bool cancel() { return false; }
};

} // namespace abstract
} // namespace ayanami
