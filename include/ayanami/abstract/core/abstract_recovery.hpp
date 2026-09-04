#pragma once

#include "ayanami/messages/recovery_params.hpp"
#include "ayanami/types.hpp"
#include "ayanami/utility/extension_module.hpp"

#include <memory>
#include <string>

namespace ayanami {
namespace abstract {

/**
 * @brief 恢复行为插件抽象 (统一执行协议: start / tick / cancel / finish)。
 *
 * 注意: 恢复类命名一律 recovery。MBF 语义: 恢复是插件化、可取消的动作。
 *
 * RecoveryExecution 按 fixed cycle 驱动: start() 一次 → tick() 反复 →
 * finish() 一次, 并作为唯一 velocity command 发布者。插件不自行 sleep
 * 循环、不直接调 NavigationCallbacks::on_velocity_command; spin/back_up 等
 * 需要持续输出的恢复在 tick() 内返回本周期期望速度 (optional); 清图等
 * 非运动恢复不携带速度。
 *
 * tick() 返回约定:
 * - ok=true + status=RUNNING [+ velocity]: 恢复进行中;
 * - ok=true + status=SUCCESS: 恢复完成;
 * - ok=false + ErrorCode: 恢复失败/被取消 (CANCELED)。
 */
class AbstractRecovery : public virtual ExtensionModule {
 public:
  using Ptr = std::shared_ptr<AbstractRecovery>;

  virtual ~AbstractRecovery() = default;

  /// @brief 每次执行开始时调用 (复位运行态; 框架在执行前调用一次)。
  virtual void start() {}

  /// @brief 单个周期 (纯虚)。框架按固定周期重复调用, 驱动到终态。
  virtual RecoveryResultT tick() = 0;

  /// @brief 每次执行结束时调用 (成功/失败/取消/异常后), 用于清理。
  virtual void finish() {}

  /// @brief 执行前注入按次覆盖参数 (RecoveryExecution 每次 run 开始前调用)。
  /// 默认忽略; 需要按次覆盖的恢复 (spin/back_up/wait) 覆写: nullopt 字段
  /// 表示保留 configure 期默认值, 禁止把 nullopt 解释为 0。
  virtual void setBehaviorParams(const ayanami::RecoveryBehaviorParams& params) {
    (void)params;
  }

  virtual void activate() {}
  virtual void deactivate() {}
  virtual void cleanup() {}

  /// @brief 协作式取消进行中的恢复 (阻塞型插件用以打断; 默认不支持)。
  virtual bool cancel() { return false; }
};

} // namespace abstract
} // namespace ayanami
