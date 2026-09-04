// Task 24: modules/costmap_wait_recovery — 等待恢复模块。
//
// 纯 C++ (零 ROS): 统一恢复协议 (start/tick/finish)。每周期返回 RUNNING
// (不携带速度), 直到 wait_duration 届满返回 SUCCESS; 无运动占用。

#include "costmap_wait_recovery.hpp"

#include <ayanami/utility/config.hpp>

#include <algorithm>
#include <cmath>

namespace ayanami {
namespace modules {


void CostmapWaitRecovery::configure(
    const ayanami::PluginContext& context,
    ayanami::costmap::CostmapContext::ConstPtr map) {
  (void)map;  // wait 不读图。
  // 参数单一来源: 本模块 json (config_recovery 指向文件) 的 recovery 段。
  param.wait_duration = context.config.param<double>(
      "recovery", "wait_duration", param.wait_duration);
  param.sim_cycle_dt =
      context.config.param<double>("recovery", "sim_cycle_dt", param.sim_cycle_dt);
}

void CostmapWaitRecovery::setBehaviorParams(
    const ayanami::RecoveryBehaviorParams& params) {
  // nullopt 字段保留 configure 期默认值 (禁止把 nullopt 解释为 0)。
  if (params.wait_duration) {
    param.wait_duration = *params.wait_duration;
  }
}

void CostmapWaitRecovery::start() {
  canceled.store(false);
  const double dt = std::max(param.sim_cycle_dt, 1e-6);
  remaining_frames = static_cast<int>(std::ceil(
      std::max(param.wait_duration, 0.0) / dt));
}

RecoveryResultT CostmapWaitRecovery::tick() {
  if (canceled.load()) {
    return RecoveryResultT::failure(ErrorCode::CANCELED,
                                    "wait recovery canceled");
  }
  if (remaining_frames <= 0) {
    RecoveryCycleData data;
    data.status = RecoveryStatus::SUCCESS;
    return RecoveryResultT::success(std::move(data));
  }
  --remaining_frames;
  return RecoveryResultT::success(RecoveryCycleData{});
}

bool CostmapWaitRecovery::cancel() {
  canceled.store(true);
  return true;
}

}  // namespace modules
}  // namespace ayanami
