// Task 24: modules/costmap_wait_recovery — 等待恢复模块。
//
// 纯 C++ (零 ROS): 周期驱动接口 (start/onCycleUpdate)。每周期返回 RUNNING +
// 零速度, 直到 wait_duration 届满返回 SUCCESS; 无运动占用。插件不自行 sleep,
// 由 RecoveryExecution 按周期驱动。M1 headless 用, 无真底盘依赖。

#include "costmap_wait_recovery.hpp"

#include <ayanami/messages/failure.hpp>
#include <ayanami/utility/config.hpp>

#include <algorithm>
#include <cmath>

namespace ayanami {
namespace modules {

CostmapWaitRecovery::CostmapWaitRecovery() {
  // 参数在构造时经 glim 式显式 Config 读取并拷贝进成员 (无 thread_local 注入,
  // 配置不可读时保持默认值): 主 config.json 的 global 单层索引
  // config_wait_recovery → 本插件 json 文件。
  try {
    const Config file(GlobalConfig::getConfigPath("config_wait_recovery"));
    param.wait_duration =
        file.param<double>("recovery", "wait_duration", param.wait_duration);
    param.sim_cycle_dt =
        file.param<double>("recovery", "sim_cycle_dt", param.sim_cycle_dt);
  } catch (...) {
    // 配置不可读时保持默认值。
  }
}

void CostmapWaitRecovery::initialize(const std::string& name,
                                     ayanami::costmap::CostmapContext::ConstPtr map) {
  (void)name;
  (void)map;  // wait 不读图。
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

ayanami::abstract::RecoveryCycleResult CostmapWaitRecovery::onCycleUpdate(
    std::string& message) {
  message.clear();
  ayanami::abstract::RecoveryCycleResult r;  // 默认 RUNNING + 零速度。
  if (canceled.load()) {
    r.status = ayanami::abstract::RecoveryCycleResult::Status::CANCELED;
    r.failure = FailureCode::Canceled;
    message = "wait recovery canceled";
    return r;
  }
  if (remaining_frames <= 0) {
    r.status = ayanami::abstract::RecoveryCycleResult::Status::SUCCESS;
    return r;
  }
  --remaining_frames;
  return r;
}

void CostmapWaitRecovery::end() {}

FailureCode CostmapWaitRecovery::runRecovery(std::string& message) {
  start();
  while (true) {
    const auto r = onCycleUpdate(message);
    switch (r.status) {
      case ayanami::abstract::RecoveryCycleResult::Status::RUNNING:
        continue;
      case ayanami::abstract::RecoveryCycleResult::Status::SUCCESS:
        return FailureCode::None;
      case ayanami::abstract::RecoveryCycleResult::Status::CANCELED:
        return FailureCode::Canceled;
      case ayanami::abstract::RecoveryCycleResult::Status::FAILED:
      default:
        return r.failure;
    }
  }
}

bool CostmapWaitRecovery::cancel() {
  canceled.store(true);
  return true;
}

}  // namespace modules
}  // namespace ayanami
