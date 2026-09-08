// modules/costmap_wait_recovery -- wait recovery module.
//
// Pure C++ (zero ROS): unified recovery protocol (start/tick/finish). Returns RUNNING each cycle
// (carrying no velocity) until wait_duration elapses, then returns SUCCESS; no motion is occupied.

#include "costmap_wait_recovery.hpp"

#include <ayanami/utility/config.hpp>

#include <algorithm>
#include <cmath>

namespace ayanami {
namespace modules {


void CostmapWaitRecovery::configure(
    const ayanami::PluginContext& context,
    ayanami::costmap::CostmapMapContext::ConstPtr map) {
  (void)map;  // wait does not read the map.
  // Single source of parameters: the recovery section of this module's json (the file config_recovery points to).
  param.wait_duration = context.config.param<double>(
      "recovery", "wait_duration", param.wait_duration);
  param.sim_cycle_dt =
      context.config.param<double>("recovery", "sim_cycle_dt", param.sim_cycle_dt);
}

void CostmapWaitRecovery::setBehaviorParams(
    const ayanami::RecoveryBehaviorParams& params) {
  // nullopt fields keep the configure-time defaults (interpreting nullopt as 0 is forbidden).
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

ResultT<void> CostmapWaitRecovery::tick(RecoveryCycleData& cycle) {
  if (canceled.load()) {
    return ResultT<void>::failure(ErrorCode::CANCELED,
                                    "wait recovery canceled");
  }
  if (remaining_frames <= 0) {
      cycle.status = RecoveryStatus::SUCCESS;
    return ResultT<void>::success();
  }
  --remaining_frames;
  cycle = RecoveryCycleData{};
  return ResultT<void>::success();
}

bool CostmapWaitRecovery::cancel() {
  canceled.store(true);
  return true;
}

}  // namespace modules
}  // namespace ayanami
