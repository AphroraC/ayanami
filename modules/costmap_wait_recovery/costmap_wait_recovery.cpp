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



void CostmapWaitRecovery::configureParameters(const ayanami::ModuleContext& context) {
  // Parameter loading, separated from configuration: the loader calls this
  // right after a successful load, and every value lands in the module's own
  // CostmapWaitRecoveryParam member (whose field initializers are the defaults for keys the
  // config omits).
  // Single source of parameters: the recovery section of this module's json (the file config_recovery points to).
  param.wait_duration = context.config.param<double>(
      context.name, "wait_duration", param.wait_duration);
}

void CostmapWaitRecovery::configure(
    const ayanami::ModuleContext& context,
    ayanami::costmap::CostmapMapContext::ConstPtr map) {
  // Direct-construction callers (tests, or a host that builds the object
  // itself) never go through the loader's configureParameters call, so bind
  // the parameters here as well. Re-reading the same keys from the same
  // config is idempotent.
  (void)map;  // wait does not read the map.
  configureParameters(context);
}


void CostmapWaitRecovery::setBehaviorParams(
    const ayanami::RecoveryBehaviorParams& params) {
  // nullopt fields keep the configure-time defaults (interpreting nullopt as 0 is forbidden).
  if (params.wait_duration) {
    param.wait_duration = *params.wait_duration;
  }
}

void CostmapWaitRecovery::setCyclePeriod(double seconds) {
  // Framework input, not a config value: the execution owns the tick cadence.
  cycle_dt = std::max(seconds, 1e-6);
}

void CostmapWaitRecovery::start() {
  canceled.store(false);
  const double dt = std::max(cycle_dt, 1e-6);
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
