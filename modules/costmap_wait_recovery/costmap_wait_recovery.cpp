// modules/costmap_wait_recovery -- wait recovery plugin (no motion).

#include "costmap_wait_recovery.hpp"

#include <ayanami/utility/config.hpp>

#include <algorithm>
#include <cmath>

namespace ayanami {
namespace modules {

void CostmapWaitRecovery::configureParameters(
    const ayanami::ModuleContext& context) {
  param.wait_duration = context.config.param<double>(
      context.name, "wait_duration", param.wait_duration);
}

void CostmapWaitRecovery::configure(
    const ayanami::ModuleContext& context,
    ayanami::costmap::CostmapMapContext::ConstPtr map,
    ClearCommandFn clear_command) {
  (void)map;           // wait reads nothing
  (void)clear_command; // and clears nothing
  configureParameters(context);
}

void CostmapWaitRecovery::setBehaviorParams(
    const ayanami::RecoveryBehaviorParams& params) {
  if (params.wait_duration) {
    param.wait_duration = *params.wait_duration;
  }
}

void CostmapWaitRecovery::setCyclePeriod(double seconds) {
  cycle_dt = std::max(seconds, 1e-6);
}

void CostmapWaitRecovery::start() {
  canceled.store(false);
  const double dt = std::max(cycle_dt, 1e-6);
  remaining_frames =
      static_cast<int>(std::ceil(std::max(param.wait_duration, 0.0) / dt));
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
