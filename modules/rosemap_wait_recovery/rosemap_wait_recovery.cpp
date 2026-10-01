// modules/rosemap_wait_recovery -- timed wait recovery module.
//
// Pure C++ (zero ROS): unified recovery protocol (start/tick/finish).
// Returns RUNNING each cycle (carrying no velocity) until wait_duration
// elapses, then SUCCESS; no motion is occupied.

#include "rosemap_wait_recovery.hpp"

#include <ayanami/utility/config.hpp>

#include <algorithm>
#include <cmath>

namespace ayanami {
namespace modules {

void RosemapWaitRecovery::configureParameters(
    const ayanami::ModuleContext& context) {
  param.wait_duration = context.config.param<double>(
      context.name, "wait_duration", param.wait_duration);
}

void RosemapWaitRecovery::configure(
    const ModuleContext& context,
    ayanami::rosemap::RosemapMapContext::ConstPtr map) {
  (void)map;  // wait does not read the field.
  configureParameters(context);
}

void RosemapWaitRecovery::setCyclePeriod(double seconds) {
  cycle_dt = std::max(seconds, 1e-6);
}

void RosemapWaitRecovery::start() {
  canceled.store(false);
  const double dt = std::max(cycle_dt, 1e-6);
  remaining_frames = static_cast<int>(std::ceil(
      std::max(param.wait_duration, 0.0) / dt));
}

ResultT<void> RosemapWaitRecovery::tick(RecoveryCycleData& cycle) {
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

bool RosemapWaitRecovery::cancel() {
  canceled.store(true);
  return true;
}

}  // namespace modules
}  // namespace ayanami
