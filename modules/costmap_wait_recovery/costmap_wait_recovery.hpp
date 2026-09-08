#pragma once

#include <atomic>
#include <memory>
#include <string>

#include <ayanami/costmap/core/costmap_recovery.hpp>
#include <ayanami/messages/failure.hpp>
#include <ayanami/messages/recovery_params.hpp>

namespace ayanami {
namespace modules {

/**
 * @brief Wait recovery module (costmap_wait_recovery).
 *
 * Pure C++ (zero ROS): unified recovery protocol (start/tick/finish). Returns RUNNING each cycle
 * (carrying no velocity) until wait_duration elapses, then returns SUCCESS; no motion is occupied. The plugin does
 * not sleep on its own; RecoveryExecution drives it per cycle. It does not read the map and,
 * per the CostmapRecovery contract, receives the snapshot but ignores it.
 */
class CostmapWaitRecovery final : public ayanami::costmap::CostmapRecovery {
 public:
  CostmapWaitRecovery() = default;
  // The derived layer adds a bare virtual ~ so that shared_ptr deletion goes through the virtual destructor.
  virtual ~CostmapWaitRecovery() = default;

  void configure(const ayanami::PluginContext& context,
                 ayanami::costmap::CostmapMapContext::ConstPtr map) override;

  void start() override;
  void setBehaviorParams(
      const ayanami::RecoveryBehaviorParams& params) override;
  ResultT<void> tick(RecoveryCycleData& cycle) override;
  void finish() override {}

  bool cancel() override;

 private:
  struct CostmapWaitRecoveryParam {
    double wait_duration = 0.1;
    double sim_cycle_dt = 0.01;
  };

  CostmapWaitRecoveryParam param;

  std::atomic<bool> canceled{false};
  int remaining_frames{0};
};

}  // namespace modules
}  // namespace ayanami
