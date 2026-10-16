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
 * @brief Wait recovery module (costmap_wait_recovery) -- the nav2 Wait
 * entry of the recovery rotation.
 *
 * Unified recovery protocol (start/tick/finish). Returns RUNNING each
 * cycle (no velocity) until wait_duration elapses, then SUCCESS; no
 * motion, no map read (per the CostmapRecovery contract it receives the
 * context but ignores it).
 */
class CostmapWaitRecovery final : public ayanami::costmap::CostmapRecovery {
 public:
  CostmapWaitRecovery() = default;
  virtual ~CostmapWaitRecovery() = default;

  void configureParameters(const ayanami::ModuleContext& context) override;

  void configure(const ayanami::ModuleContext& context,
                 ayanami::costmap::CostmapMapContext::ConstPtr map,
                 ClearCommandFn clear_command) override;

  void setCyclePeriod(double seconds) override;
  void start() override;
  void setBehaviorParams(
      const ayanami::RecoveryBehaviorParams& params) override;
  ResultT<void> tick(RecoveryCycleData& cycle) override;
  void finish() override {}
  bool cancel() override;

 private:
  struct CostmapWaitRecoveryParam {
    double wait_duration = 2.0;
  };

  CostmapWaitRecoveryParam param;

  double cycle_dt{0.01};
  std::atomic<bool> canceled{false};
  int remaining_frames{0};
};

}  // namespace modules
}  // namespace ayanami
