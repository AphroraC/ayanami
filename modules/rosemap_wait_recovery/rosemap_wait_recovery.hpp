#pragma once

#include <atomic>
#include <memory>
#include <string>

#include <ayanami/rosemap/core/rosemap_recovery.hpp>
#include <ayanami/types.hpp>

namespace ayanami {
namespace modules {

/**
 * @brief Timed wait recovery (rosemap family; the costmap_wait_recovery
 * twin). Returns RUNNING each cycle carrying no velocity until
 * wait_duration elapses, then SUCCESS; no motion is occupied.
 */
class RosemapWaitRecovery final : public ayanami::rosemap::RosemapRecovery {
 public:
  RosemapWaitRecovery() = default;
  ~RosemapWaitRecovery() override = default;

  void configureParameters(const ayanami::ModuleContext& context) override;

  void configure(const ModuleContext& context,
                 ayanami::rosemap::RosemapMapContext::ConstPtr map) override;

  void setCyclePeriod(double seconds) override;

  void start() override;
  ResultT<void> tick(RecoveryCycleData& cycle) override;
  void finish() override {}

  bool cancel() override;

 private:
  struct RosemapWaitRecoveryParam {
    double wait_duration = 0.5;
  };

  RosemapWaitRecoveryParam param;

  double cycle_dt{0.01};
  std::atomic<bool> canceled{false};
  int remaining_frames{0};
};

}  // namespace modules
}  // namespace ayanami
