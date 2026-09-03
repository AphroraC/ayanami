#pragma once

#include <atomic>
#include <memory>
#include <string>

#include <ayanami/costmap/core/costmap_recovery.hpp>
#include <ayanami/messages/recovery_params.hpp>

namespace ayanami {
namespace modules {

/**
 * @brief 等待恢复模块 (spec §6.6 costmap_wait_recovery)。
 *
 * 纯 C++ (零 ROS): 周期驱动接口 (start/onCycleUpdate)。每周期返回 RUNNING +
 * 零速度, 直到 wait_duration 届满返回 SUCCESS; 无运动占用。插件不自行 sleep,
 * 由 RecoveryExecution 按周期驱动。M1 headless 用, 无真底盘依赖。不读图,
 * 按 CostmapRecovery 契约接收快照但忽略之。
 */
class CostmapWaitRecovery final : public ayanami::costmap::CostmapRecovery {
 public:
  using ayanami::costmap::CostmapRecovery::runRecovery;
  CostmapWaitRecovery();
  // Ruling #3: 基类 AbstractRecovery 无虚析构, 派生层补裸 virtual ~。
  virtual ~CostmapWaitRecovery() = default;

  void initialize(const std::string& name,
                  ayanami::costmap::CostmapContext::ConstPtr map) override;

  FailureCode runRecovery(std::string& message) override;

  void start() override;
  void setBehaviorParams(
      const ayanami::RecoveryBehaviorParams& params) override;
  ayanami::abstract::RecoveryCycleResult onCycleUpdate(
      std::string& message) override;
  void end() override;

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
