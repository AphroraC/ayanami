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
 * @brief 等待恢复模块 (spec §6.6 costmap_wait_recovery)。
 *
 * 纯 C++ (零 ROS): 统一恢复协议 (start/tick/finish)。每周期返回 RUNNING
 * (不携带速度), 直到 wait_duration 届满返回 SUCCESS; 无运动占用。插件不
 * 自行 sleep, 由 RecoveryExecution 按周期驱动。不读图,
 * 按 CostmapRecovery 契约接收快照但忽略之。
 */
class CostmapWaitRecovery final : public ayanami::costmap::CostmapRecovery {
 public:
  CostmapWaitRecovery() = default;
  // Ruling #3: 派生层补裸 virtual ~ 让 shared_ptr 删除走虚析构。
  virtual ~CostmapWaitRecovery() = default;

  void configure(const ayanami::PluginContext& context,
                 ayanami::costmap::CostmapContext::ConstPtr map) override;

  void start() override;
  void setBehaviorParams(
      const ayanami::RecoveryBehaviorParams& params) override;
  RecoveryResultT tick() override;
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
