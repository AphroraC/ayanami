#pragma once

#include <atomic>
#include <memory>
#include <string>

#include <ayanami/costmap/components/costmap_context.hpp>
#include <ayanami/costmap/core/costmap_recovery.hpp>

namespace ayanami {
namespace modules {

/**
 * @brief 等待恢复模块 (spec §6.6 costmap_wait_recovery)。
 *
 * 纯 C++ (零 ROS): 循环 sleep 直到 wait_duration 或 cancel; 不发速度。
 */
class CostmapWaitRecovery : public ayanami::costmap::CostmapRecovery {
 public:
  CostmapWaitRecovery();
  // Ruling #3: 基类 AbstractRecovery 无虚析构, 派生层补裸 virtual ~。
  virtual ~CostmapWaitRecovery() = default;

  void initialize(const std::string& name,
                  ayanami::costmap::CostmapContext::Ptr ctx) override;

  uint32_t runRecovery(std::string& message) override;

  bool cancel() override;

 private:
  struct Config {
    double wait_duration = 0.1;
    double sim_cycle_dt = 0.01;
  };

  void loadConfig();

  ayanami::costmap::CostmapContext::Ptr costmap_context_;
  std::atomic<bool> canceled_{false};
  Config config_;
};

}  // namespace modules
}  // namespace ayanami
