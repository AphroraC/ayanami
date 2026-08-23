#pragma once

#include <atomic>
#include <memory>
#include <string>

#include <ayanami/costmap/components/costmap_context.hpp>
#include <ayanami/costmap/core/costmap_recovery.hpp>

namespace ayanami {
namespace modules {

/**
 * @brief 后退恢复模块 (spec §6.6 costmap_back_up_recovery)。
 *
 * 纯 C++ (零 ROS): 类似 spin, 但逐帧 emit 负向线速度建议 (vx=-max_v);
 * 每帧 10ms + cancel 检查; 碰撞检测 (global costmap robot 附近 cell >= 253)
 * → FailureCode::RecoveryFailed, message 含 "collision detected"。
 */
class CostmapBackUpRecovery : public ayanami::costmap::CostmapRecovery {
 public:
  CostmapBackUpRecovery();
  // Ruling #3: 基类 AbstractRecovery 无虚析构, 派生层补裸 virtual ~。
  virtual ~CostmapBackUpRecovery() = default;

  void initialize(const std::string& name,
                  ayanami::costmap::CostmapContext::Ptr ctx) override;

  FailureCode runRecovery(std::string& message) override;

  bool cancel() override;

 private:
  struct Config {
    double backup_dist = 0.2;
    double max_v = 0.2;
    double sim_cycle_dt = 0.01;
    double collision_radius = 0.3;
  };

  void loadConfig();
  bool collisionNearRobot(const ayanami::geometry_msgs::PoseStamped& pose) const;
  void emitZeroVelocity() const;

  ayanami::costmap::CostmapContext::Ptr costmap_context_;
  std::atomic<bool> canceled_{false};
  Config config_;
};

}  // namespace modules
}  // namespace ayanami
