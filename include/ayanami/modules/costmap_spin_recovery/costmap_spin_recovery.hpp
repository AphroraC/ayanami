#pragma once

#include <atomic>
#include <memory>
#include <string>

#include <ayanami/costmap/components/costmap_context.hpp>
#include <ayanami/costmap/core/costmap_recovery.hpp>

namespace ayanami {
namespace modules {

/**
 * @brief 原地旋转恢复模块 (spec §6.6 costmap_spin_recovery)。
 *
 * 纯 C++ (零 ROS): 在注入的 CostmapContext 上按 spin_dist/max_w 时长逐帧
 * emit on_velocity_command 纯角速度建议 (M1 headless 无真底盘, 不依赖底盘
 * 实际运动, 发完时长即 SUCCESS); 每帧 10ms + cancel 检查; 碰撞检测
 * (global costmap robot 附近 cell >= 253) → FailureCode::RecoveryFailed。
 */
class CostmapSpinRecovery : public ayanami::costmap::CostmapRecovery {
 public:
  CostmapSpinRecovery();
  // Ruling #3: 基类 AbstractRecovery 无虚析构 (仅 AbstractPlanner 有),
  // 派生层补裸 virtual ~ 让 shared_ptr 删除走虚析构。
  virtual ~CostmapSpinRecovery() = default;

  void initialize(const std::string& name,
                  ayanami::costmap::CostmapContext::Ptr ctx) override;

  FailureCode runRecovery(std::string& message) override;

  bool cancel() override;

 private:
  struct Config {
    double spin_dist = 1.57;
    double max_w = 0.5;
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
