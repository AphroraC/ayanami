#pragma once

#include <atomic>
#include <memory>
#include <string>

#include <Eigen/Geometry>

#include <ayanami/costmap/core/costmap_recovery.hpp>
#include <ayanami/messages/recovery_params.hpp>
#include <ayanami/messages/geometry_msgs.hpp>

namespace ayanami {
namespace modules {

/**
 * @brief 后退恢复模块 (spec §6.6 costmap_back_up_recovery)。
 *
 * 纯 C++ (零 ROS): 周期驱动接口 (start/onCycleUpdate)。每周期返回负向线速度
 * 建议 (vx = -max_v), 由 RecoveryExecution 作为唯一发布者按周期发布并终态
 * 补零; 插件不自行 sleep、不直接调 NavigationCallbacks。碰撞检测用接收的
 * 不可变 CostmapSnapshot (global grid) 机器人附近 cell >= 253 →
 * FAILED(RecoveryFailed), message 含 "collision detected"。
 * 不持有 context / wrapper / 可变 grid。
 */
class CostmapBackUpRecovery final : public ayanami::costmap::CostmapRecovery {
 public:
  using ayanami::costmap::CostmapRecovery::runRecovery;
  CostmapBackUpRecovery();
  // Ruling #3: 基类 AbstractRecovery 无虚析构, 派生层补裸 virtual ~。
  virtual ~CostmapBackUpRecovery() = default;

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
  struct CostmapBackUpRecoveryParam {
    double backup_dist = 0.2;
    double max_v = 0.2;
    double sim_cycle_dt = 0.01;
    double collision_radius = 0.3;
  };

  CostmapBackUpRecoveryParam param;

  bool collisionNearRobot(const ayanami::costmap::CostmapView& costmap,
                          const ::Eigen::Isometry3d& robot_pose) const;

  ayanami::costmap::CostmapContext::ConstPtr map_;

  std::atomic<bool> canceled{false};
  int remaining_frames{0};
};

}  // namespace modules
}  // namespace ayanami
