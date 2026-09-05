#pragma once

#include <atomic>
#include <memory>
#include <string>

#include <Eigen/Geometry>

#include <ayanami/costmap/core/costmap_recovery.hpp>
#include <ayanami/messages/recovery_params.hpp>
#include <ayanami/messages/geometry_msgs.hpp>
#include <ayanami/types.hpp>

namespace ayanami {
namespace modules {

/**
 * @brief 后退恢复模块 (spec §6.6 costmap_back_up_recovery)。
 *
 * 纯 C++ (零 ROS): 统一恢复协议 (start/tick/finish)。每周期返回负向线速度
 * 建议 (vx = -max_v), 由 RecoveryExecution 作为唯一发布者按周期发布并终态
 * 补零; 插件不自行 sleep、不直接调 NavigationCallbacks。碰撞检测用地图模块
 * 加锁读的 global grid 机器人附近 cell >= 253 → FAILED(RecoveryFailed),
 * message 含 "collision detected"。不持有 context / wrapper / 可变 grid。
 * 参数统一从 PluginContext.config (config_recovery 指向的本模块 json) 读取。
 */
class CostmapBackUpRecovery final : public ayanami::costmap::CostmapRecovery {
 public:
  CostmapBackUpRecovery() = default;
  // Ruling #3: 派生层补裸 virtual ~ 让 shared_ptr 删除走虚析构。
  virtual ~CostmapBackUpRecovery() = default;

  void configure(const ayanami::PluginContext& context,
                 ayanami::costmap::CostmapMapContext::ConstPtr map) override;

  void start() override;
  void setBehaviorParams(
      const ayanami::RecoveryBehaviorParams& params) override;
  RecoveryResultT tick() override;
  void finish() override {}

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

  ayanami::costmap::CostmapMapContext::ConstPtr map_context;

  std::atomic<bool> canceled{false};
  int remaining_frames{0};
};

}  // namespace modules
}  // namespace ayanami
