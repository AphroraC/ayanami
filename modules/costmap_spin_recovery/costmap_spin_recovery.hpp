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
 * @brief Rotate-in-place recovery module (costmap_spin_recovery).
 *
 * Pure C++ (zero ROS): unified recovery protocol (start/tick/finish). Each cycle returns a pure angular velocity
 * suggestion (angular.z = max_w); RecoveryExecution acts as the sole publisher, publishing per cycle and
 * zeroing on the final state; the plugin neither sleeps on its own nor calls NavigationCallbacks directly.
 * Collision detection uses the lock-held read view of the map module: if any global grid cell near the robot is >= 253
 * -> FAILED(RecoveryFailed). Holds no context / wrapper / mutable grid.
 * Parameters are uniformly read from PluginContext.config (the module json that config_recovery points to).
 */
class CostmapSpinRecovery final : public ayanami::costmap::CostmapRecovery {
 public:
  CostmapSpinRecovery() = default;
  // The derived layer adds a bare virtual ~ so that shared_ptr deletion goes through the virtual destructor.
  virtual ~CostmapSpinRecovery() = default;

  void configure(const ayanami::PluginContext& context,
                 ayanami::costmap::CostmapMapContext::ConstPtr map) override;

  void start() override;
  void setBehaviorParams(
      const ayanami::RecoveryBehaviorParams& params) override;
  ResultT<void> tick(RecoveryCycleData& cycle) override;
  void finish() override {}

  bool cancel() override;

 private:
  struct CostmapSpinRecoveryParam {
    double spin_dist = 1.57;
    double max_w = 0.5;
    double sim_cycle_dt = 0.01;
    double collision_radius = 0.3;
  };

  CostmapSpinRecoveryParam param;

  bool collisionNearRobot(const ayanami::costmap::CostmapView& costmap,
                          const ::Eigen::Isometry3d& robot_pose) const;

  ayanami::costmap::CostmapMapContext::ConstPtr map_context;

  std::atomic<bool> canceled{false};
  int remaining_frames{0};
};

}  // namespace modules
}  // namespace ayanami
