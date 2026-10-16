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
 * @brief Rotate-in-place recovery module (costmap_spin_recovery) -- the
 * nav2 Spin entry of the recovery rotation.
 *
 * Unified recovery protocol (start/tick/finish). Each cycle returns a pure
 * angular velocity suggestion (angular.z = max_w); RecoveryExecution is
 * the sole publisher (per-cycle publish, zero on the terminal state); the
 * module never sleeps or publishes on its own. Collision detection uses
 * the lock-held read view of the global grid: any cell >= 253 within
 * collision_radius of the robot -> FAILED (spinning inside the inscribed
 * halo is a physical impossibility, not something to attempt).
 */
class CostmapSpinRecovery final : public ayanami::costmap::CostmapRecovery {
 public:
  CostmapSpinRecovery() = default;
  virtual ~CostmapSpinRecovery() = default;

  void configureParameters(const ayanami::ModuleContext& context) override;

  void configure(const ayanami::ModuleContext& context,
                 ayanami::costmap::CostmapMapContext::ConstPtr map,
                 ClearCommandFn clear_command) override;

  void setCyclePeriod(double seconds) override;
  void start() override;
  void setBehaviorParams(
      const ayanami::RecoveryBehaviorParams& params) override;
  ResultT<void> tick(RecoveryCycleData& cycle) override;

 private:
  ResultT<void> tickWithMap(RecoveryCycleData& cycle,
                            const ayanami::costmap::CostmapReader& costmap);

 public:
  void finish() override {}
  bool cancel() override;

 private:
  struct CostmapSpinRecoveryParam {
    double spin_dist = 1.57;
    double max_w = 0.5;
    double collision_radius = 0.3;
  };

  CostmapSpinRecoveryParam param;

  bool collisionNearRobot(const ayanami::costmap::CostmapReader& costmap,
                          const ::Eigen::Isometry3d& robot_pose) const;

  ayanami::costmap::CostmapMapContext::ConstPtr map_context;

  double cycle_dt{0.01};
  std::atomic<bool> canceled{false};
  int remaining_frames{0};
};

}  // namespace modules
}  // namespace ayanami
