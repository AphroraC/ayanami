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
 * @brief Back-up recovery module (costmap_back_up_recovery).
 *
 * Pure C++ (zero ROS): unified recovery protocol (start/tick/finish). Each cycle returns a negative linear velocity
 * suggestion (vx = -max_v); RecoveryExecution acts as the sole publisher, publishing per cycle and zeroing
 * on the final state; the module neither sleeps on its own nor calls NavigationCallbacks directly. Collision detection uses the
 * lock-held read view of the map module: if any global grid cell near the robot is >= 253 -> FAILED(RecoveryFailed),
 * message contains "collision detected". Holds no context / wrapper / mutable grid.
 * Parameters are uniformly read from ModuleContext.config (the module json that config_recovery points to).
 */
class CostmapBackUpRecovery final : public ayanami::costmap::CostmapRecovery {
 public:
  CostmapBackUpRecovery() = default;
  // The derived layer adds a bare virtual ~ so that shared_ptr deletion goes through the virtual destructor.
  virtual ~CostmapBackUpRecovery() = default;

  /// @brief Parameter-loading seam (ExtensionModule): called by the loader
  /// right after a successful load, and by configure() for callers that build
  /// the object directly.
  void configureParameters(const ayanami::ModuleContext& context) override;

  void configure(const ayanami::ModuleContext& context,
                 ayanami::costmap::CostmapMapContext::ConstPtr map) override;

  void setCyclePeriod(double seconds) override;

  void start() override;
  void setBehaviorParams(
      const ayanami::RecoveryBehaviorParams& params) override;
  ResultT<void> tick(RecoveryCycleData& cycle) override;

 private:
  /// @brief tick() body with the map already read-locked for its duration.
  ResultT<void> tickWithMap(RecoveryCycleData& cycle,
                            const ayanami::costmap::CostmapReader& costmap);

 public:
  void finish() override {}

  bool cancel() override;

 private:
  struct CostmapBackUpRecoveryParam {
    double backup_dist = 0.2;
    double max_v = 0.2;
    double collision_radius = 0.3;
  };

  CostmapBackUpRecoveryParam param;

  /// Escape-progress bookkeeping: start position of the running escape and
  /// whether it was recorded; the run fails as "escape stalled" when the
  /// robot did not physically move by half the backup distance.
  ::Eigen::Vector2d escape_start_pos_{::Eigen::Vector2d::Zero()};
  bool escape_progress_valid_{false};
  /// Two-leg escape: the first stalled leg flips the direction once before
  /// the run is declared stalled (boxed in).
  int escape_total_frames_{0};
  bool tried_opposite_dir_{false};

  /// Any lethal cell within radius_m (meters) of the robot center.
  bool lethalWithin(const ayanami::costmap::CostmapReader& costmap,
                    const ::Eigen::Isometry3d& robot_pose,
                    double radius_m) const;

  /// Split the lethal cells around the pose into front/rear (robot frame)
  /// and pick the escape direction away from the heavier contact side.
  /// Returns false when there is no lethal cell to branch on.
  bool pickEscapeDirection(const ayanami::costmap::CostmapReader& costmap,
                           const ::Eigen::Isometry3d& robot_pose,
                           double& escape_dir, double contact_radius) const;

  ayanami::costmap::CostmapMapContext::ConstPtr map_context;

  /// @brief Tick cadence pushed by the execution before each run (s).
  double cycle_dt{0.01};
  std::atomic<bool> canceled{false};
  int remaining_frames{0};
};

}  // namespace modules
}  // namespace ayanami
