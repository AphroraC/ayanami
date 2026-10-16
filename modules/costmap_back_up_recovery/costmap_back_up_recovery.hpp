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
 * @brief Back-up recovery module (costmap_back_up_recovery) -- the nav2
 * BackUp entry of the recovery rotation, with the double-band escape
 * semantics:
 * - contact belt (0.5*collision_radius + one cell): escape AWAY from the
 *   heavier contact side, no map-based refusal (around a touching robot
 *   the map is the least trustworthy input);
 * - near belt (1.5*collision_radius + one cell), not touching: refuse
 *   only when backing up leads TOWARD the near obstacle (a robot parked
 *   facing a wall must still be able to back away).
 * Escape validity is physical displacement: a stalled leg flips the
 * direction once; both directions stalled = honestly boxed -> FAILED.
 */
class CostmapBackUpRecovery final : public ayanami::costmap::CostmapRecovery {
 public:
  CostmapBackUpRecovery() = default;
  virtual ~CostmapBackUpRecovery() = default;

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
  struct CostmapBackUpRecoveryParam {
    double backup_dist = 0.35;
    double max_v = 0.3;
    double collision_radius = 0.3;
  };

  CostmapBackUpRecoveryParam param;

  ::Eigen::Vector2d escape_start_pos_{::Eigen::Vector2d::Zero()};
  bool escape_progress_valid_{false};
  int escape_total_frames_{0};
  bool tried_opposite_dir_{false};

  bool lethalWithin(const ayanami::costmap::CostmapReader& costmap,
                    const ::Eigen::Isometry3d& robot_pose,
                    double radius_m) const;
  bool pickEscapeDirection(const ayanami::costmap::CostmapReader& costmap,
                           const ::Eigen::Isometry3d& robot_pose,
                           double& escape_dir, double contact_radius) const;

  ayanami::costmap::CostmapMapContext::ConstPtr map_context;

  double cycle_dt{0.01};
  std::atomic<bool> canceled{false};
  int remaining_frames{0};
};

}  // namespace modules
}  // namespace ayanami
