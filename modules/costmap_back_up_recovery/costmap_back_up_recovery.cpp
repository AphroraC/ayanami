// modules/costmap_back_up_recovery -- back-up recovery module.
//
// Pure C++ (zero ROS): unified recovery protocol (start/tick/finish). Each cycle returns a negative linear velocity
// suggestion (vx = -max_v); RecoveryExecution acts as the sole publisher, publishing per cycle and zeroing
// on the final state; the plugin neither loops/waits on its own nor bypass-publishes velocity commands. M1 headless does not depend on
// actual base motion; SUCCESS is returned once duration has elapsed. Collision detection (lock-held global grid read,
// any cell >= 253 near the robot) -> FAILED(RecoveryFailed), message contains "collision detected".

#include "costmap_back_up_recovery.hpp"

#include <algorithm>
#include <cmath>

namespace ayanami {
namespace modules {

namespace {
constexpr int k_lethal_threshold = 253;  //
}

void CostmapBackUpRecovery::configure(
    const ayanami::PluginContext& context,
    ayanami::costmap::CostmapMapContext::ConstPtr map) {
  // Single source of parameters: the recovery section of this module's json (the file config_recovery points to).
  param.backup_dist =
      context.config.param<double>(context.name, "backup_dist", param.backup_dist);
  param.max_v =
      context.config.param<double>(context.name, "max_v", param.max_v);
  param.sim_cycle_dt =
      context.config.param<double>(
          "recovery", "sim_cycle_dt", param.sim_cycle_dt);
  param.collision_radius = context.config.param<double>(
      context.name, "collision_radius", param.collision_radius);
  map_context = std::move(map);
}

void CostmapBackUpRecovery::setBehaviorParams(
    const ayanami::RecoveryBehaviorParams& params) {
  // nullopt fields keep the configure-time defaults (interpreting nullopt as 0 is forbidden).
  if (params.backup_distance) {
    param.backup_dist = *params.backup_distance;
  }
  if (params.backup_speed) {
    param.max_v = *params.backup_speed;
  }
}

void CostmapBackUpRecovery::start() {
  canceled.store(false);
  const double dt = std::max(param.sim_cycle_dt, 1e-6);
  remaining_frames = static_cast<int>(std::ceil(
      std::max(param.backup_dist, 0.0) / std::max(param.max_v, 1e-6) / dt));
}

ResultT<void> CostmapBackUpRecovery::tick(RecoveryCycleData& cycle) {
  // Hold the shared lock to read the map (released at the end of the cycle), zero copy.
  auto costmap = map_context->lockGlobal();
  const std::string frame =
      costmap.frame().empty() ? std::string("map") : costmap.frame();

  if (canceled.load()) {
    return ResultT<void>::failure(ErrorCode::CANCELED,
                                    "back up recovery canceled");
  }

  ::Eigen::Isometry3d robot_pose = ::Eigen::Isometry3d::Identity();
  const bool have_pose =
      costmap.has_robot_pose() &&
      (robot_pose = costmap.robot_pose().pose.toIsometry(), true);
  if (have_pose && collisionNearRobot(costmap, robot_pose)) {
    return ResultT<void>::failure(ErrorCode::RECOVERY_FAILED,
                                    "collision detected");
  }

  if (remaining_frames <= 0) {
      cycle.status = RecoveryStatus::SUCCESS;
    return ResultT<void>::success();
  }
  --remaining_frames;
  cycle.status = RecoveryStatus::RUNNING;
  ayanami::geometry_msgs::TwistStamped velocity;
  velocity.header.frame_id = frame;
  velocity.twist.linear.x() = -param.max_v;
  cycle.velocity = std::move(velocity);
  return ResultT<void>::success();
}

bool CostmapBackUpRecovery::cancel() {
  canceled.store(true);
  return true;
}

bool CostmapBackUpRecovery::collisionNearRobot(
    const ayanami::costmap::CostmapView& costmap,
    const ::Eigen::Isometry3d& robot_pose) const {
  const costmap_2d::Costmap2D* grid = costmap.grid();
  if (!grid) {
    return false;
  }
  unsigned int cx = 0;
  unsigned int cy = 0;
  if (!grid->worldToMap(robot_pose.translation().x(),
                        robot_pose.translation().y(), cx, cy)) {
    return false;
  }
  const unsigned int r =
      std::max<unsigned int>(1, grid->cellDistance(param.collision_radius));
  const int sx = static_cast<int>(grid->getSizeInCellsX());
  const int sy = static_cast<int>(grid->getSizeInCellsY());
  for (int dy = -static_cast<int>(r); dy <= static_cast<int>(r); ++dy) {
    for (int dx = -static_cast<int>(r); dx <= static_cast<int>(r); ++dx) {
      const int mx = static_cast<int>(cx) + dx;
      const int my = static_cast<int>(cy) + dy;
      if (mx < 0 || my < 0 || mx >= sx || my >= sy) {
        continue;
      }
      if (grid->getCost(static_cast<unsigned int>(mx),
                        static_cast<unsigned int>(my)) >= k_lethal_threshold) {
        return true;
      }
    }
  }
  return false;
}

}  // namespace modules
}  // namespace ayanami
