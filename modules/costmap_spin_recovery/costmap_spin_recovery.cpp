// modules/costmap_spin_recovery -- rotate-in-place recovery plugin.
//
// Unified recovery protocol; each cycle suggests a pure angular velocity;
// the execution layer is the sole publisher. SUCCESS once spin_dist/max_w
// seconds of cycles have elapsed; any inscribed-halo lethal cell near the
// robot -> FAILED("collision detected during spin").

#include "costmap_spin_recovery.hpp"

#include <ayanami/utility/config.hpp>

#include <algorithm>
#include <cmath>

namespace ayanami {
namespace modules {

namespace {
constexpr int k_lethal_threshold = 253;  // inscribed inflated counts as lethal
}

void CostmapSpinRecovery::configureParameters(
    const ayanami::ModuleContext& context) {
  param.spin_dist =
      context.config.param<double>(context.name, "spin_dist", param.spin_dist);
  param.max_w =
      context.config.param<double>(context.name, "max_w", param.max_w);
  param.collision_radius = context.config.param<double>(
      context.name, "collision_radius", param.collision_radius);
}

void CostmapSpinRecovery::configure(
    const ayanami::ModuleContext& context,
    ayanami::costmap::CostmapMapContext::ConstPtr map,
    ClearCommandFn clear_command) {
  configureParameters(context);
  (void)clear_command;  // spin reads the map; it never clears
  map_context = std::move(map);
}

void CostmapSpinRecovery::setBehaviorParams(
    const ayanami::RecoveryBehaviorParams& params) {
  // nullopt fields keep the configure-time defaults (interpreting nullopt
  // as 0 is forbidden).
  if (params.target_yaw) {
    param.spin_dist = *params.target_yaw;
  }
}

void CostmapSpinRecovery::setCyclePeriod(double seconds) {
  cycle_dt = std::max(seconds, 1e-6);
}

void CostmapSpinRecovery::start() {
  canceled.store(false);
  const double dt = std::max(cycle_dt, 1e-6);
  remaining_frames = static_cast<int>(std::ceil(
      std::max(param.spin_dist, 0.0) / std::max(param.max_w, 1e-6) / dt));
}

ResultT<void> CostmapSpinRecovery::tick(RecoveryCycleData& cycle) {
  return map_context->readGlobal(
      [&](const ayanami::costmap::CostmapReader& costmap) -> ResultT<void> {
        return tickWithMap(cycle, costmap);
      });
}

ResultT<void> CostmapSpinRecovery::tickWithMap(
    RecoveryCycleData& cycle,
    const ayanami::costmap::CostmapReader& costmap) {
  const std::string frame =
      costmap.frame().empty() ? std::string("map") : costmap.frame();

  if (canceled.load()) {
    return ResultT<void>::failure(ErrorCode::CANCELED,
                                  "spin recovery canceled");
  }

  ::Eigen::Isometry3d robot_pose = ::Eigen::Isometry3d::Identity();
  const bool have_pose =
      costmap.hasRobotPose() &&
      (robot_pose = costmap.robot_pose().pose.toIsometry(), true);
  if (have_pose && collisionNearRobot(costmap, robot_pose)) {
    return ResultT<void>::failure(ErrorCode::RECOVERY_FAILED,
                                  "collision detected during spin");
  }

  if (remaining_frames <= 0) {
    cycle.status = RecoveryStatus::SUCCESS;
    return ResultT<void>::success();
  }
  --remaining_frames;
  cycle.status = RecoveryStatus::RUNNING;
  ayanami::geometry_msgs::TwistStamped velocity;
  velocity.header.frame_id = frame;
  velocity.twist.angular.z() = param.max_w;
  cycle.velocity = std::move(velocity);
  return ResultT<void>::success();
}

bool CostmapSpinRecovery::cancel() {
  canceled.store(true);
  return true;
}

bool CostmapSpinRecovery::collisionNearRobot(
    const ayanami::costmap::CostmapReader& costmap,
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
