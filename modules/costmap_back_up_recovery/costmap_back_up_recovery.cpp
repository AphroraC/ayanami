// modules/costmap_back_up_recovery -- back-up recovery module.
//
// Pure C++ (zero ROS): unified recovery protocol (start/tick/finish). Each cycle returns a linear velocity
// suggestion (vx = -max_v normally; when in contact with an obstacle, max_v away from the heavier contact side);
// RecoveryExecution acts as the sole publisher, publishing per cycle and zeroing
// on the final state; the module neither loops/waits on its own nor bypass-publishes velocity commands. M1 headless does not depend on
// actual base motion; SUCCESS is returned once duration has elapsed. Collision detection (lock-held global grid read,
// any cell >= 253 near the robot) -> FAILED(RecoveryFailed), message contains "collision detected". While in contact,
// the neighborhood check is meaningless, so the refusal tests only the swept escape path outside the contact area.

#include "costmap_back_up_recovery.hpp"

#include <algorithm>
#include <cmath>

namespace ayanami {
namespace modules {

namespace {
constexpr int k_lethal_threshold = 253;  //
}


void CostmapBackUpRecovery::configureParameters(const ayanami::ModuleContext& context) {
  // Parameter loading, separated from configuration: the loader calls this
  // right after a successful load, and every value lands in the module's own
  // CostmapBackUpRecoveryParam member (whose field initializers are the defaults for keys the
  // config omits).
  // Single source of parameters: the recovery section of this module's json (the file config_recovery points to).
  param.backup_dist =
      context.config.param<double>(context.name, "backup_dist", param.backup_dist);
  param.max_v =
      context.config.param<double>(context.name, "max_v", param.max_v);
  param.collision_radius = context.config.param<double>(
      context.name, "collision_radius", param.collision_radius);
}

void CostmapBackUpRecovery::configure(
    const ayanami::ModuleContext& context,
    ayanami::costmap::CostmapMapContext::ConstPtr map) {
  // Direct-construction callers (tests, or a host that builds the object
  // itself) never go through the loader's configureParameters call, so bind
  // the parameters here as well. Re-reading the same keys from the same
  // config is idempotent.
  configureParameters(context);
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

void CostmapBackUpRecovery::setCyclePeriod(double seconds) {
  // Framework input, not a config value: the execution owns the tick cadence.
  cycle_dt = std::max(seconds, 1e-6);
}

void CostmapBackUpRecovery::start() {
  canceled.store(false);
  escape_progress_valid_ = false;
  tried_opposite_dir_ = false;
  const double dt = std::max(cycle_dt, 1e-6);
  remaining_frames = static_cast<int>(std::ceil(
      std::max(param.backup_dist, 0.0) / std::max(param.max_v, 1e-6) / dt));
  escape_total_frames_ = remaining_frames;
}

ResultT<void> CostmapBackUpRecovery::tick(RecoveryCycleData& cycle) {
  // The read callback owns the shared lock for its duration, zero copy: the map
  // is readable for exactly this cycle and the writer is never held off longer.
  return map_context->readGlobal(
      [&](const ayanami::costmap::CostmapReader& costmap) -> ResultT<void> {
        return tickWithMap(cycle, costmap);
      });
}

ResultT<void> CostmapBackUpRecovery::tickWithMap(
    RecoveryCycleData& cycle,
    const ayanami::costmap::CostmapReader& costmap) {
  const std::string frame =
      costmap.frame().empty() ? std::string("map") : costmap.frame();

  if (canceled.load()) {
    return ResultT<void>::failure(ErrorCode::CANCELED,
                                    "back up recovery canceled");
  }

  ::Eigen::Isometry3d robot_pose = ::Eigen::Isometry3d::Identity();
  const bool have_pose =
      costmap.hasRobotPose() &&
      (robot_pose = costmap.robot_pose().pose.toIsometry(), true);
  // Two belts decide the mode (radii: half collision_radius + one cell for
  // contact, 1.5 collision_radius + one cell for the near-obstacle safety):
  // - touching (lethal inside the contact belt): escape AWAY from the
  //   contact side. NO map-based refusal here -- around a touching robot
  //   the map is the least trustworthy input (clear/re-mark races,
  //   footprint self-marks), and refusing the escape deadlocks exactly the
  //   case the escape exists for. Physical progress is the verdict instead
  //   (stall check below).
  // - near but not touching: the old safety semantics -- refuse rather than
  //   back into a known obstacle.
  const double cell = costmap.grid() ? costmap.grid()->getResolution() : 0.05;
  const double contact_radius = param.collision_radius * 0.5 + cell;
  const double near_radius = param.collision_radius * 1.5 + cell;
  const bool in_contact =
      have_pose && lethalWithin(costmap, robot_pose, contact_radius);
  if (!in_contact && have_pose &&
      lethalWithin(costmap, robot_pose, near_radius)) {
    // Direction-aware safety: refuse only when backing up leads TOWARD the
    // near obstacle. A robot parked FACING a near wall must still be able
    // to back away -- a blanket refusal deadlocks it (the planner cannot
    // start inside the inflation zone either).
    double near_dir = -1.0;
    if (!pickEscapeDirection(costmap, robot_pose, near_dir, near_radius) ||
        near_dir > 0.0) {
      return ResultT<void>::failure(ErrorCode::RECOVERY_FAILED,
                                    "collision detected");
    }
  }
  double escape_dir = -1.0;
  if (in_contact &&
      !pickEscapeDirection(costmap, robot_pose, escape_dir, contact_radius)) {
    // Contact detected but the belt scan could not branch on it (pose off
    // the grid): fall back to the classic back-up direction.
    escape_dir = -1.0;
  }
  if (!escape_progress_valid_) {
    escape_start_pos_ = robot_pose.translation().head<2>();
    escape_progress_valid_ = true;
  }

  if (remaining_frames <= 0) {
    // Progress verdict per leg: the escape only counts if the robot
    // physically moved. A stalled leg means that direction is wedged --
    // flip once to the opposite direction; both stalled = honestly boxed.
    const double moved = (robot_pose.translation().head<2>() -
                          escape_start_pos_)
                             .norm();
    if (moved >= 0.5 * param.backup_dist) {
      escape_progress_valid_ = false;
      cycle.status = RecoveryStatus::SUCCESS;
      return ResultT<void>::success();
    }
    if (!tried_opposite_dir_) {
      tried_opposite_dir_ = true;
      escape_dir = -escape_dir;
      remaining_frames = escape_total_frames_;
      escape_start_pos_ = robot_pose.translation().head<2>();
    } else {
      escape_progress_valid_ = false;
      cycle.status = RecoveryStatus::FAILED;
      return ResultT<void>::failure(ErrorCode::RECOVERY_FAILED,
                                    "escape stalled (no motion)");
    }
  }
  --remaining_frames;
  cycle.status = RecoveryStatus::RUNNING;
  ayanami::geometry_msgs::TwistStamped velocity;
  velocity.header.frame_id = frame;
  velocity.twist.linear.x() = escape_dir * param.max_v;
  cycle.velocity = std::move(velocity);
  return ResultT<void>::success();
}

bool CostmapBackUpRecovery::cancel() {
  canceled.store(true);
  return true;
}

bool CostmapBackUpRecovery::lethalWithin(
    const ayanami::costmap::CostmapReader& costmap,
    const ::Eigen::Isometry3d& robot_pose, double radius_m) const {
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
  // Meter-exact: the scan ring is generous, the verdict compares the CELL
  // CENTER distance (cell quantization must not blur the contact/near
  // belt distinction on coarse grids).
  const unsigned int r =
      std::max<unsigned int>(1, grid->cellDistance(radius_m) + 1);
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
                        static_cast<unsigned int>(my)) < k_lethal_threshold) {
        continue;
      }
      double wx = 0.0;
      double wy = 0.0;
      grid->mapToWorld(static_cast<unsigned int>(mx),
                       static_cast<unsigned int>(my), wx, wy);
      if (std::hypot(wx - robot_pose.translation().x(),
                     wy - robot_pose.translation().y()) <= radius_m) {
        return true;
      }
    }
  }
  return false;
}

bool CostmapBackUpRecovery::pickEscapeDirection(
    const ayanami::costmap::CostmapReader& costmap,
    const ::Eigen::Isometry3d& robot_pose, double& escape_dir,
    double contact_radius) const {
  const costmap_2d::Costmap2D* grid = costmap.grid();
  if (!grid) {
    return false;
  }
  unsigned int mx = 0;
  unsigned int my = 0;
  const ::Eigen::Vector3d& start = robot_pose.translation();
  if (!grid->worldToMap(start.x(), start.y(), mx, my)) {
    return false;
  }
  const double yaw =
      std::atan2(robot_pose.linear()(1, 0), robot_pose.linear()(0, 0));
  const double fwd_x = std::cos(yaw);
  const double fwd_y = std::sin(yaw);
  const unsigned int r = std::max<unsigned int>(1, grid->cellDistance(contact_radius));
  int front_lethal = 0;
  int rear_lethal = 0;
  for (int ddy = -static_cast<int>(r); ddy <= static_cast<int>(r); ++ddy) {
    for (int ddx = -static_cast<int>(r); ddx <= static_cast<int>(r); ++ddx) {
      const int cx = static_cast<int>(mx) + ddx;
      const int cy = static_cast<int>(my) + ddy;
      if (cx < 0 || cy < 0 ||
          cx >= static_cast<int>(grid->getSizeInCellsX()) ||
          cy >= static_cast<int>(grid->getSizeInCellsY())) {
        continue;
      }
      if (grid->getCost(static_cast<unsigned int>(cx),
                        static_cast<unsigned int>(cy)) <
          k_lethal_threshold) {
        continue;
      }
      double wx = 0.0;
      double wy = 0.0;
      grid->mapToWorld(static_cast<unsigned int>(cx),
                       static_cast<unsigned int>(cy), wx, wy);
      const double along =
          (wx - start.x()) * fwd_x + (wy - start.y()) * fwd_y;
      if (along >= 0.0) {
        ++front_lethal;
      } else {
        ++rear_lethal;
      }
    }
  }
  if (front_lethal <= 0 && rear_lethal <= 0) {
    return false;
  }
  // Escape away from the heavier contact side.
  escape_dir = rear_lethal > front_lethal ? 1.0 : -1.0;
  return true;
}

}  // namespace modules
}  // namespace ayanami
