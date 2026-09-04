// Task 24: modules/costmap_spin_recovery — 原地旋转恢复模块。
//
// 纯 C++ (零 ROS): 统一恢复协议 (start/tick/finish)。每周期返回纯角速度
// 建议 (angular.z = max_w), 由 RecoveryExecution 作为唯一发布者按周期发布并
// 在终态补零。插件不自行循环等待、不直接旁路发布速度命令。M1 headless
// 无真底盘: 不依赖底盘实际运动, 发完 spin_dist/max_w 时长即 SUCCESS。
// 碰撞检测 (不可变 global 快照, 机器人附近 >= 253) → FAILED(RecoveryFailed)。

#include "costmap_spin_recovery.hpp"

#include <ayanami/utility/config.hpp>

#include <algorithm>
#include <cmath>

namespace ayanami {
namespace modules {

namespace {
constexpr int k_lethal_threshold = 253;  // Ruling #6: inscribed inflated 起 lethal

bool poseToIsometry(const ayanami::geometry_msgs::PoseStamped& pose,
                    ::Eigen::Isometry3d& out) {
  out = pose.pose.toIsometry();
  return true;
}
}

CostmapSpinRecovery::CostmapSpinRecovery() {
  // 参数在构造时经 glim 式显式 Config 读取并拷贝进成员 (无 thread_local 注入,
  // 配置不可读时保持默认值): 主 config.json 的 global 单层索引
  // config_spin_recovery → 本插件 json 文件。
  try {
    const Config file(GlobalConfig::getConfigPath("config_spin_recovery"));
    param.spin_dist = file.param<double>("recovery", "spin_dist", param.spin_dist);
    param.max_w = file.param<double>("recovery", "max_w", param.max_w);
    param.sim_cycle_dt =
        file.param<double>("recovery", "sim_cycle_dt", param.sim_cycle_dt);
    param.collision_radius = file.param<double>(
        "recovery", "collision_radius", param.collision_radius);
  } catch (...) {
    // 配置不可读时保持默认值。
  }
}

void CostmapSpinRecovery::initialize(const std::string& name,
                                     ayanami::costmap::CostmapContext::ConstPtr map) {
  (void)name;
  map_ = std::move(map);
}

void CostmapSpinRecovery::setBehaviorParams(
    const ayanami::RecoveryBehaviorParams& params) {
  // nullopt 字段保留 configure 期默认值 (禁止把 nullopt 解释为 0)。
  if (params.target_yaw) {
    param.spin_dist = *params.target_yaw;
  }
}

void CostmapSpinRecovery::start() {
  canceled.store(false);
  const double dt = std::max(param.sim_cycle_dt, 1e-6);
  remaining_frames = static_cast<int>(std::ceil(
      std::max(param.spin_dist, 0.0) / std::max(param.max_w, 1e-6) / dt));
}

RecoveryResultT CostmapSpinRecovery::tick() {
  // 持 shared 锁读图 (整个周期结束释放), 零拷贝。
  auto costmap = map_->lockGlobal();
  const std::string frame =
      costmap.frame().empty() ? std::string("map") : costmap.frame();

  if (canceled.load()) {
    return RecoveryResultT::failure(ErrorCode::CANCELED,
                                    "spin recovery canceled");
  }

  // 只取一次机器人位姿 (M1 headless 无底盘运动, 视为静态; 由读视图解析)。
  ::Eigen::Isometry3d robot_pose = ::Eigen::Isometry3d::Identity();
  const bool have_pose =
      costmap.has_robot_pose() &&
      poseToIsometry(costmap.robot_pose(), robot_pose);
  if (have_pose && collisionNearRobot(costmap, robot_pose)) {
    return RecoveryResultT::failure(ErrorCode::RECOVERY_FAILED,
                                    "collision detected during spin");
  }

  if (remaining_frames <= 0) {
    RecoveryCycleData data;
    data.status = RecoveryStatus::SUCCESS;
    return RecoveryResultT::success(std::move(data));
  }
  --remaining_frames;
  RecoveryCycleData data;
  data.status = RecoveryStatus::RUNNING;
  ayanami::geometry_msgs::TwistStamped velocity;
  velocity.header.frame_id = frame;
  velocity.twist.angular.z() = param.max_w;
  data.velocity = std::move(velocity);
  return RecoveryResultT::success(std::move(data));
}

bool CostmapSpinRecovery::cancel() {
  canceled.store(true);
  return true;
}

bool CostmapSpinRecovery::collisionNearRobot(
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
