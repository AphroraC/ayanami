// Task 24: modules/costmap_back_up_recovery — 后退恢复模块。
//
// 纯 C++ (零 ROS): 周期驱动接口 (start/onCycleUpdate)。每周期返回负向线速度
// 建议 (vx = -max_v), 由 RecoveryExecution 作为唯一发布者按周期发布并终态
// 补零; 插件不自行循环等待、不直接旁路发布速度命令。M1 headless 不依赖
// 底盘实际运动, 发完 duration 即 SUCCESS。碰撞检测 (不可变 global 快照,
// 机器人附近 >= 253) → FAILED(RecoveryFailed), message 含 "collision detected"。

#include "costmap_back_up_recovery.hpp"

#include <ayanami/messages/failure.hpp>
#include <ayanami/utility/config.hpp>

#include <algorithm>
#include <cmath>

namespace ayanami {
namespace modules {

namespace {
constexpr int k_lethal_threshold = 253;  // Ruling #6
}

CostmapBackUpRecovery::CostmapBackUpRecovery() {
  // 参数在构造时经 glim 式显式 Config 读取并拷贝进成员 (无 thread_local 注入,
  // 配置不可读时保持默认值): 主 config.json 的 global 单层索引
  // config_back_up_recovery → 本插件 json 文件。
  try {
    const Config file(GlobalConfig::getConfigPath("config_back_up_recovery"));
    param.backup_dist =
        file.param<double>("recovery", "backup_dist", param.backup_dist);
    param.max_v = file.param<double>("recovery", "max_v", param.max_v);
    param.sim_cycle_dt =
        file.param<double>("recovery", "sim_cycle_dt", param.sim_cycle_dt);
    param.collision_radius = file.param<double>(
        "recovery", "collision_radius", param.collision_radius);
  } catch (...) {
    // 配置不可读时保持默认值。
  }
}

void CostmapBackUpRecovery::initialize(const std::string& name,
                                       ayanami::costmap::CostmapContext::ConstPtr map) {
  (void)name;
  map_ = std::move(map);
}

void CostmapBackUpRecovery::setBehaviorParams(
    const ayanami::RecoveryBehaviorParams& params) {
  // nullopt 字段保留 configure 期默认值 (禁止把 nullopt 解释为 0)。
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

ayanami::abstract::RecoveryCycleResult CostmapBackUpRecovery::onCycleUpdate(
    std::string& message) {
  message.clear();
  // 持 shared 锁读图 (整个周期结束释放), 零拷贝。
  auto costmap = map_->lockGlobal();

  ayanami::abstract::RecoveryCycleResult r;  // 默认 RUNNING + 零速度。
  const std::string frame =
      costmap.frame().empty() ? std::string("map") : costmap.frame();

  if (canceled.load()) {
    r.status = ayanami::abstract::RecoveryCycleResult::Status::CANCELED;
    r.failure = FailureCode::Canceled;
    message = "back up recovery canceled";
    return r;
  }

  ::Eigen::Isometry3d robot_pose = ::Eigen::Isometry3d::Identity();
  const bool have_pose =
      costmap.has_robot_pose() && [&] {
        robot_pose = costmap.robot_pose().pose.toIsometry();
        return true;
      }();
  if (have_pose && collisionNearRobot(costmap, robot_pose)) {
    r.status = ayanami::abstract::RecoveryCycleResult::Status::FAILED;
    r.failure = FailureCode::RecoveryFailed;
    message = "collision detected";
    return r;
  }

  if (remaining_frames <= 0) {
    r.status = ayanami::abstract::RecoveryCycleResult::Status::SUCCESS;
    return r;
  }
  --remaining_frames;
  r.velocity.header.frame_id = frame;
  r.velocity.twist.linear.x() = -param.max_v;
  return r;
}

void CostmapBackUpRecovery::end() {}

FailureCode CostmapBackUpRecovery::runRecovery(std::string& message) {
  start();
  while (true) {
    const auto r = onCycleUpdate(message);
    switch (r.status) {
      case ayanami::abstract::RecoveryCycleResult::Status::RUNNING:
        continue;
      case ayanami::abstract::RecoveryCycleResult::Status::SUCCESS:
        return FailureCode::None;
      case ayanami::abstract::RecoveryCycleResult::Status::CANCELED:
        return FailureCode::Canceled;
      case ayanami::abstract::RecoveryCycleResult::Status::FAILED:
      default:
        return r.failure;
    }
  }
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
