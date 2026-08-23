// Task 24: modules/costmap_back_up_recovery — 后退恢复模块。
//
// 纯 C++ (零 ROS): 继承 costmap::CostmapRecovery, 类似 spin 但逐帧 emit
// 负向线速度建议 (vx=-max_v)。每帧 10ms + cancel 检查; 碰撞检测
// (global costmap robot 附近 >= 253) → FailureCode::RecoveryFailed, message 含
// "collision detected"。M1 headless 不依赖底盘实际运动, 发完时长即 SUCCESS。

#include "costmap_back_up_recovery.hpp"

#include <ayanami/interfaces/callbacks.hpp>
#include <ayanami/messages/failure.hpp>
#include <ayanami/utility/config.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>

namespace ayanami {
namespace modules {

namespace {
constexpr int kLethalThreshold = 253;  // Ruling #6
}

CostmapBackUpRecovery::CostmapBackUpRecovery() { loadConfig(); }

void CostmapBackUpRecovery::loadConfig() {
  try {
    const ayanami::ComponentConfig holder;
    const auto& file = holder.get();
    config_.backup_dist =
        file.param<double>("recovery", "backup_dist", config_.backup_dist);
    config_.max_v = file.param<double>("recovery", "max_v", config_.max_v);
    config_.sim_cycle_dt =
        file.param<double>("recovery", "sim_cycle_dt", config_.sim_cycle_dt);
    config_.collision_radius = file.param<double>(
        "recovery", "collision_radius", config_.collision_radius);
  } catch (...) {
    // 配置不可读时保持默认值。
  }
}

void CostmapBackUpRecovery::initialize(
    const std::string& name, ayanami::costmap::CostmapContext::Ptr ctx) {
  (void)name;
  costmap_context_ = std::move(ctx);
  canceled_.store(false);
}

FailureCode CostmapBackUpRecovery::runRecovery(std::string& message) {
  message.clear();
  if (canceled_.load()) {
    message = "back up recovery canceled";
    return FailureCode::Canceled;
  }
  if (!costmap_context_) {
    message = "back up recovery not initialized";
    return FailureCode::InternalError;
  }

  const double dt = std::max(config_.sim_cycle_dt, 1e-6);
  const double total_frames =
      std::max(config_.backup_dist, 0.0) / std::max(config_.max_v, 1e-6) / dt;

  ayanami::geometry_msgs::PoseStamped pose;
  const bool have_pose = costmap_context_->getRobotPose(pose);

  for (int i = 0; i < static_cast<int>(std::ceil(total_frames)); ++i) {
    if (canceled_.load()) {
      emitZeroVelocity();
      message = "back up recovery canceled";
      return FailureCode::Canceled;
    }
    if (have_pose && collisionNearRobot(pose)) {
      emitZeroVelocity();
      message = "collision detected";
      return FailureCode::RecoveryFailed;
    }
    ayanami::geometry_msgs::TwistStamped cmd;
    cmd.header.frame_id = costmap_context_->getGlobalFrame();
    cmd.twist.linear.x() = -config_.max_v;
    ayanami::interfaces::NavigationCallbacks::on_velocity_command(cmd);
    std::this_thread::sleep_for(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::duration<double>(dt)));
  }

  emitZeroVelocity();
  return FailureCode::None;
}

bool CostmapBackUpRecovery::cancel() {
  canceled_.store(true);
  return true;
}

bool CostmapBackUpRecovery::collisionNearRobot(
    const ayanami::geometry_msgs::PoseStamped& pose) const {
  costmap_2d::Costmap2D* grid = costmap_context_->globalCostmap2D();
  if (!grid) {
    return false;
  }
  unsigned int cx = 0;
  unsigned int cy = 0;
  if (!grid->worldToMap(pose.pose.translation().x(),
                        pose.pose.translation().y(), cx, cy)) {
    return false;
  }
  const unsigned int r =
      std::max<unsigned int>(1, grid->cellDistance(config_.collision_radius));
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
                        static_cast<unsigned int>(my)) >= kLethalThreshold) {
        return true;
      }
    }
  }
  return false;
}

void CostmapBackUpRecovery::emitZeroVelocity() const {
  ayanami::geometry_msgs::TwistStamped cmd;
  cmd.header.frame_id = costmap_context_ ? costmap_context_->getGlobalFrame()
                                         : std::string("map");
  ayanami::interfaces::NavigationCallbacks::on_velocity_command(cmd);
}

}  // namespace modules
}  // namespace ayanami
