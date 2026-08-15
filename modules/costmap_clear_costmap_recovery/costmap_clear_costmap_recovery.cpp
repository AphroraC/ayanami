// Task 24: modules/costmap_clear_costmap_recovery — 清代价地图恢复模块。
//
// 纯 C++ (零 ROS): 继承 costmap::CostmapRecovery, 清除 global/local
// (按 affected_maps) master grid 中 robot 附近 radius 范围内 cell →
// FREE_SPACE(0)。直接改 master grid (clearArea 语义), 不需要底盘。

#include <ayanami/modules/costmap_clear_costmap_recovery/costmap_clear_costmap_recovery.hpp>

#include <ayanami/messages/actions.hpp>
#include <ayanami/utility/config.hpp>

#include <algorithm>
#include <cmath>

namespace ayanami {
namespace modules {

CostmapClearCostmapRecovery::CostmapClearCostmapRecovery() { loadConfig(); }

void CostmapClearCostmapRecovery::loadConfig() {
  try {
    const ayanami::Config file(
        ayanami::GlobalConfig::get_config_path(
            "costmap_clear_costmap_recovery"));
    config_.clear_radius =
        file.param<double>("recovery", "clear_radius", config_.clear_radius);
    config_.affected_maps = file.param<std::string>(
        "recovery", "affected_maps", config_.affected_maps);
  } catch (...) {
    // 配置不可读时保持默认值。
  }
}

void CostmapClearCostmapRecovery::initialize(
    const std::string& name, ayanami::costmap::CostmapContext::Ptr ctx) {
  (void)name;
  costmap_context_ = std::move(ctx);
  canceled_.store(false);
}

uint32_t CostmapClearCostmapRecovery::runRecovery(std::string& message) {
  message.clear();
  if (canceled_.load()) {
    message = "clear costmap recovery canceled";
    return ayanami::actions::RecoveryResult::CANCELED;
  }
  if (!costmap_context_) {
    message = "clear costmap recovery not initialized";
    return ayanami::actions::RecoveryResult::INTERNAL_ERROR;
  }

  ayanami::geometry_msgs::PoseStamped pose;
  const bool have_pose = costmap_context_->getRobotPose(pose);
  const double robot_x = have_pose ? pose.pose.translation().x() : 0.0;
  const double robot_y = have_pose ? pose.pose.translation().y() : 0.0;

  const bool clear_global =
      config_.affected_maps == "global" || config_.affected_maps == "both";
  const bool clear_local =
      config_.affected_maps == "local" || config_.affected_maps == "both";

  if (clear_global) {
    clearCostmap(costmap_context_->globalCostmap2D(), robot_x, robot_y);
  }
  if (clear_local) {
    clearCostmap(costmap_context_->localCostmap2D(), robot_x, robot_y);
  }

  return ayanami::actions::RecoveryResult::SUCCESS;
}

bool CostmapClearCostmapRecovery::cancel() {
  canceled_.store(true);
  return true;
}

void CostmapClearCostmapRecovery::clearCostmap(
    costmap_2d::Costmap2D* grid, double robot_x, double robot_y) const {
  if (!grid) {
    return;
  }
  if (config_.clear_radius <= 0.0) {
    grid->resetMap();
    return;
  }
  unsigned int cx = 0;
  unsigned int cy = 0;
  if (!grid->worldToMap(robot_x, robot_y, cx, cy)) {
    grid->resetMap();
    return;
  }
  const unsigned int r = grid->cellDistance(config_.clear_radius);
  const int sx = static_cast<int>(grid->getSizeInCellsX());
  const int sy = static_cast<int>(grid->getSizeInCellsY());
  const unsigned int x0 = std::max(0, static_cast<int>(cx) - static_cast<int>(r));
  const unsigned int y0 = std::max(0, static_cast<int>(cy) - static_cast<int>(r));
  const unsigned int x1 =
      std::min(sx, static_cast<int>(cx) + static_cast<int>(r) + 1);
  const unsigned int y1 =
      std::min(sy, static_cast<int>(cy) + static_cast<int>(r) + 1);
  grid->resetMapToValue(x0, y0, x1, y1, costmap_2d::FREE_SPACE);
}

}  // namespace modules
}  // namespace ayanami
