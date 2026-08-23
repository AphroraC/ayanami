// Task 24: modules/costmap_clear_costmap_recovery — 清代价地图恢复模块。
//
// 纯 C++ (零 ROS): 继承 costmap::CostmapRecovery, 清除 global/local
// (按 affected_maps) 动态障碍层，不破坏 global 静态地图。

#include <ayanami/modules/costmap_clear_costmap_recovery/costmap_clear_costmap_recovery.hpp>

#include <ayanami/messages/failure.hpp>
#include <ayanami/utility/config.hpp>

#include <costmap_2d_layers/simple_obstacle_layer.hpp>
#include <costmap_2d_layers/static_layer.hpp>

#include <algorithm>
#include <cmath>

namespace ayanami {
namespace modules {

CostmapClearCostmapRecovery::CostmapClearCostmapRecovery() { loadConfig(); }

void CostmapClearCostmapRecovery::loadConfig() {
  try {
    const ayanami::ComponentConfig holder;
    const auto& file = holder.get();
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

FailureCode CostmapClearCostmapRecovery::runRecovery(std::string& message) {
  message.clear();
  if (canceled_.load()) {
    message = "clear costmap recovery canceled";
    return FailureCode::Canceled;
  }
  if (!costmap_context_) {
    message = "clear costmap recovery not initialized";
    return FailureCode::InternalError;
  }

  const bool clear_global =
      config_.affected_maps == "global" || config_.affected_maps == "both";
  const bool clear_local =
      config_.affected_maps == "local" || config_.affected_maps == "both";

  if (clear_global) {
    if (!clearCostmap(costmap_context_->global_costmap_,
                      costmap_context_->global_frame_)) {
      message = "clear costmap recovery could not resolve global robot pose";
      return FailureCode::TransformUnavailable;
    }
  }
  if (clear_local) {
    const std::string local_frame = costmap_context_->local_costmap_
                                        ? costmap_context_->local_costmap_->getGlobalFrame()
                                        : std::string{};
    if (!clearCostmap(costmap_context_->local_costmap_, local_frame)) {
      message = "clear costmap recovery could not resolve local robot pose";
      return FailureCode::TransformUnavailable;
    }
  }

  return FailureCode::None;
}

bool CostmapClearCostmapRecovery::cancel() {
  canceled_.store(true);
  return true;
}

bool CostmapClearCostmapRecovery::clearCostmap(
    const costmap_2d::CostmapWrapper::Ptr& wrapper,
    const std::string& frame) const {
  if (!wrapper) {
    return true;
  }
  if (!costmap_context_->transform_store_ || frame.empty()) {
    return false;
  }

  Eigen::Isometry3d robot_pose = Eigen::Isometry3d::Identity();
  if (!costmap_context_->transform_store_->lookupTransform(
          frame, costmap_context_->robot_frame_, ayanami::Timestamp::latest(),
          robot_pose)) {
    return false;
  }

  auto* grid = wrapper->get().getCostmap();
  if (!grid) {
    return true;
  }
  const double robot_x = robot_pose.translation().x();
  const double robot_y = robot_pose.translation().y();
  const double radius = config_.clear_radius > 0.0
                            ? config_.clear_radius
                            : std::max(grid->getSizeInMetersX(),
                                       grid->getSizeInMetersY());
  const double min_x = robot_x - radius;
  const double min_y = robot_y - radius;
  const double max_x = robot_x + radius;
  const double max_y = robot_y + radius;

  for (const auto& layer : wrapper->get().getPlugins()) {
    if (auto* static_layer =
            dynamic_cast<costmap_2d_layers::StaticLayer*>(layer.get())) {
      static_layer->addUpdateBounds(min_x, min_y, max_x, max_y);
    }
    if (auto* obstacle_layer =
            dynamic_cast<costmap_2d_layers::SimpleObstacleLayer*>(layer.get())) {
      obstacle_layer->clearArea(min_x, min_y, max_x, max_y);
    }
  }
  const double yaw =
      std::atan2(robot_pose.linear()(1, 0), robot_pose.linear()(0, 0));
  wrapper->updateMap(robot_x, robot_y, yaw);
  return true;
}

}  // namespace modules
}  // namespace ayanami
