#pragma once

#include <cmath>
#include <shared_mutex>

#include <Eigen/Geometry>

#include <costmap_2d_layers/simple_obstacle_layer.hpp>
#include <costmap_2d_layers/static_layer.hpp>
#include <costmap_server/costmap_wrapper.hpp>

namespace ayanami {
namespace costmap {

/**
 * @brief Writer-level map clearing implementation: inside the wrapper's
 * write lock, clear the dynamic obstacle layer within radius centered on
 * the robot (in that map's frame), let the StaticLayer recompute its
 * update window, and finally updateMapUnlocked recomputes the master grid
 * (the caller already holds the write lock).
 *
 * The caller (the sole writer = CostmapNavigationServer / an equivalent
 * test writer) is responsible for resolving the robot pose into the
 * wrapper's frame and guaranteeing single-writer access.
 */
inline bool clearCostmapArea(const costmap_2d::CostmapWrapper::Ptr& wrapper,
                             const Eigen::Isometry3d& robot_pose,
                             double radius) {
  if (!wrapper) {
    return false;
  }
  std::unique_lock<std::shared_mutex> lock(wrapper->mutex());
  auto* grid = wrapper->get().getCostmap();
  if (!grid) {
    return true;
  }
  const double robot_x = robot_pose.translation().x();
  const double robot_y = robot_pose.translation().y();
  const double clear_radius =
      radius > 0.0 ? radius
                   : std::max(grid->getSizeInMetersX(),
                              grid->getSizeInMetersY());
  const double min_x = robot_x - clear_radius;
  const double min_y = robot_y - clear_radius;
  const double max_x = robot_x + clear_radius;
  const double max_y = robot_y + clear_radius;

  for (const auto& layer : wrapper->get().getModules()) {
    // Base-class dirty protocol: the cleared region must be repainted from
    // layer state on the next cycle (extra bounds work for every layer).
    layer->addExtraBounds(min_x, min_y, max_x, max_y);
    if (auto* obstacle_layer =
            dynamic_cast<costmap_2d_layers::SimpleObstacleLayer*>(
                layer.get())) {
      obstacle_layer->clearArea(min_x, min_y, max_x, max_y);
    }
    if (auto* obstacle_layer =
            dynamic_cast<costmap_2d_layers::SimpleObstacleLayer*>(
                layer.get())) {
      obstacle_layer->clearArea(min_x, min_y, max_x, max_y);
    }
  }
  const double yaw =
      std::atan2(robot_pose.linear()(1, 0), robot_pose.linear()(0, 0));
  wrapper->updateMapUnlocked(robot_x, robot_y, yaw);
  return true;
}

}  // namespace costmap
}  // namespace ayanami
