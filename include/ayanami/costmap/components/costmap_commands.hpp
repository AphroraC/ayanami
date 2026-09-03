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
 * @brief 写者级清图实现: 在 wrapper 的写锁内, 以 robot (该 map frame 下)
 * 为中心的 radius 内清空动态障碍层, 并让 StaticLayer 重算更新窗口, 最后
 * updateMapUnlocked 重算 master grid (调用方已持写锁)。
 *
 * 调用方 (唯一写者 = CostmapNavigationServer / 等价测试写者) 负责解析 robot
 * pose 到 wrapper 的 frame 并保证单写者。
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

  for (const auto& layer : wrapper->get().getPlugins()) {
    if (auto* static_layer =
            dynamic_cast<costmap_2d_layers::StaticLayer*>(layer.get())) {
      static_layer->addUpdateBounds(min_x, min_y, max_x, max_y);
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
