#pragma once

#include <string>
#include <vector>

#include "ayanami/messages/geometry_msgs.hpp"

namespace ayanami {
namespace costmap {

/**
 * @brief Short-lock copy of one master grid: the planner's lock-free map
 * input.
 *
 * The map's shared lock is held only for the memcpy inside
 * CostmapMapContext::snapshotGlobal(); the search runs on this immutable
 * copy afterwards, so the map writer is never delayed by planning and no
 * reader contract ("<10 ms per read call") has to be defended against the
 * search duration. World<->cell conversions mirror costmap_2d::Costmap2D
 * exactly (floor-based worldToMap, +0.5 cell-center mapToWorld, row-major
 * index = y * size_x + x).
 */
struct CostmapSnapshot {
  unsigned int size_x{0};
  unsigned int size_y{0};
  double resolution{0.05};
  double origin_x{0.0};
  double origin_y{0.0};
  /// @brief The frame the copied grid is expressed in.
  std::string frame;
  /// @brief Robot pose in that frame, resolved BEFORE the copy lock was
  /// taken (never a transform lookup under a map lock); check
  /// has_robot_pose before use.
  ayanami::geometry_msgs::PoseStamped robot_pose;
  bool has_robot_pose{false};
  /// @brief Raw master-grid costs, row-major (index = y * size_x + x).
  std::vector<unsigned char> costs;

  bool valid() const {
    return size_x > 0 && size_y > 0 && costs.size() == static_cast<std::size_t>(size_x) * size_y;
  }

  /// @brief World -> cell; false when outside the snapshot bounds
  /// (mirrors costmap_2d::Costmap2D::worldToMap).
  bool worldToMap(double wx, double wy, unsigned int& mx,
                  unsigned int& my) const {
    if (wx < origin_x || wy < origin_y || resolution <= 0.0) {
      return false;
    }
    mx = static_cast<unsigned int>((wx - origin_x) / resolution);
    my = static_cast<unsigned int>((wy - origin_y) / resolution);
    return mx < size_x && my < size_y;
  }

  /// @brief Cell -> world cell center (mirrors Costmap2D::mapToWorld).
  void mapToWorld(unsigned int mx, unsigned int my, double& wx,
                  double& wy) const {
    wx = origin_x + (static_cast<double>(mx) + 0.5) * resolution;
    wy = origin_y + (static_cast<double>(my) + 0.5) * resolution;
  }

  unsigned int getIndex(unsigned int mx, unsigned int my) const {
    return my * size_x + mx;
  }

  unsigned char getCost(unsigned int mx, unsigned int my) const {
    return costs[static_cast<std::size_t>(my) * size_x + mx];
  }
};

}  // namespace costmap
}  // namespace ayanami
