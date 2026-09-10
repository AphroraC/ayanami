#pragma once

#include <string>
#include <vector>

#include <costmap_2d/cost_values.hpp>

#include "ayanami/messages/geometry_msgs.hpp"

namespace ayanami {
namespace costmap {

/**
 * @brief Materialized planning input: the global+local overlay costs copied
 * into a caller-owned buffer plus the grid geometry needed to query it.
 *
 * Family read contract (planner): composePlanningOverlay() holds the two map
 * shared locks ONLY for the compose pass (bounded, ~O(N) copy); the locks are
 * released when it returns and the search algorithm then queries the buffer
 * lock-free in O(1) per cell. This keeps the writer stall bounded even when
 * the search itself is slow (unreachable goal / large map) -- the old
 * lock-held-for-the-whole-A* contract made "<10 ms" a promise instead of a
 * mechanism.
 *
 * The overlay is expressed in the GLOBAL costmap frame and grid: cell (mx,my)
 * covers world (origin_x + (mx+0.5)*res, origin_y + (my+0.5)*res). Local-map
 * participation is resolved during composition (the local<-global transform
 * is looked up exactly once per compose), so consumers never see frames.
 */
struct PlanningOverlay {
  /// @brief Overlay costs, row-major x-fastest, size_x * size_y entries.
  std::vector<unsigned char> costs;
  unsigned int size_x{0};
  unsigned int size_y{0};
  double resolution{0.05};
  double origin_x{0.0};
  double origin_y{0.0};
  /// @brief The overlay frame = the global costmap frame.
  std::string frame;
  /// @brief Robot pose in the overlay frame, resolved at compose time.
  geometry_msgs::PoseStamped robot_pose;
  bool hasRobotPose{false};
  /// @brief False when the local map could not participate (missing wrapper
  /// or missing local<-global transform): the overlay is global-only and
  /// local obstacles are NOT reflected -- consumers treating the overlay as
  /// safety evidence (isPathValid) must decide whether that is acceptable.
  bool local_overlay_valid{false};
  bool valid{false};

  /// @brief Reset to empty; keeps the costs buffer capacity for reuse.
  void invalidate() {
    costs.clear();
    size_x = 0;
    size_y = 0;
    valid = false;
    local_overlay_valid = false;
    hasRobotPose = false;
    robot_pose = geometry_msgs::PoseStamped{};
  }

  bool worldToMap(double wx, double wy, unsigned int& mx,
                  unsigned int& my) const {
    if (resolution <= 0.0 || size_x == 0 || size_y == 0) {
      return false;
    }
    if (wx < origin_x || wy < origin_y) {
      return false;
    }
    const double fx = (wx - origin_x) / resolution;
    const double fy = (wy - origin_y) / resolution;
    const unsigned int ix = static_cast<unsigned int>(fx);
    const unsigned int iy = static_cast<unsigned int>(fy);
    if (ix >= size_x || iy >= size_y) {
      return false;
    }
    mx = ix;
    my = iy;
    return true;
  }

  void mapToWorld(unsigned int mx, unsigned int my, double& wx,
                  double& wy) const {
    wx = origin_x + (static_cast<double>(mx) + 0.5) * resolution;
    wy = origin_y + (static_cast<double>(my) + 0.5) * resolution;
  }

  unsigned char costAt(unsigned int mx, unsigned int my) const {
    return costs[static_cast<std::size_t>(my) * size_x + mx];
  }

  /// @brief Blocking check on the overlay (>= INSCRIBED, matching A* rules).
  bool blockedCell(unsigned int mx, unsigned int my) const {
    return costAt(mx, my) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
  }
};

}  // namespace costmap
}  // namespace ayanami
