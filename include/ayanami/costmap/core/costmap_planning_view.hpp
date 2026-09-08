#pragma once

#include <string>

#include <Eigen/Geometry>

#include <costmap_2d/costmap_2d.hpp>

#include "ayanami/costmap/core/costmap_view.hpp"
#include "ayanami/messages/geometry_msgs.hpp"

namespace ayanami {
namespace costmap {

/**
 * @brief Planning overlay read view (family read contract: the planner reads
 * the superposition of global+local content).
 *
 * Holds the shared locks of both maps simultaneously (RAII; acquisition
 * order local -> global, matching the writer; the writer never nests the
 * two locks, so no lock-ordering cycle can form). Zero copy: both grids are
 * read in place under their locks -- COW/deep-copy snapshots are forbidden.
 *
 * Overlay rule (conservative, per global-grid cell):
 * cost = max(global_cost, local_cost) where the local value participates
 * only when the point falls inside the local window; NO_INFORMATION never
 * downgrades (unknown stays unknown -- local free space cannot clear global
 * unknown space).
 */
class CostmapPlanningView {
 public:
  CostmapPlanningView() = default;
  CostmapPlanningView(CostmapPlanningView&&) = default;
  CostmapPlanningView& operator=(CostmapPlanningView&&) = default;
  CostmapPlanningView(const CostmapPlanningView&) = delete;
  CostmapPlanningView& operator=(const CostmapPlanningView&) = delete;

  CostmapPlanningView(CostmapView global, CostmapView local,
                      Eigen::Isometry3d local_from_global, bool local_ready)
      : global_view(std::move(global)),
        local_view(std::move(local)),
        local_from_global(std::move(local_from_global)),
        has_local(local_ready && local_view.valid() && global_view.valid()) {}

  /// @brief Valid when the global grid is readable (local is optional).
  bool valid() const { return global_view.valid(); }

  /// @brief Global master grid (search geometry: sizes/worldToMap/mapToWorld).
  const costmap_2d::Costmap2D* globalGrid() const { return global_view.grid(); }

  const std::string& globalFrame() const { return global_view.frame(); }

  /// @brief Robot pose resolved in the global frame at view construction.
  const geometry_msgs::PoseStamped& robotPose() const {
    return global_view.robot_pose();
  }
  bool hasRobotPose() const { return global_view.has_robot_pose(); }

  /// @brief Overlay cost at a global-grid cell (cell must be in bounds).
  unsigned char costAtGlobalCell(unsigned int mx, unsigned int my) const {
    const auto* global_grid = global_view.grid();
    const unsigned char global_cost = global_grid->getCost(mx, my);
    if (!has_local) {
      return global_cost;
    }
    double wx = 0.0;
    double wy = 0.0;
    global_grid->mapToWorld(mx, my, wx, wy);
    const Eigen::Vector3d local_point =
        local_from_global * Eigen::Vector3d(wx, wy, 0.0);
    const auto* local_grid = local_view.grid();
    unsigned int lx = 0;
    unsigned int ly = 0;
    if (local_grid == nullptr ||
        !local_grid->worldToMap(local_point.x(), local_point.y(), lx, ly)) {
      return global_cost;  // outside the local window: global only
    }
    const unsigned char local_cost = local_grid->getCost(lx, ly);
    if (local_cost == costmap_2d::NO_INFORMATION) {
      return global_cost;  // unknown never downgrades
    }
    return global_cost > local_cost ? global_cost : local_cost;
  }

  /// @brief Blocking check on the overlay (>= INSCRIBED, matching A* rules).
  bool blockedAtGlobalCell(unsigned int mx, unsigned int my) const {
    return costAtGlobalCell(mx, my) >=
           costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
  }

 private:
  CostmapView global_view;
  CostmapView local_view;
  Eigen::Isometry3d local_from_global{Eigen::Isometry3d::Identity()};
  bool has_local{false};
};

}  // namespace costmap
}  // namespace ayanami
