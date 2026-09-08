#pragma once

#include <string>
#include <vector>

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
 * read in place under their locks -- consistency snapshots are forbidden.
 *
 * Both grids share one map frame, so a global cell maps onto the local grid
 * through plain world<->cell conversions only (no frame transform).
 *
 * Overlay rule (conservative, per global-grid cell):
 * cost = max(global_cost, local_cost) where the local value participates
 * only when the point falls inside the local window; NO_INFORMATION never
 * downgrades (unknown stays unknown -- local free space cannot clear global
 * unknown space).
 *
 * Hot-path note: per-node queries (costAtGlobalCell) are meant for sparse
 * probes (start/goal checks); grid-search algorithms should compose the
 * overlay once per call via composeOverlay() into a caller-owned buffer and
 * query the buffer in O(1).
 */
class CostmapPlanningView {
 public:
  CostmapPlanningView() = default;
  CostmapPlanningView(CostmapPlanningView&&) = default;
  CostmapPlanningView& operator=(CostmapPlanningView&&) = default;
  CostmapPlanningView(const CostmapPlanningView&) = delete;
  CostmapPlanningView& operator=(const CostmapPlanningView&) = delete;

  CostmapPlanningView(CostmapView global, CostmapView local)
      : global_view(std::move(global)),
        local_view(std::move(local)),
        has_local(local_view.valid() && global_view.valid()) {}

  /// @brief Valid when the global grid is readable (local is optional).
  bool valid() const { return global_view.valid(); }

  /// @brief Global master grid (search geometry: sizes/worldToMap/mapToWorld).
  const costmap_2d::Costmap2D* globalGrid() const { return global_view.grid(); }

  const std::string& globalFrame() const { return global_view.frame(); }

  /// @brief Robot pose resolved in the map frame at view construction.
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
    const auto* local_grid = local_view.grid();
    unsigned int lx = 0;
    unsigned int ly = 0;
    if (local_grid == nullptr ||
        !local_grid->worldToMap(wx, wy, lx, ly)) {
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

  /// @brief Compose the full overlay into a caller-owned row-major buffer
  /// (global grid dimensions, one entry per global cell, row-major x-fastest).
  /// One O(N) pass under the already-held locks; call it once per planning
  /// call and query the buffer instead of doing per-node overlay lookups.
  /// The buffer is cleared when the view is invalid.
  void composeOverlay(std::vector<unsigned char>& buffer) const {
    const auto* global_grid = global_view.grid();
    if (!global_grid) {
      buffer.clear();
      return;
    }
    const unsigned int size_x = global_grid->getSizeInCellsX();
    const unsigned int size_y = global_grid->getSizeInCellsY();
    buffer.resize(static_cast<std::size_t>(size_x) * size_y);
    for (unsigned int my = 0; my < size_y; ++my) {
      for (unsigned int mx = 0; mx < size_x; ++mx) {
        buffer[static_cast<std::size_t>(my) * size_x + mx] =
            costAtGlobalCell(mx, my);
      }
    }
  }

 private:
  CostmapView global_view;
  CostmapView local_view;
  bool has_local{false};
};

}  // namespace costmap
}  // namespace ayanami
