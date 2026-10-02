#pragma once

#include <Eigen/Geometry>

#include <costmap/cost_values.hpp>

#include "ayanami/costmap/core/costmap_snapshot.hpp"

namespace ayanami {
namespace costmap {

/**
 * @brief Global+local overlay view for planning-time collision queries.
 *
 * The planning read model of the static-global policy: the global grid
 * (static + inflation, held as an immutable copy) provides the map-level
 * geometry, and the local rolling snapshot provides fresh dynamic obstacle
 * marks. The two cost sources are disjoint (a static wall never carries a
 * dynamic mark and vice versa, each grid carries its own inflation), so the
 * fused cost is simply the per-point MAX -- no double-counted inflation.
 *
 * The view is read-only and holds nothing: it references a caller-owned
 * global snapshot (must outlive the view) and an optional caller-owned
 * local snapshot (null => global-only degradation). The map<-odom transform
 * is resolved ONCE by the caller (CostmapMapContext::localFromGlobal,
 * outside any map lock) instead of per probe.
 *
 * Probing is dual-mode: cell-indexed for grid walks (A* expansion, nearest
 * valid goal) and world-point for sparse checks (path validation). The
 * world point is expressed in the GLOBAL frame; local probing transforms it
 * into the local grid frame and skips points outside the local window.
 */
struct FusedMapView {
  /// @brief The global grid (planning timescale). Must be non-null and
  /// valid(); out-of-bounds world probes report NO_INFORMATION.
  const CostmapSnapshot* global{nullptr};
  /// @brief Optional local overlay (perception timescale). Null, invalid,
  /// or transform-less degrades every probe to global-only.
  const CostmapSnapshot* local{nullptr};
  /// @brief Transform of global-frame points into the local grid frame
  /// (p_local = local_from_global * p_global); meaningful only when local
  /// is set.
  Eigen::Isometry3d local_from_global{Eigen::Isometry3d::Identity()};

  /// @brief The view is usable when the global grid is present and valid;
  /// the local half is optional by design.
  bool valid() const { return global != nullptr && global->valid(); }

  /// @brief Fused cost at a global-grid cell (the cell walk interface):
  /// max(global cell, local cell under the same world point when the point
  /// falls inside the local window).
  unsigned char costAtCell(unsigned int mx, unsigned int my) const {
    double wx = 0.0;
    double wy = 0.0;
    global->mapToWorld(mx, my, wx, wy);
    unsigned char cost = global->getCost(mx, my);
    if (local == nullptr || !local->valid()) {
      return cost;
    }
    const Eigen::Vector3d p_local =
        local_from_global * Eigen::Vector3d(wx, wy, 0.0);
    unsigned int lmx = 0;
    unsigned int lmy = 0;
    if (local->worldToMap(p_local.x(), p_local.y(), lmx, lmy)) {
      const unsigned char local_cost = local->getCost(lmx, lmy);
      if (local_cost > cost) {
        cost = local_cost;
      }
    }
    return cost;
  }

  /// @brief Fused cost at an arbitrary global-frame world point (the sparse
  /// check interface): NO_INFORMATION outside the global grid (blocked
  /// under the isLethal >= 253 convention), mirroring the planner's
  /// out-of-bounds handling.
  unsigned char costAt(double wx, double wy) const {
    unsigned int mx = 0;
    unsigned int my = 0;
    if (!global->worldToMap(wx, wy, mx, my)) {
      return costmap_2d::NO_INFORMATION;
    }
    return costAtCell(mx, my);
  }

  /// @brief Blocking predicate at a global-grid cell (>= inscribed/lethal).
  bool blockedAtCell(unsigned int mx, unsigned int my) const {
    return costAtCell(mx, my) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
  }

  /// @brief Blocking predicate at a global-frame world point.
  bool blockedAt(double wx, double wy) const {
    return costAt(wx, wy) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
  }
};

}  // namespace costmap
}  // namespace ayanami
