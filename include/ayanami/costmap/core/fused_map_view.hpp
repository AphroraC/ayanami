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
 *
 * Hot-path form: grid-walk probes run inside the innermost loops of the
 * A* search and the MPPI trajectory scoring (hundreds of probes per
 * expansion / rollout), where the world-coordinate roundtrip per probe
 * (mapToWorld -> rigid transform -> worldToMap, two divisions each) is the
 * dominant constant. prepare() folds that chain into six per-view
 * coefficients so a probe is two dot products in CELL space plus integer
 * bounds checks; callers wire the view fields and call prepare() once. An
 * unprepared view falls back to the original world-space path (identical
 * results -- see fused_map_view_test's randomized parity lock).
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

  /// @brief Fold the world-space probe chain into cell-space coefficients
  /// (call once after wiring global/local/local_from_global). Must be
  /// re-run if any referenced snapshot or the transform changes.
  void prepare() {
    if (global == nullptr || local == nullptr) {
      prepared = false;
      return;
    }
    const double r00 = local_from_global(0, 0);
    const double r01 = local_from_global(0, 1);
    const double r10 = local_from_global(1, 0);
    const double r11 = local_from_global(1, 1);
    const double tx = local_from_global.translation().x();
    const double ty = local_from_global.translation().y();
    const double res_g = global->resolution;
    const double res_l = local->resolution;
    // lx_cells = (r00*wx + r01*wy + tx - l.ox)/l.res with wx/wy the global
    // cell-center world point; expanded so mx/my appear linearly.
    ax = r00 * res_g / res_l;
    bx = r01 * res_g / res_l;
    ay = r10 * res_g / res_l;
    by = r11 * res_g / res_l;
    const double hx =
        (r00 * global->origin_x + r01 * global->origin_y + tx -
         local->origin_x) /
        res_l;
    const double hy =
        (r10 * global->origin_x + r11 * global->origin_y + ty -
         local->origin_y) /
        res_l;
    cx = hx + 0.5 * (ax + bx);
    cy = hy + 0.5 * (ay + by);
    prepared = true;
  }

  /// @brief The view is usable when the global grid is present and valid;
  /// the local half is optional by design.
  bool valid() const { return global != nullptr && global->valid(); }

  /// @brief Fused cost at a global-grid cell (the cell walk interface):
  /// max(global cell, local cell under the same world point when the point
  /// falls inside the local window).
  unsigned char costAtCell(unsigned int mx, unsigned int my) const {
    unsigned char cost = global->getCost(mx, my);
    if (local == nullptr || !local->valid()) {
      return cost;
    }
    unsigned int lmx = 0;
    unsigned int lmy = 0;
    if (prepared) {
      if (!localCell(mx, my, lmx, lmy)) {
        return cost;
      }
    } else {
      double wx = 0.0;
      double wy = 0.0;
      global->mapToWorld(mx, my, wx, wy);
      const Eigen::Vector3d p_local =
          local_from_global * Eigen::Vector3d(wx, wy, 0.0);
      if (!local->worldToMap(p_local.x(), p_local.y(), lmx, lmy)) {
        return cost;
      }
    }
    const unsigned char local_cost = local->getCost(lmx, lmy);
    if (local_cost > cost) {
      cost = local_cost;
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

 private:
  /// @brief Local-cell lookup through the prepared coefficients. The
  /// negative guards mirror CostmapSnapshot::worldToMap's `wx < origin`
  /// rejection; the trunc cast equals floor for the non-negative values
  /// that pass those guards.
  bool localCell(unsigned int mx, unsigned int my, unsigned int& lmx,
                 unsigned int& lmy) const {
    const double fx = ax * static_cast<double>(mx) +
                      bx * static_cast<double>(my) + cx;
    const double fy = ay * static_cast<double>(mx) +
                      by * static_cast<double>(my) + cy;
    if (!(fx >= 0.0) || !(fy >= 0.0)) {
      return false;
    }
    lmx = static_cast<unsigned int>(fx);
    lmy = static_cast<unsigned int>(fy);
    return lmx < local->size_x && lmy < local->size_y;
  }

  /// @brief Set by prepare(); false keeps the world-space probe path.
  bool prepared{false};
  /// @brief Cell-space affine: lmx = ax*mx + bx*my + cx (local cells, floor
  /// semantics), lmy = ay*mx + by*my + cy.
  double ax{0.0};
  double bx{0.0};
  double cx{0.0};
  double ay{0.0};
  double by{0.0};
  double cy{0.0};
};

}  // namespace costmap
}  // namespace ayanami
