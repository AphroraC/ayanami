#pragma once

#include <functional>
#include <vector>

#include <Eigen/Dense>

namespace ayanami {
namespace modules {
namespace corridor {

/// @brief Axis-aligned free rectangle [xmin, ymin, xmax, ymax] (tdt
/// SfcSquare semantics, world coordinates).
using Box = Eigen::Vector4d;

/// @brief Largest free axis-aligned rectangle around the anchor, expanded
/// symmetrically one cell per round: every direction stops at the first
/// blocked sample (boundary rows/columns AND corners, tdt corner rule) or
/// at max_range. worldBlocked(wx, wy) reports the occupancy of the cell
/// containing the point (NO_INFORMATION blocks). The anchor should be
/// snapped to its cell center by the caller so the expansion stays
/// raster-aligned. `shrink` pulls every edge inward while keeping the
/// anchor inside (tdt SfcSquare::shrink). Returns false only when the
/// anchor itself is blocked (a degenerate point box).
bool computeBound(const std::function<bool(double, double)>& world_blocked,
                  double anchor_x, double anchor_y, double max_range,
                  double cell_size, double shrink, Box& out);

/// @brief Pulls the box edges inward by `shrink` (clamped to half extents)
/// while keeping every limit point inside the shrunk box.
Box shrinkBound(const Box& box, double shrink,
                const std::vector<Eigen::Vector2d>& limits);

/// @brief Collision check of the segment a->b by dense sampling at
/// quarter-cell spacing (>= Bresenham cell-walk density).
bool segmentBlocked(const std::function<bool(double, double)>& world_blocked,
                    const Eigen::Vector2d& a, const Eigen::Vector2d& b,
                    double cell_size);

}  // namespace corridor
}  // namespace modules
}  // namespace ayanami
