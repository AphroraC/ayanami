#include "square_corridor.hpp"

#include <algorithm>
#include <cmath>

namespace ayanami {
namespace modules {
namespace corridor {

namespace {
constexpr double k_epsilon = 1e-9;
}

Box shrinkBound(const Box& box, double shrink,
                const std::vector<Eigen::Vector2d>& limits) {
  Box out = box;
  const double shrink_x =
      std::clamp(shrink, 0.0, (out[2] - out[0]) / 2 - 1e-6);
  const double shrink_y =
      std::clamp(shrink, 0.0, (out[3] - out[1]) / 2 - 1e-6);
  out[0] += shrink_x;
  out[2] -= shrink_x;
  out[1] += shrink_y;
  out[3] -= shrink_y;
  for (const auto& limit : limits) {
    out[0] = std::min(out[0], limit.x());
    out[2] = std::max(out[2], limit.x());
    out[1] = std::min(out[1], limit.y());
    out[3] = std::max(out[3], limit.y());
  }
  return out;
}

bool computeBound(const std::function<bool(double, double)>& world_blocked,
                  double anchor_x, double anchor_y, double max_range,
                  double cell_size, double shrink, Box& out) {
  if (cell_size <= 0.0) {
    return false;
  }
  if (world_blocked(anchor_x, anchor_y)) {
    out = {anchor_x, anchor_y, anchor_x, anchor_y};
    return false;
  }
  double left = anchor_x;
  double right = anchor_x;
  double bottom = anchor_y;
  double top = anchor_y;
  // max_range limits each side's expansion from the anchor (box extent
  // per axis is at most 2 * max_range).
  const int max_cells =
      std::isfinite(max_range)
          ? static_cast<int>(std::max(0.0, max_range) / cell_size)
          : 1 << 20;
  for (int round = 0; round < max_cells; ++round) {
    const bool range_left = anchor_x - (left - cell_size) <= max_range + 1e-9;
    const bool range_right = (right + cell_size) - anchor_x <= max_range + 1e-9;
    const bool range_bottom = anchor_y - (bottom - cell_size) <= max_range + 1e-9;
    const bool range_top = (top + cell_size) - anchor_y <= max_range + 1e-9;
    // Column under test spans the closed interval [bottom, top] at cell
    // steps (expansion is cell-stepped, so alignment is preserved).
    bool free_left = true;
    bool free_right = true;
    bool free_bottom = true;
    bool free_top = true;
    for (double y = bottom; y <= top + k_epsilon; y += cell_size) {
      if (free_left && world_blocked(left - cell_size, y)) {
        free_left = false;
      }
      if (free_right && world_blocked(right + cell_size, y)) {
        free_right = false;
      }
      if (!free_left && !free_right) {
        break;
      }
    }
    for (double x = left; x <= right + k_epsilon; x += cell_size) {
      if (free_bottom && world_blocked(x, bottom - cell_size)) {
        free_bottom = false;
      }
      if (free_top && world_blocked(x, top + cell_size)) {
        free_top = false;
      }
      if (!free_bottom && !free_top) {
        break;
      }
    }
    // Corner rule (tdt): a blocked corner closes both adjacent sides.
    if (free_left && free_bottom &&
        world_blocked(left - cell_size, bottom - cell_size)) {
      free_left = false;
      free_bottom = false;
    }
    if (free_left && free_top &&
        world_blocked(left - cell_size, top + cell_size)) {
      free_left = false;
      free_top = false;
    }
    if (free_right && free_bottom &&
        world_blocked(right + cell_size, bottom - cell_size)) {
      free_right = false;
      free_bottom = false;
    }
    if (free_right && free_top &&
        world_blocked(right + cell_size, top + cell_size)) {
      free_right = false;
      free_top = false;
    }
    if ((!free_left || !range_left) && (!free_right || !range_right) &&
        (!free_bottom || !range_bottom) && (!free_top || !range_top)) {
      break;
    }
    if (free_left && range_left) {
      left -= cell_size;
    }
    if (free_right && range_right) {
      right += cell_size;
    }
    if (free_bottom && range_bottom) {
      bottom -= cell_size;
    }
    if (free_top && range_top) {
      top += cell_size;
    }
  }
  out = shrink ? shrinkBound(Box{left, bottom, right, top}, shrink,
                               {Eigen::Vector2d(anchor_x, anchor_y)})
               : Box{left, bottom, right, top};
  return true;
}

bool segmentBlocked(const std::function<bool(double, double)>& world_blocked,
                    const Eigen::Vector2d& a, const Eigen::Vector2d& b,
                    double cell_size) {
  const double length = (b - a).norm();
  const int steps = static_cast<int>(
      std::ceil(length / std::max(0.25 * cell_size, 1e-6)));
  for (int s = 0; s <= steps; ++s) {
    const Eigen::Vector2d p =
        a + (b - a) * (static_cast<double>(s) / std::max(steps, 1));
    if (world_blocked(p.x(), p.y())) {
      return true;
    }
  }
  return false;
}

}  // namespace corridor
}  // namespace modules
}  // namespace ayanami
