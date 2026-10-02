#pragma once

#include <chrono>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace esdfmap {

/// @brief One immutable ESDF snapshot (the package's read currency).
///
/// The writer builds a fresh snapshot every rebuild cycle and publishes it
/// by an atomic shared_ptr swap into EsdfMap; readers hold the shared_ptr
/// for as long as they need the field -- zero locks on the read path, the
/// writer is never delayed by a reader. A published snapshot is never
/// mutated (ayanami concurrency discipline; the rose_navigation original
/// let planner and map threads race on the same grid).
///
/// Content: a 2D signed distance grid in meters, row-major, cell (x, y) at
/// index y * size_x + x, cell center at world
///   (origin_x + (x + 0.5) * resolution, origin_y + (y + 0.5) * resolution).
/// Positive = distance to the nearest obstacle in free space, negative =
/// penetration depth inside an obstacle, clamped to +-max_distance.
///
/// Freshness: `sequence` increments on every published rebuild (an equal
/// sequence means byte-identical content); `received` is the steady-clock
/// publish instant (age-gating consumers).
struct EsdfSnapshot {
  std::string frame;
  double resolution{0.05};
  int size_x{0};
  int size_y{0};
  double origin_x{0.0};
  double origin_y{0.0};
  double max_distance{3.0};
  std::uint64_t sequence{0};
  std::chrono::steady_clock::time_point received{
      std::chrono::steady_clock::now()};
  /// @brief Signed distance grid (meters).
  std::vector<float> distance;
  /// @brief Binarized occupancy raster of the same shape (1 = obstacle):
  /// the visualization + test surface (distance alone cannot distinguish
  /// "obstacle here" from "clamp plateau").
  std::vector<unsigned char> occupied;

  bool valid() const {
    return size_x > 0 && size_y > 0 &&
           distance.size() ==
               static_cast<std::size_t>(size_x) *
                   static_cast<std::size_t>(size_y);
  }

  /// @brief World -> continuous cell coordinates (cell centers land on
  /// integers). Returns false when the point is outside the field.
  bool worldToMap(double wx, double wy, double& mx, double& my) const {
    mx = (wx - origin_x) / resolution - 0.5;
    my = (wy - origin_y) / resolution - 0.5;
    return mx >= 0.0 && my >= 0.0 &&
           mx <= static_cast<double>(size_x - 1) &&
           my <= static_cast<double>(size_y - 1);
  }

  /// @brief Nearest-cell read without bounds checking (internal fast path).
  float distanceAt(int x, int y) const {
    return distance[static_cast<std::size_t>(y) * static_cast<std::size_t>(
                                                       size_x) +
                    static_cast<std::size_t>(x)];
  }
};

}  // namespace esdfmap
