#pragma once

#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace ayanami {
namespace rosemap {

// Exact squared Euclidean distance transform (Felzenszwalb-Huttenlocher
// lower-envelope algorithm), pure C++ / Eigen-free. This is the canonical
// implementation for the rosemap family.
//
// occupied[i]!=0 seeds distance 0; every other cell receives the squared
// distance (in cells) to the nearest occupied cell. Infinity only when the
// input line carries no occupied cell at all.
inline void edt1d(const std::vector<double>& f, std::vector<double>& d) {
  const double inf = std::numeric_limits<double>::infinity();
  const int n = static_cast<int>(f.size());
  d.assign(n, inf);
  if (n == 0) {
    return;
  }
  // Infinite parabolas never enter the lower envelope: skip them and seed
  // the envelope from the first finite sample.
  int first = -1;
  for (int i = 0; i < n; ++i) {
    if (!std::isinf(f[i])) {
      first = i;
      break;
    }
  }
  if (first < 0) {
    return;
  }
  std::vector<int> v(n, 0);
  std::vector<double> z(n + 1, 0.0);
  int k = 0;
  v[0] = first;
  z[0] = -inf;
  z[1] = inf;
  for (int q = first + 1; q < n; ++q) {
    if (std::isinf(f[q])) {
      continue;
    }
    const double fq = f[q] + static_cast<double>(q) * q;
    while (true) {
      const int vk = v[k];
      const double intersect =
          (fq - (f[vk] + static_cast<double>(vk) * vk)) /
          (2.0 * static_cast<double>(q - vk));
      if (intersect <= z[k]) {
        if (k == 0) {
          v[0] = q;
          z[0] = -inf;
          z[1] = inf;
          break;
        }
        --k;
      } else {
        ++k;
        v[k] = q;
        z[k] = intersect;
        z[k + 1] = inf;
        break;
      }
    }
  }
  k = 0;
  for (int q = 0; q < n; ++q) {
    while (z[k + 1] < q) {
      ++k;
    }
    const int vk = v[k];
    const double dx = static_cast<double>(q - vk);
    d[q] = dx * dx + f[vk];
  }
}

// 2D exact squared EDT: one 1D pass per column, then per row. Returns false
// on shape mismatch; on failure `out` is left untouched.
inline bool squaredEdt(const std::vector<unsigned char>& occupied, int width,
                       int height, std::vector<double>& out) {
  if (width <= 0 || height <= 0 ||
      occupied.size() !=
          static_cast<std::size_t>(width) * static_cast<std::size_t>(height)) {
    return false;
  }
  const double inf = std::numeric_limits<double>::infinity();
  std::vector<double> grid(occupied.size(), inf);
  std::vector<double> f(height);
  std::vector<double> d;
  for (int x = 0; x < width; ++x) {
    for (int y = 0; y < height; ++y) {
      f[y] = occupied[static_cast<std::size_t>(y) * width + x] ? 0.0 : inf;
    }
    edt1d(f, d);
    for (int y = 0; y < height; ++y) {
      grid[static_cast<std::size_t>(y) * width + x] = d[y];
    }
  }
  out.assign(occupied.size(), 0.0);
  f.assign(width, 0.0);
  for (int y = 0; y < height; ++y) {
    const std::size_t row = static_cast<std::size_t>(y) * width;
    for (int x = 0; x < width; ++x) {
      f[x] = grid[row + x];
    }
    edt1d(f, d);
    for (int x = 0; x < width; ++x) {
      out[row + x] = d[x];
    }
  }
  return true;
}

/// @brief 2D signed distance field over a binary occupancy raster
/// (rose-map semantics, rose_navigation/src/map/esdf.cpp concept rebuilt on
/// the exact EDT above): the signed value is
///   signed = d_occ - d_free
/// where d_occ = distance to the nearest occupied cell (0 inside obstacles)
/// and d_free = distance to the nearest free cell (0 in free space). Free
/// space therefore carries d_occ (positive), obstacle interiors carry
/// -d_free (negative), and the field stays continuous across the boundary.
/// Values are clamped to +-max_distance (meters).
/// @param occupied 1 raster, 0 free; row-major, width * height entries.
/// @param out distances in meters; untouched when the shape mismatches.
inline bool signedEdt2d(const std::vector<unsigned char>& occupied, int width,
                        int height, double resolution, double max_distance,
                        std::vector<float>& out) {
  if (resolution <= 0.0 || max_distance <= 0.0) {
    return false;
  }
  std::vector<double> d_occ_sq;
  if (!squaredEdt(occupied, width, height, d_occ_sq)) {
    return false;
  }
  std::vector<unsigned char> inverted(occupied.size());
  for (std::size_t i = 0; i < occupied.size(); ++i) {
    inverted[i] = occupied[i] ? 0 : 1;
  }
  std::vector<double> d_free_sq;
  if (!squaredEdt(inverted, width, height, d_free_sq)) {
    return false;
  }
  out.assign(occupied.size(), 0.0f);
  for (std::size_t i = 0; i < occupied.size(); ++i) {
    const double d_occ = std::sqrt(d_occ_sq[i]) * resolution;
    const double d_free = std::sqrt(d_free_sq[i]) * resolution;
    out[i] = static_cast<float>(
        std::clamp(d_occ - d_free, -max_distance, max_distance));
  }
  return true;
}

}  // namespace rosemap
}  // namespace ayanami
