#pragma once

#include <algorithm>
#include <cmath>

#include "esdfmap/esdf_snapshot.hpp"

namespace esdfmap {

enum class EsdfQueryStatus {
  kOk,
  kOutOfField,
};

/// @brief One ESDF query result: distance + analytic gradient.
///
/// The gradient is the package-owned interpolation derivative (quadratic
/// Lagrange weights on the 3x3 neighborhood) -- consumers must NOT
/// finite-difference the field themselves: one interpolation contract keeps
/// every consumer's gradient consistent (planner clearance terms, smoother
/// penalty terms, safety probes).
struct EsdfQueryResult {
  double distance{0.0};
  double gradient_x{0.0};
  double gradient_y{0.0};
  EsdfQueryStatus status{EsdfQueryStatus::kOk};
};

namespace detail {

/// @brief 3-point quadratic Lagrange interpolation weights over the cells
/// (k-1, k, k+1) for a sample at offset md in [-0.5, 0.5] from the center
/// of cell k:
///   w = (0.5*md*(md-1), 1 - md*md, 0.5*md*(md+1))
///   w' = (md - 0.5, -2*md, md + 0.5)
/// md=0 reads (0, 1, 0): the kernel INTERPOLATES at cell centers, and the
/// gradient is the piecewise-linear derivative of the same quadratic.
inline void quadraticWeights(double md, double* w, double* dw) {
  w[0] = 0.5 * md * (md - 1.0);
  w[1] = 1.0 - md * md;
  w[2] = 0.5 * md * (md + 1.0);
  dw[0] = md - 0.5;
  dw[1] = -2.0 * md;
  dw[2] = md + 0.5;
}

}  // namespace detail

/// @brief Sample the signed distance field at a world point with the
/// package interpolation contract (quadratic kernel + analytic gradient).
///
/// Out-of-field points return kOutOfField with +max_distance and a zero
/// gradient (outside the rolling window is unobserved space: far, and with
/// no meaningful descent direction).
inline EsdfQueryResult sampleDistance(const EsdfSnapshot& snapshot,
                                      double wx, double wy) {
  EsdfQueryResult result;
  result.distance = snapshot.max_distance;
  double mx = 0.0;
  double my = 0.0;
  if (!snapshot.worldToMap(wx, wy, mx, my)) {
    result.status = EsdfQueryStatus::kOutOfField;
    return result;
  }
  // Kernel anchoring: k = nearest cell center; md = signed offset from the
  // center in [-0.5, 0.5]. The 3-point kernel spans cells k-1 .. k+1.
  int kx = static_cast<int>(std::lround(mx));
  int ky = static_cast<int>(std::lround(my));
  const double mdx = mx - static_cast<double>(kx);
  const double mdy = my - static_cast<double>(ky);
  // Clamp the kernel origin so k-1 and k+1 stay in range at the borders.
  kx = std::clamp(kx, 1, snapshot.size_x - 2);
  ky = std::clamp(ky, 1, snapshot.size_y - 2);
  double wx_weights[3];
  double wx_derivatives[3];
  double wy_weights[3];
  double wy_derivatives[3];
  detail::quadraticWeights(mdx, wx_weights, wx_derivatives);
  detail::quadraticWeights(mdy, wy_weights, wy_derivatives);

  double value = 0.0;
  double grad_x = 0.0;
  double grad_y = 0.0;
  for (int j = 0; j < 3; ++j) {
    double row_value = 0.0;
    double row_derivative = 0.0;
    for (int i = 0; i < 3; ++i) {
      const float v = snapshot.distanceAt(kx - 1 + i, ky - 1 + j);
      row_value += wx_weights[i] * static_cast<double>(v);
      row_derivative += wx_derivatives[i] * static_cast<double>(v);
    }
    value += wy_weights[j] * row_value;
    grad_x += wy_weights[j] * row_derivative;
    grad_y += wy_derivatives[j] * row_value;
  }
  result.distance = value;
  result.gradient_x = grad_x / snapshot.resolution;
  result.gradient_y = grad_y / snapshot.resolution;
  result.status = EsdfQueryStatus::kOk;
  return result;
}

}  // namespace esdfmap
