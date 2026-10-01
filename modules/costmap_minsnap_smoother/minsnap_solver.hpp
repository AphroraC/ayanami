#pragma once

#include <string>
#include <vector>

#include <Eigen/Dense>

namespace ayanami {
namespace modules {
namespace minsnap {

// Minimum-snap trajectory core (tdt-nav-kit MinimumSnap semantics), pure
// math with no map dependency: per-segment ascending-power polynomials
// p(u) = sum c[i] u^i with u = t (physical) or t/T (normalized), minimum of
// the squared derivative cost integral under position / waypoint-velocity /
// continuity equality constraints (closed form) or corridor box inequality
// constraints (OSQP).
//
// Numerical-stability measures ported from tdt-nav-kit verbatim:
// - optional per-segment normalized time (normT) with physical-derivative
//   rescaling (d^a p/dt^a = T^-a d^a p/dtau^a);
// - constraint rows normalized to unit norm;
// - Q normalized by its largest absolute entry;
// - solution accepted only after an equality/constraint residual check.

using MatXd = Eigen::MatrixXd;
using VecXd = Eigen::VectorXd;

/// @brief One trajectory control point.
struct ControlPoint {
  Eigen::Vector2d xy{Eigen::Vector2d::Zero()};
  /// @brief Constrain the trajectory velocity while passing through this
  /// point (also pins the position exactly).
  bool has_velocity{false};
  Eigen::Vector2d velocity{Eigen::Vector2d::Zero()};
  /// @brief Box constraint [xmin, ymin, xmax, ymax] (OSQP backend); for the
  /// closed-form backend every point is pinned at xy instead.
  Eigen::Vector4d corridor{0.0, 0.0, 0.0, 0.0};
};

/// @brief Fully specified smoothing problem:
/// points.size() == times.size() + 1, every time > 0.
struct SolveProblem {
  std::vector<double> times;
  std::vector<ControlPoint> points;
  int order{6};               ///< polynomial coefficient count per segment
  int derivative_order{3};    ///< 3 = minimum jerk
  bool normalized_time{true};
};

/// @brief Per-segment polynomial coefficients, ascending power, columns
/// [x, y]; rows = segments * order.
using SegmentCoefficients = Eigen::Matrix<double, Eigen::Dynamic, 2>;

/// @brief Falling factorial from*(from-1)*...*(from-n+1).
int fallingFactorial(int from, int n);

/// @brief Trapezoidal per-segment time allocation honoring max speed/acc
/// (tdt trapezoidalTimeAllocation); empty on invalid input.
std::vector<double> trapezoidalTimeAllocation(
    const std::vector<Eigen::Vector2d>& points, double max_speed,
    double max_acc);

/// @brief True when times is usable for the given problem shape.
bool validTimes(const std::vector<double>& times, size_t point_count,
                int order, int derivative_order);

/// @brief Closed-form equality-constrained solve (zero external
/// dependencies). Solves min 0.5 x^T Q x subject to position / velocity /
/// continuity equalities via a nullspace factorization; false on singular
/// or residual-check failure.
bool solveClosedForm(const SolveProblem& problem,
                     SegmentCoefficients& coefficients_out);

/// @brief OSQP corridor solve (compiled only when AYANAMI_MINSNAP_HAS_OSQP
/// is defined; always returns false otherwise). `corridor[i]` boxes each
/// control point; velocity-equality items are honored.
bool solveOsqpCorridor(const SolveProblem& problem,
                       const std::vector<Eigen::Vector4d>& corridor,
                       SegmentCoefficients& coefficients_out);

/// @brief Samples one segment at `samples`+1 points (tdt lineDecoder: 10
/// subdivisions), normalized time handled internally.
std::vector<Eigen::Vector2d> sampleSegment(
    const SegmentCoefficients& coefficients, int order,
    const std::vector<double>& times, int index, bool normalized_time,
    int samples = 10);

/// @brief Inserts the midpoint of every colliding segment (and splits its
/// time); returns the inserted control-point indices. Shared repair step of
/// both backends.
std::vector<int> insertMidpoints(std::vector<double>& times,
                                 std::vector<ControlPoint>& points,
                                 const std::vector<int>& bad_segments);

}  // namespace minsnap
}  // namespace modules
}  // namespace ayanami
