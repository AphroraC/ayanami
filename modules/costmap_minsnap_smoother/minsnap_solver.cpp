#include "minsnap_solver.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#ifdef AYANAMI_MINSNAP_HAS_OSQP
#include <OsqpEigen/OsqpEigen.h>
#endif

namespace ayanami {
namespace modules {
namespace minsnap {

namespace {

constexpr double k_infinity = std::numeric_limits<double>::infinity();

// Constraint row count; layout shared by generateA/generateBounds:
// [0] start velocity, [1] start position, [2..N+1] point positions
// (point j at row 1+j, N = segments + 1 points), [N+2] end velocity,
// then (segments-1)*continuity continuity rows, then segments-1 waypoint
// velocity rows.
int constraintRowCount(const SolveProblem& problem) {
  const int segments = static_cast<int>(problem.times.size());
  const int continuity =
      std::min(problem.derivative_order, problem.order - 1);
  return 1 + 1 + segments + 1 + (segments - 1) * continuity +
         (segments - 1);
}

// Q sub-block for one segment: integral of the squared derivative_order-th
// derivative of the ascending-power polynomial over [0, T], plus a small
// distance penalty on the linear term (tdt generateQ).
MatXd generateSegmentQ(double t, int order, int derivative_order,
                       bool normalized_time) {
  MatXd q = MatXd::Zero(order, order);
  if (normalized_time) {
    const double scale = std::pow(t, 1 - 2 * derivative_order);
    for (int i = derivative_order; i < order; ++i) {
      for (int l = i; l < order; ++l) {
        q(i, l) = scale *
                  static_cast<double>(fallingFactorial(i, derivative_order) *
                                      fallingFactorial(l, derivative_order)) /
                  static_cast<double>(i + l - 2 * derivative_order + 1);
      }
    }
    q(1, 1) += 0.01 / (t * t);
  } else {
    for (int i = derivative_order; i < order; ++i) {
      for (int l = i; l < order; ++l) {
        q(i, l) = static_cast<double>(fallingFactorial(i, derivative_order) *
                                      fallingFactorial(l, derivative_order)) *
                  std::pow(t, i + l - 2 * derivative_order + 1) /
                  static_cast<double>(i + l - 2 * derivative_order + 1);
      }
    }
    q(1, 1) += 0.01;
  }
  return q;
}

MatXd generateQ(const SolveProblem& problem) {
  const int order = problem.order;
  const int segments = static_cast<int>(problem.times.size());
  MatXd q = MatXd::Zero(order * segments, order * segments);
  for (int k = 0; k < segments; ++k) {
    q.block(k * order, k * order, order, order) =
        generateSegmentQ(problem.times[k], order, problem.derivative_order,
                         problem.normalized_time);
  }
  q /= std::max(q.cwiseAbs().maxCoeff(), 1e-12);
  return q;
}

// Row giving d^derivative p / dt^derivative at physical time t of a segment
// of length segment_t (ascending-power coefficients; normalized time
// rescales by T^-derivative, tdt getfx/getdnx).
VecXd derivativeRow(double t, int derivative, double segment_t,
                    const SolveProblem& problem) {
  const int order = problem.order;
  VecXd row = VecXd::Zero(order);
  const double u = problem.normalized_time ? t / segment_t : t;
  const double scale =
      problem.normalized_time ? std::pow(segment_t, -derivative) : 1.0;
  for (int b = derivative; b < order; ++b) {
    row(b) = static_cast<double>(fallingFactorial(b, derivative)) *
             (b == derivative ? 1.0 : std::pow(u, b - derivative)) * scale;
  }
  return row;
}

MatXd generateA(const SolveProblem& problem) {
  const int order = problem.order;
  const int segments = static_cast<int>(problem.times.size());
  const int continuity =
      std::min(problem.derivative_order, order - 1);
  MatXd a = MatXd::Zero(constraintRowCount(problem), order * segments);

  int line = 0;
  a.block(line++, 0, 1, problem.order) =
      derivativeRow(0.0, 1, problem.times[0], problem).transpose();
  a.block(line++, 0, 1, problem.order) =
      derivativeRow(0.0, 0, problem.times[0], problem).transpose();
  // Point positions at their segment end (t = T_i keeps the matrix away
  // from u = 0 degeneracy on later segments; tdt generateA). Each row
  // segment occupies columns [i*order, (i+1)*order) of the constraint row.
  for (int i = 0; i < segments; ++i) {
    a.block(line, i * problem.order, 1, problem.order) =
        derivativeRow(problem.times[i], 0, problem.times[i],
                      problem).transpose();
    ++line;
  }
  a.block(line, (segments - 1) * order, 1, order) =
      derivativeRow(problem.times.back(), 1, problem.times.back(), problem)
          .transpose();
  ++line;
  // Continuity across interior points (end of segment i vs start of
  // i+1), rows normalized to unit norm (tdt).
  for (int i = 0; i + 1 < segments; ++i) {
    for (int d = 0; d < continuity; ++d) {
      a.block(line, i * order, 1, order) =
          derivativeRow(problem.times[i], d, problem.times[i], problem)
              .transpose();
      a.block(line, (i + 1) * order, 1, order) =
          -derivativeRow(0.0, d, problem.times[i + 1], problem).transpose();
      const double norm = a.row(line).norm();
      if (norm > 0.0) {
        a.row(line) /= norm;
      }
      ++line;
    }
  }
  // Waypoint velocity rows: start-of-segment first derivative coefficient
  // (rescaled under normalized time); continuity pins both sides equal.
  for (int i = 1; i < segments; ++i) {
    a(line, i * order + 1) = problem.normalized_time ? 1.0 / problem.times[i]
                                                     : 1.0;
    ++line;
  }
  return a;
}

// Bounds for one axis; row layout mirrors generateA. Velocity-unconstrained
// rows get infinite bounds; position rows are the corridor box (has_velocity
// points are pinned exactly, their corridor ignored; tdt makeBounds).
std::pair<VecXd, VecXd> generateBounds(const SolveProblem& problem, int axis) {
  const int segments = static_cast<int>(problem.times.size());
  const int rows = constraintRowCount(problem);
  VecXd low = VecXd::Zero(rows);
  VecXd up = VecXd::Zero(rows);

  low(0) = problem.points.front().has_velocity
               ? problem.points.front().velocity(axis)
               : -k_infinity;
  up(0) = problem.points.front().has_velocity
              ? problem.points.front().velocity(axis)
              : k_infinity;
  for (int i = 0; i <= segments; ++i) {
    const auto& p = problem.points[i];
    const int row = 1 + i;
    if (p.has_velocity) {
      low(row) = up(row) = p.xy(axis);
    } else {
      const double lo = axis == 0 ? p.corridor[0] : p.corridor[1];
      const double hi = axis == 0 ? p.corridor[2] : p.corridor[3];
      low(row) = std::min(lo, hi);
      up(row) = std::max(lo, hi);
    }
  }
  const int end_row = segments + 2;
  low(end_row) = problem.points.back().has_velocity
                     ? problem.points.back().velocity(axis)
                     : -k_infinity;
  up(end_row) = problem.points.back().has_velocity
                    ? problem.points.back().velocity(axis)
                    : k_infinity;
  const int continuity =
      std::min(problem.derivative_order, problem.order - 1);
  int line = segments + 3 + (segments - 1) * continuity;
  for (int i = 1; i < segments; ++i) {
    if (problem.points[i].has_velocity) {
      low(line) = up(line) = problem.points[i].velocity(axis);
    } else {
      low(line) = -k_infinity;
      up(line) = k_infinity;
    }
    ++line;
  }
  return {low, up};
}

}  // namespace

int fallingFactorial(int from, int n) {
  int value = 1;
  for (int i = 0; i < n; ++i) {
    value *= from - i;
  }
  return value;
}

std::vector<double> trapezoidalTimeAllocation(
    const std::vector<Eigen::Vector2d>& points, double max_speed,
    double max_acc) {
  if (points.size() < 2 || !std::isfinite(max_speed) ||
      !std::isfinite(max_acc) || max_speed <= 0.0 || max_acc <= 0.0) {
    return {};
  }
  for (const auto& p : points) {
    if (!p.allFinite()) {
      return {};
    }
  }
  std::vector<double> times(points.size() - 1);
  const double accel_dist = 0.5 * max_speed * max_speed / max_acc;
  for (std::size_t i = 0; i + 1 < points.size(); ++i) {
    const double dist = (points[i + 1] - points[i]).norm();
    times[i] = dist < accel_dist
                   ? 2.0 * std::sqrt(dist / max_acc)
                   : (dist - accel_dist) / max_speed +
                         2.0 * std::sqrt(accel_dist / max_acc);
  }
  return times;
}

bool validTimes(const std::vector<double>& times, size_t point_count,
                int order, int derivative_order) {
  if (point_count < 2 || times.size() + 1 != point_count || order < 3 ||
      derivative_order < 2) {
    return false;
  }
  return std::all_of(times.begin(), times.end(), [](double t) {
    return std::isfinite(t) && t > 0.0;
  });
}

bool solveClosedForm(const SolveProblem& problem,
                     SegmentCoefficients& coefficients_out) {
  if (!validTimes(problem.times, problem.points.size(), problem.order,
                  problem.derivative_order)) {
    return false;
  }
  // The closed-form backend is equality-only: unpinned points must carry
  // degenerate corridor boxes (tdt Backend::Close contract).
  for (const auto& p : problem.points) {
    if (p.corridor[0] != p.corridor[2] || p.corridor[1] != p.corridor[3]) {
      return false;
    }
  }

  const MatXd a = generateA(problem);
  const int rows = static_cast<int>(a.rows());
  MatXd bounds(rows, 2);
  std::vector<int> equality_rows;
  for (int axis = 0; axis < 2; ++axis) {
    const auto [low, up] = generateBounds(problem, axis);
    bounds.col(axis) = low;
    if (axis == 0) {
      for (int r = 0; r < rows; ++r) {
        if (std::isfinite(low(r)) && low(r) == up(r)) {
          equality_rows.push_back(r);
        }
      }
    }
  }

  const int eq = static_cast<int>(equality_rows.size());
  MatXd equality(eq, a.cols());
  MatXd target(eq, 2);
  for (int i = 0; i < eq; ++i) {
    const int r = equality_rows[i];
    const double scale = a.row(r).norm();
    if (!std::isfinite(scale) || scale == 0.0) {
      return false;
    }
    equality.row(i) = a.row(r) / scale;
    target.row(i) = bounds.row(r) / scale;
  }

  // p = p0 + N z: p0 satisfies the equalities, N spans their nullspace;
  // the free variables minimize the quadratic form (tdt closeSolver).
  Eigen::FullPivLU<MatXd> decomposition(equality);
  MatXd solution = decomposition.solve(target);
  if ((equality * solution - target).cwiseAbs().maxCoeff() > 1e-7) {
    return false;
  }
  if (decomposition.dimensionOfKernel() > 0) {
    MatXd kernel = decomposition.kernel();
    Eigen::HouseholderQR<MatXd> qr(kernel);
    kernel = qr.householderQ() * MatXd::Identity(kernel.rows(), kernel.cols());

    const MatXd q = generateQ(problem);
    const MatXd reduced_q = kernel.transpose() * q * kernel;
    const MatXd rhs = -kernel.transpose() * q * solution;
    const MatXd free_values =
        reduced_q.completeOrthogonalDecomposition().solve(rhs);
    solution += kernel * free_values;
  }
  if (!solution.allFinite() ||
      (equality * solution - target).cwiseAbs().maxCoeff() > 1e-6) {
    return false;
  }
  coefficients_out = solution;
  return true;
}

bool solveOsqpCorridor(const SolveProblem& problem,
                       const std::vector<Eigen::Vector4d>& corridor,
                       SegmentCoefficients& coefficients_out) {
#ifndef AYANAMI_MINSNAP_HAS_OSQP
  (void)problem;
  (void)corridor;
  (void)coefficients_out;
  return false;
#else
  const int segments = static_cast<int>(problem.times.size());
  if (!validTimes(problem.times, problem.points.size(), problem.order,
                  problem.derivative_order) ||
      static_cast<int>(corridor.size()) != segments + 1) {
    return false;
  }
  SolveProblem boxed = problem;
  for (int i = 0; i <= segments; ++i) {
    for (int c = 0; c < 4; ++c) {
      if (!std::isfinite(corridor[i][c])) {
        return false;
      }
    }
    if (corridor[i][0] > corridor[i][2] || corridor[i][1] > corridor[i][3]) {
      return false;
    }
    boxed.points[i].corridor = corridor[i];
  }

  MatXd dense_a = generateA(boxed);
  const int rows = static_cast<int>(dense_a.rows());
  MatXd bounds_low(rows, 2);
  MatXd bounds_up(rows, 2);
  for (int axis = 0; axis < 2; ++axis) {
    const auto [low, up] = generateBounds(boxed, axis);
    bounds_low.col(axis) = low;
    bounds_up.col(axis) = up;
  }
  // Row normalization keeps high-order terms comparable (tdt osqpExecute):
  // constraints and their bounds are scaled together.
  for (int r = 0; r < rows; ++r) {
    const double scale = dense_a.row(r).norm();
    if (!std::isfinite(scale) || scale == 0.0) {
      return false;
    }
    dense_a.row(r) /= scale;
    bounds_low.row(r) /= scale;
    bounds_up.row(r) /= scale;
  }

  Eigen::SparseMatrix<double> q = generateQ(boxed).sparseView();
  Eigen::SparseMatrix<double> a = dense_a.sparseView();
  VecXd gradient = VecXd::Zero(a.cols());
  MatXd solution(a.cols(), 2);

  // x and y share Q and A; only the bounds differ (tdt).
  for (int axis = 0; axis < 2; ++axis) {
    OsqpEigen::Solver solver;
    solver.settings()->setVerbosity(false);
    solver.settings()->setPolish(true);
    solver.settings()->setAbsoluteTolerance(1e-8);
    solver.settings()->setRelativeTolerance(1e-8);
    solver.settings()->setMaxIteration(20000);
    solver.settings()->setTimeLimit(0.1);
    // OsqpEigen 0.8 API: the size setters return void, and the bound
    // setters take materialized vectors (no expressions).
    solver.data()->setNumberOfVariables(static_cast<int>(a.cols()));
    solver.data()->setNumberOfConstraints(static_cast<int>(a.rows()));
    VecXd lower = bounds_low.col(axis);
    VecXd upper = bounds_up.col(axis);
    if (!solver.data()->setHessianMatrix(q) ||
        !solver.data()->setGradient(gradient) ||
        !solver.data()->setLinearConstraintsMatrix(a) ||
        !solver.data()->setLowerBound(lower) ||
        !solver.data()->setUpperBound(upper) ||
        !solver.initSolver()) {
      return false;
    }
    if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError ||
        (solver.getStatus() != OsqpEigen::Status::Solved &&
         solver.getStatus() != OsqpEigen::Status::SolvedInaccurate)) {
      return false;
    }
    solution.col(axis) = solver.getSolution();
    const VecXd values = a * solution.col(axis);
    if (!solution.col(axis).allFinite() ||
        (bounds_low.col(axis) - values).maxCoeff() > 1e-5 ||
        (values - bounds_up.col(axis)).maxCoeff() > 1e-5) {
      return false;
    }
  }
  coefficients_out = solution;
  return true;
#endif
}

std::vector<Eigen::Vector2d> sampleSegment(
    const SegmentCoefficients& coefficients, int order,
    const std::vector<double>& times, int index, bool normalized_time,
    int samples) {
  std::vector<Eigen::Vector2d> out;
  out.reserve(samples + 1);
  for (int s = 0; s <= samples; ++s) {
    const double t = static_cast<double>(s) / samples;
    const double u = normalized_time ? t : t * times[index];
    Eigen::Vector2d point = Eigen::Vector2d::Zero();
    for (int i = 0; i < order; ++i) {
      point += Eigen::Vector2d(coefficients(index * order + i, 0),
                               coefficients(index * order + i, 1)) *
               std::pow(u, i);
    }
    out.push_back(point);
  }
  return out;
}

std::vector<int> insertMidpoints(std::vector<double>& times,
                                 std::vector<ControlPoint>& points,
                                 const std::vector<int>& bad_segments) {
  if (times.size() + 1 != points.size()) {
    return {};
  }
  std::vector<int> inserted;
  for (auto it = bad_segments.rbegin(); it != bad_segments.rend(); ++it) {
    const int i = *it;
    if (i < 0 || static_cast<size_t>(i) >= times.size() ||
        static_cast<size_t>(i) + 1 >= points.size()) {
      continue;
    }
    ControlPoint mid;
    mid.xy = (points[i].xy + points[i + 1].xy) * 0.5;
    mid.corridor = {mid.xy.x(), mid.xy.y(), mid.xy.x(), mid.xy.y()};
    points.insert(points.begin() + i + 1, mid);
    for (auto& index : inserted) {
      if (index >= i + 1) {
        ++index;
      }
    }
    inserted.push_back(i + 1);
    const double half = times[i] * 0.5;
    times[i] = half;
    times.insert(times.begin() + i + 1, half);
  }
  std::sort(inserted.begin(), inserted.end());
  return inserted;
}

}  // namespace minsnap
}  // namespace modules
}  // namespace ayanami
