/*
 * neupan_cpp: C++ port of the NeuPAN planner.
 *
 * Ported from NeuPAN (https://github.com/hanruihua/NeuPAN),
 * Copyright (c) 2025 Ruihua Han <hanrh@connect.hku.hk>.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version. See <https://www.gnu.org/licenses/>.
 */

#include "neupan/nrmp.hpp"

#include <OsqpEigen/OsqpEigen.h>

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace neupan {

namespace {
// OSQP_INFTY rather than a real infinity: an inf bound makes OSQP v1.0's
// duality-gap termination check produce NaN, so it never reports Solved.
constexpr double kInf = OSQP_INFTY;
}  // namespace

NRMP::NRMP(std::shared_ptr<const Robot> robot, const NRMPParams& params)
    : robot_(std::move(robot)), p_(params), T_(robot_->T), M_(params.max_num) {
  off_u_ = 3 * (T_ + 1);
  off_d_ = off_u_ + 2 * T_;
  off_e_ = off_d_ + T_;
  nz_ = off_e_ + M_ * T_;

  // rows: init eq (3) | dynamics (3T) | speed (2T) | acce (2(T-1)) |
  //       d box (T) | avoidance (M*T) | e >= 0 (M*T)
  nc_ = 3 + 3 * T_ + 2 * T_ + 2 * (T_ - 1) + T_ + 2 * M_ * T_;

  A_.resize(nc_, nz_);
  q_.resize(nz_);
  lb_.resize(nc_);
  ub_.resize(nc_);
  buildHessian();
}

NRMP::~NRMP() = default;
NRMP::NRMP(NRMP&&) noexcept = default;
NRMP& NRMP::operator=(NRMP&&) noexcept = default;

void NRMP::updateParams(const NRMPParams& params) {
  if (params.max_num != M_)
    throw std::invalid_argument(
        "nrmp: max_num sets the problem size and cannot change after "
        "construction");
  p_ = params;
  buildHessian();
  initialized_ = false;
}

NRMP::Coefficients NRMP::buildFaFb(const std::vector<Mat>& mu_list,
                                   const std::vector<Mat2X>& lam_list,
                                   const std::vector<Mat2X>& point_list) const {
  Coefficients co;
  co.fa.assign(T_, Mat::Zero(M_, 2));
  co.fb.assign(T_, Vec::Zero(M_));

  if (mu_list.empty()) return co;

  for (int t = 0; t < T_; ++t) {
    const Mat& mu = mu_list[t + 1];
    const Mat2X& lam = lam_list[t + 1];
    const Mat2X& pts = point_list[t + 1];
    Mat& fa = co.fa[t];
    Vec& fb = co.fb[t];

    const int pn = std::min<int>(static_cast<int>(mu.cols()), M_);
    for (int k = 0; k < pn; ++k) {
      fa.row(k) = lam.col(k).transpose();
      fb(k) = lam.col(k).dot(pts.col(k)) + mu.col(k).dot(robot_->h);
    }
    // Upstream pads missing rows with the closest point's constraint.
    for (int k = pn; k < M_; ++k) {
      fa.row(k) = fa.row(0);
      fb(k) = fb(0);
    }
  }
  return co;
}

void NRMP::checkShapes(const NomRef& nom, const Coefficients& co) const {
  const bool ok = nom.nom_s.cols() == T_ + 1 && nom.nom_u.cols() == T_ &&
                  nom.ref_s.cols() == T_ + 1 && nom.ref_us.size() == T_ &&
                  static_cast<int>(co.fa.size()) == T_ &&
                  static_cast<int>(co.fb.size()) == T_ &&
                  std::all_of(co.fa.begin(), co.fa.end(),
                              [&](const Mat& fa) {
                                return fa.rows() == M_ && fa.cols() == 2;
                              }) &&
                  std::all_of(co.fb.begin(), co.fb.end(),
                              [&](const Vec& fb) { return fb.size() == M_; });
  if (!ok)
    throw std::invalid_argument(
        "nrmp: problem data does not match horizon T / max_num (build the "
        "coefficients with buildFaFb)");
}

// cvxpy objective J(z) = z' Q z + c' z; OSQP form 0.5 z' P z + q' z with
// P = 2 Q. Terms: ||q_s .* s - q_s .* ref_s||^2, ||p_u u_v - p_u ref_us||^2,
// 0.5 bk ||s - nom_s||^2, -eta sum(d), 0.5 ro_obs ||e||^2.
// Only q depends on the frame; P depends on the weights alone.
void NRMP::buildHessian() {
  const Vec3& qs = p_.q_s;
  std::vector<Eigen::Triplet<double>> trip;
  trip.reserve(nz_);

  for (int t = 0; t <= T_; ++t)
    for (int r = 0; r < 3; ++r)
      trip.emplace_back(sIdx(t, r), sIdx(t, r), 2.0 * qs(r) * qs(r) + p_.bk);
  for (int t = 0; t < T_; ++t) {
    trip.emplace_back(uIdx(t, 0), uIdx(t, 0), 2.0 * p_.p_u * p_.p_u);
    trip.emplace_back(uIdx(t, 1), uIdx(t, 1), 0.0);
  }
  for (int t = 0; t < T_; ++t) trip.emplace_back(dIdx(t), dIdx(t), 0.0);
  for (int j = off_e_; j < nz_; ++j) trip.emplace_back(j, j, p_.ro_obs);

  P_.resize(nz_, nz_);
  P_.setFromTriplets(trip.begin(), trip.end());
}

void NRMP::assemble(const NomRef& nom, const Coefficients& co) {
  const Vec3& qs = p_.q_s;

  // ---- gradient q ----
  q_.setZero();
  for (int t = 0; t <= T_; ++t)
    for (int r = 0; r < 3; ++r) {
      const double gamma_a = qs(r) * nom.ref_s(r, t);
      q_(sIdx(t, r)) = -2.0 * qs(r) * gamma_a - p_.bk * nom.nom_s(r, t);
    }
  for (int t = 0; t < T_; ++t)
    q_(uIdx(t, 0)) = -2.0 * p_.p_u * (p_.p_u * nom.ref_us(t));
  for (int t = 0; t < T_; ++t) q_(dIdx(t)) = -p_.eta;

  // ---- constraints: lb <= A z <= ub ----
  a_trip_.clear();
  a_trip_.reserve(20 * T_ + 6 * M_ * T_);
  int row = 0;
  const auto add = [&](int col, double value) {
    a_trip_.emplace_back(row, col, value);
  };
  const auto bound = [&](double lo, double hi) {
    lb_(row) = lo;
    ub_(row) = hi;
    ++row;
  };

  // init: s_0 == nom_s[:, 0]
  for (int r = 0; r < 3; ++r) {
    add(sIdx(0, r), 1.0);
    bound(nom.nom_s(r, 0), nom.nom_s(r, 0));
  }

  // dynamics: A_t s_t + B_t u_t - s_{t+1} == -C_t. Every entry that can be
  // nonzero for the diff model is emitted, even when it is currently zero,
  // so the sparsity pattern never changes.
  for (int t = 0; t < T_; ++t) {
    Mat33 A;
    Mat32 B;
    Vec3 C;
    robot_->linearize(nom.nom_s.col(t), nom.nom_u.col(t), A, B, C);

    for (int r = 0; r < 3; ++r) {
      add(sIdx(t, r), A(r, r));
      if (r < 2) {
        add(sIdx(t, 2), A(r, 2));
        add(uIdx(t, 0), B(r, 0));
        // Omni's B(r,1) (velocity-heading column) is nonzero on these rows;
        // the diff model writes an explicit zero so BOTH kinematics share
        // one fixed sparsity pattern.
        add(uIdx(t, 1), B(r, 1));
      } else {
        add(uIdx(t, 1), B(r, 1));
      }
      add(sIdx(t + 1, r), -1.0);
      bound(-C(r), -C(r));
    }
  }

  // speed box: |u| <= max_speed
  for (int t = 0; t < T_; ++t)
    for (int r = 0; r < 2; ++r) {
      add(uIdx(t, r), 1.0);
      bound(-robot_->max_speed(r), robot_->max_speed(r));
    }

  // acceleration box: |u_{t+1} - u_t| <= max_acce * dt
  for (int t = 0; t + 1 < T_; ++t)
    for (int r = 0; r < 2; ++r) {
      add(uIdx(t + 1, r), 1.0);
      add(uIdx(t, r), -1.0);
      bound(-robot_->acce_bound(r), robot_->acce_bound(r));
    }

  // distance box: d_min <= d <= d_max
  for (int t = 0; t < T_; ++t) {
    add(dIdx(t), 1.0);
    bound(p_.d_min, p_.d_max);
  }

  // avoidance: fa . s_xy(t+1) - d_t + e_{t,k} >= fb
  for (int t = 0; t < T_; ++t)
    for (int k = 0; k < M_; ++k) {
      add(sIdx(t + 1, 0), co.fa[t](k, 0));
      add(sIdx(t + 1, 1), co.fa[t](k, 1));
      add(dIdx(t), -1.0);
      add(eIdx(t, k), 1.0);
      bound(co.fb[t](k), kInf);
    }

  // e >= 0
  for (int t = 0; t < T_; ++t)
    for (int k = 0; k < M_; ++k) {
      add(eIdx(t, k), 1.0);
      bound(0.0, kInf);
    }

  if (row != nc_) throw std::logic_error("nrmp: constraint row mismatch");

  if (a_pos_.empty()) {
    A_.setFromTriplets(a_trip_.begin(), a_trip_.end());
    a_pos_.resize(a_trip_.size());
    const auto* outer = A_.outerIndexPtr();
    const auto* inner = A_.innerIndexPtr();
    for (size_t k = 0; k < a_trip_.size(); ++k) {
      const auto* first = inner + outer[a_trip_[k].col()];
      const auto* last = inner + outer[a_trip_[k].col() + 1];
      a_pos_[k] = std::lower_bound(first, last, a_trip_[k].row()) - inner;
    }
    if (A_.nonZeros() != static_cast<Eigen::Index>(a_trip_.size()))
      throw std::logic_error("nrmp: duplicate constraint entries");
  } else {
    // OSQP's value-only update is only valid for the pattern it was set up
    // with; a different entry count would misalign every value after it.
    if (a_trip_.size() != a_pos_.size())
      throw std::logic_error("nrmp: constraint sparsity pattern changed");
    double* values = A_.valuePtr();
    for (size_t k = 0; k < a_trip_.size(); ++k)
      values[a_pos_[k]] = a_trip_[k].value();
  }
}

void NRMP::initSolver() {
  solver_ = std::make_unique<OsqpEigen::Solver>();
  auto& settings = *solver_->settings();
  settings.setVerbosity(p_.solver.verbose);
  settings.setWarmStart(true);
  settings.setPolish(p_.solver.polish);
  settings.setAbsoluteTolerance(p_.solver.eps_abs);
  settings.setRelativeTolerance(p_.solver.eps_rel);
  settings.setMaxIteration(p_.solver.max_iter);
  // OSQP rejects 0; leaving it unset keeps OSQP's "no limit" default.
  if (p_.solver.time_limit > 0.0) settings.setTimeLimit(p_.solver.time_limit);

  auto& data = *solver_->data();
  data.setNumberOfVariables(nz_);
  data.setNumberOfConstraints(nc_);
  if (!data.setHessianMatrix(P_) || !data.setGradient(q_) ||
      !data.setLinearConstraintsMatrix(A_) || !data.setLowerBound(lb_) ||
      !data.setUpperBound(ub_) || !solver_->initSolver())
    throw std::runtime_error("nrmp: OSQP init failed");
  initialized_ = true;
}

NRMP::Result NRMP::solve(const NomRef& nom, const Coefficients& coeffs) {
  checkShapes(nom, coeffs);
  assemble(nom, coeffs);

  if (!initialized_) {
    initSolver();
  } else if (!solver_->updateLinearConstraintsMatrix(A_) ||
             !solver_->updateGradient(q_) || !solver_->updateBounds(lb_, ub_)) {
    // (OsqpEigen 0.8 port: the upstream osqp-1.x C call reduces to these
    // wrappers -- P is constant and A's sparsity pattern never changes.)
    throw std::runtime_error("nrmp: OSQP update failed");
  }

  Result res;
  const bool no_error =
      solver_->solveProblem() == OsqpEigen::ErrorExitFlag::NoError;
  const OsqpEigen::Status status = solver_->getStatus();
  res.status = static_cast<int>(status);
  res.success = no_error && status == OsqpEigen::Status::Solved;

  const Vec z = solver_->getSolution();
  res.s = Eigen::Map<const Mat>(z.data(), 3, T_ + 1);
  res.u = Eigen::Map<const Mat>(z.data() + off_u_, 2, T_);
  res.d = z.segment(off_d_, T_);
  return res;
}

double NRMP::objective(const Mat3X& s, const Mat2X& u, const Vec& d,
                       const NomRef& nom, const Coefficients& co) const {
  double J = 0.0;
  for (int t = 0; t <= T_; ++t)
    for (int r = 0; r < 3; ++r) {
      const double diff = p_.q_s(r) * s(r, t) - p_.q_s(r) * nom.ref_s(r, t);
      J += diff * diff;
    }
  for (int t = 0; t < T_; ++t) {
    const double diff = p_.p_u * u(0, t) - p_.p_u * nom.ref_us(t);
    J += diff * diff;
  }
  J += 0.5 * p_.bk * (s - nom.nom_s).squaredNorm();
  J -= p_.eta * d.sum();
  for (int t = 0; t < T_; ++t)
    for (int k = 0; k < M_; ++k) {
      const double I = co.fa[t](k, 0) * s(0, t + 1) +
                       co.fa[t](k, 1) * s(1, t + 1) - co.fb[t](k) - d(t);
      const double neg = std::max(-I, 0.0);
      J += 0.5 * p_.ro_obs * neg * neg;
    }
  return J;
}

}  // namespace neupan
