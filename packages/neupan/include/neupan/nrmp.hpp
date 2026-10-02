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

#pragma once

#include <Eigen/Sparse>
#include <memory>
#include <vector>

#include "neupan/robot.hpp"
#include "neupan/types.hpp"

namespace OsqpEigen {
class Solver;
}

namespace neupan {

// Adjustable weights, mirroring the NRMP / adjust kwargs in nrmp.py.
struct NRMPParams {
  int max_num = 10;         // nrmp_max_num
  Vec3 q_s = Vec3::Ones();  // scalar q_s == all three entries equal
  double p_u = 1.0;
  double eta = 10.0;
  double d_max = 1.0;
  double d_min = 0.1;
  double ro_obs = 400.0;
  double bk = 0.1;

  // OSQP settings. The tolerances are tight for parity with the cvxpy
  // reference; time_limit (seconds, 0 = none) caps the worst-case solve so a
  // hard frame fails into the planner's fallback instead of overrunning the
  // control period.
  struct Solver {
    double eps_abs = 1e-5;
    double eps_rel = 1e-5;
    int max_iter = 10000;
    bool polish = true;
    double time_limit = 0.0;
    bool verbose = false;
  } solver;
};

// Port of neupan/blocks/nrmp.py with the cvxpy problem restated as a QP for
// OSQP. The neg()-penalty cost 0.5*ro_obs*||neg(I)||^2 is reformulated with
// slack variables e >= max(0, -I), which is exact at the optimum.
//
// Decision vector z = [ s (3*(T+1)) | u (2*T) | d (T) | e (max_num*T) ].
// The Hessian is constant and the constraint matrix keeps a fixed sparsity
// pattern, so after the first solve only values are pushed to OSQP.
class NRMP {
 public:
  NRMP(std::shared_ptr<const Robot> robot, const NRMPParams& params);
  ~NRMP();

  // The user-declared destructor suppresses the implicit move members; bring
  // them back so NRMP (and PAN/NeuPANPlanner above it) stay movable. Defined
  // in the .cpp where OsqpEigen::Solver is a complete type.
  NRMP(NRMP&&) noexcept;
  NRMP& operator=(NRMP&&) noexcept;

  // Avoidance constraints fa_t . s_xy(t+1) - d_t >= fb_t for stages 1..T,
  // one row per kept obstacle point.
  struct Coefficients {
    std::vector<Mat> fa;  // T of (max_num, 2)
    std::vector<Vec> fb;  // T of (max_num,)
  };

  // Port of generate_coefficient_parameter_value: build the coefficients from
  // the DUNE outputs (mu/lam/points are distance-sorted, the first max_num
  // points are kept, short stages padded with point 0). Empty lists mean no
  // points and give all-zero coefficients, as upstream.
  Coefficients buildFaFb(const std::vector<Mat>& mu_list,
                         const std::vector<Mat2X>& lam_list,
                         const std::vector<Mat2X>& point_list) const;

  struct Result {
    Mat3X s;  // (3, T+1)
    Mat2X u;  // (2, T)
    Vec d;    // (T,)
    bool success = false;
    int status = -1;  // raw OsqpEigen::Status value, for diagnostics
  };

  Result solve(const NomRef& nom, const Coefficients& coeffs);

  // Changing params alters the Hessian; the solver is rebuilt on next solve.
  // max_num fixes the problem size and cannot change after construction.
  void updateParams(const NRMPParams& params);
  const NRMPParams& params() const { return p_; }

  // Objective in the original cvxpy form, evaluated at a given solution.
  double objective(const Mat3X& s, const Mat2X& u, const Vec& d,
                   const NomRef& nom, const Coefficients& coeffs) const;

 private:
  // Column of each variable in z (layout above).
  int sIdx(int t, int r) const { return 3 * t + r; }
  int uIdx(int t, int r) const { return off_u_ + 2 * t + r; }
  int dIdx(int t) const { return off_d_ + t; }
  int eIdx(int t, int k) const { return off_e_ + M_ * t + k; }

  void checkShapes(const NomRef& nom, const Coefficients& coeffs) const;
  void buildHessian();
  void assemble(const NomRef& nom, const Coefficients& coeffs);
  void initSolver();

  std::shared_ptr<const Robot> robot_;
  NRMPParams p_;
  int T_, M_;                  // horizon, points per stage
  int off_u_, off_d_, off_e_;  // block offsets in z
  int nz_, nc_;                // number of variables / constraints

  Eigen::SparseMatrix<double> P_, A_;
  Vec q_, lb_, ub_;

  // A's entries are emitted in the same order every frame; a_pos_[k] is where
  // the k-th one lives in A_'s value array, filled on the first assemble.
  std::vector<Eigen::Triplet<double>> a_trip_;
  std::vector<Eigen::Index> a_pos_;

  std::unique_ptr<OsqpEigen::Solver> solver_;
  bool initialized_ = false;
};

}  // namespace neupan
