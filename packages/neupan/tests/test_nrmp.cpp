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

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "neupan/nrmp.hpp"
#include "neupan/tensor_io.hpp"
#include "reference_data.hpp"

using namespace neupan;

namespace {
const std::string kData = NEUPAN_TEST_DATA_DIR;
}

// Replays every dumped NRMP call (each frame, each PAM iteration) against
// the cvxpy/ECOS reference: action inf-norm diff < 5%, objective relative
// diff < 1%.
TEST(Nrmp, SolveMatchesCvxpy) {
  const TensorFile tf = TensorFile::load(kData + "/frames.nptf");

  const int T = ref::scalarInt(tf, "meta.T");
  const int n_frames = ref::scalarInt(tf, "meta.n_frames");

  NRMP nrmp(ref::robot(tf), ref::nrmpParams(tf));

  double max_action_diff = 0.0, max_obj_rel = 0.0, max_u0_scale = 0.0;
  int n_solves = 0;

  for (int f = 0; f < n_frames; ++f) {
    const int n_iters = ref::scalarInt(tf, ref::frameKey(f) + ".n_iters");

    for (int it = 0; it < n_iters; ++it) {
      const std::string pre = ref::iterKey(f, it);
      const NomRef nom = ref::nomRef(tf, pre);

      // Rebuild fa/fb from the dumped (sorted) DUNE outputs, exercising
      // buildFaFb the same way the upstream NRMP consumes mu/lam/points.
      const NRMP::Coefficients coeffs =
          nrmp.buildFaFb(ref::stages<Mat>(tf, pre, "mu", T),
                         ref::stages<Mat2X>(tf, pre, "lam", T),
                         ref::stages<Mat2X>(tf, pre, "sp", T));

      const NRMP::Result res = nrmp.solve(nom, coeffs);
      ASSERT_TRUE(res.success) << pre;
      ++n_solves;

      const Mat3X s_py = tf.at(pre + ".opt_s");
      const Mat2X u_py = tf.at(pre + ".opt_u");
      const Vec d_py = tf.at(pre + ".opt_d").row(0).transpose();

      // action = first control column
      const double action_diff =
          (res.u.col(0) - u_py.col(0)).cwiseAbs().maxCoeff();
      max_action_diff = std::max(max_action_diff, action_diff);
      max_u0_scale = std::max(max_u0_scale, u_py.col(0).cwiseAbs().maxCoeff());

      const double obj_cpp = nrmp.objective(res.s, res.u, res.d, nom, coeffs);
      const double obj_py = nrmp.objective(s_py, u_py, d_py, nom, coeffs);
      const double obj_rel =
          std::abs(obj_cpp - obj_py) / std::max(1.0, std::abs(obj_py));
      max_obj_rel = std::max(max_obj_rel, obj_rel);

      EXPECT_LT(action_diff, 0.05 * std::max(1.0, max_u0_scale)) << pre;
      EXPECT_LT(obj_rel, 0.01) << pre;
      // C++ optimum must not be worse than the reference solution.
      EXPECT_LT(obj_cpp, obj_py + 0.01 * std::max(1.0, std::abs(obj_py)))
          << pre;
    }
  }

  std::printf(
      "NRMP: %d solves, max action diff %.4f (scale %.3f), "
      "max objective rel diff %.3e\n",
      n_solves, max_action_diff, max_u0_scale, max_obj_rel);
}
