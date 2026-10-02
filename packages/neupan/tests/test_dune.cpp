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
#include <cstdio>
#include <memory>

#include "neupan/dune.hpp"
#include "neupan/mlp.hpp"
#include "neupan/robot.hpp"
#include "neupan/tensor_io.hpp"
#include "reference_data.hpp"

using namespace neupan;

namespace {

const std::string kData = NEUPAN_TEST_DATA_DIR;
const std::string kModel = std::string(NEUPAN_MODEL_DIR) + "/diff_default.bin";

std::shared_ptr<const Robot> lonRobot() {
  // example/LON/planner.yaml: diff, max_speed [8,1], max_acce [8,3],
  // length 1.6, width 2.0, T=10, dt=0.1
  return std::make_shared<const Robot>(
      Robot::diffRectangle(10, 0.1, Vec2(8, 1), Vec2(8, 3), 1.6, 2.0));
}

}  // namespace

TEST(Dune, MlpMatchesPyTorch) {
  const MLP mlp = MLP::load(kModel);
  EXPECT_EQ(mlp.inputDim(), 2);
  EXPECT_EQ(mlp.outputDim(), 4);

  const TensorFile tf = TensorFile::load(kData + "/dune_mlp.nptf");
  const MatF points = tf.at("points").cast<float>();
  const Mat mu_py = tf.at("mu");

  const MatF mu_cpp = mlp.forward(points);
  ASSERT_EQ(mu_cpp.rows(), mu_py.rows());
  ASSERT_EQ(mu_cpp.cols(), mu_py.cols());

  const double max_diff = (mu_cpp.cast<double>() - mu_py).cwiseAbs().maxCoeff();
  std::printf("MLP max|mu_cpp - mu_py| = %.3e over %ld points\n", max_diff,
              static_cast<long>(mu_py.cols()));
  EXPECT_LT(max_diff, 1e-4);
}

TEST(Dune, RobotGhMatchesUpstream) {
  const TensorFile tf = TensorFile::load(kData + "/frames.nptf");
  const auto robot = lonRobot();
  EXPECT_LT((robot->G - tf.at("meta.G")).cwiseAbs().maxCoeff(), 1e-12);
  EXPECT_LT((robot->h - tf.at("meta.h")).cwiseAbs().maxCoeff(), 1e-12);
  EXPECT_LT((robot->max_speed - tf.at("meta.max_speed")).cwiseAbs().maxCoeff(),
            1e-12);
  EXPECT_LT(
      (robot->acce_bound - tf.at("meta.acce_bound")).cwiseAbs().maxCoeff(),
      1e-12);
}

TEST(Dune, ForwardMatchesPython) {
  const TensorFile tf = TensorFile::load(kData + "/frames.nptf");
  const DUNE dune(lonRobot(), kModel);

  const int T = ref::scalarInt(tf, "meta.T");
  const int n_frames = ref::scalarInt(tf, "meta.n_frames");

  double max_mu_diff = 0.0, max_lam_diff = 0.0, max_pt_diff = 0.0;
  const auto maxAbsDiff = [](const Mat& a, const Mat& b) {
    return (a - b).cwiseAbs().maxCoeff();
  };

  for (int f = 0; f < n_frames; ++f) {
    const int n_iters = ref::scalarInt(tf, ref::frameKey(f) + ".n_iters");

    for (int it = 0; it < n_iters; ++it) {
      const std::string pre = ref::iterKey(f, it);

      const DUNE::Output out =
          dune.forward(ref::stages<Mat2X>(tf, pre, "pf", T),
                       ref::stages<Mat22>(tf, pre, "R", T),
                       ref::stages<Mat2X>(tf, pre, "op", T));

      EXPECT_NEAR(out.min_distance, tf.at(pre + ".min_dist")(0, 0), 1e-4);
      for (int t = 0; t <= T; ++t) {
        const std::string st = ref::stageKey(t);
        max_mu_diff = std::max(
            max_mu_diff, maxAbsDiff(out.mu_list[t], tf.at(pre + ".mu" + st)));
        max_lam_diff =
            std::max(max_lam_diff,
                     maxAbsDiff(out.lam_list[t], tf.at(pre + ".lam" + st)));
        max_pt_diff = std::max(
            max_pt_diff,
            maxAbsDiff(out.sort_point_list[t], tf.at(pre + ".sp" + st)));
      }
    }
  }

  std::printf(
      "DUNE forward: max mu diff %.3e, lam diff %.3e, point diff %.3e\n",
      max_mu_diff, max_lam_diff, max_pt_diff);
  EXPECT_LT(max_mu_diff, 1e-4);
  EXPECT_LT(max_lam_diff, 1e-3);
  // identical sorting => identical point selection
  EXPECT_LT(max_pt_diff, 1e-9);
}
