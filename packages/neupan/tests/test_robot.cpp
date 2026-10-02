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

#include <cmath>
#include <stdexcept>

#include "neupan/robot.hpp"

using namespace neupan;

namespace {

Mat2X polygon(std::initializer_list<Vec2> pts) {
  Mat2X m(2, pts.size());
  Eigen::Index i = 0;
  for (const Vec2& p : pts) m.col(i++) = p;
  return m;
}

}  // namespace

TEST(Robot, InequalitiesContainExactlyTheFootprint) {
  Mat G;
  Vec h;
  genInequalFromVertex(
      polygon({Vec2(-1, -0.5), Vec2(1, -0.5), Vec2(1, 0.5), Vec2(-1, 0.5)}), G,
      h);
  ASSERT_EQ(G.rows(), 4);
  EXPECT_TRUE(((G * Vec2(0, 0)) - h).maxCoeff() < 0);            // inside
  EXPECT_TRUE(((G * Vec2(0.9, 0.4)) - h).maxCoeff() < 0);        // inside
  EXPECT_TRUE(((G * Vec2(1.1, 0.0)) - h).maxCoeff() > 0);        // outside
  EXPECT_NEAR(((G * Vec2(1.0, 0.0)) - h).maxCoeff(), 0, 1e-12);  // edge
}

TEST(Robot, ClockwiseInputGivesTheSameConstraintsAsCounterClockwise) {
  Mat G_ccw, G_cw;
  Vec h_ccw, h_cw;
  genInequalFromVertex(
      polygon({Vec2(0, 0), Vec2(2, 0), Vec2(2, 1), Vec2(0, 1)}), G_ccw, h_ccw);
  genInequalFromVertex(
      polygon({Vec2(0, 0), Vec2(0, 1), Vec2(2, 1), Vec2(2, 0)}), G_cw, h_cw);
  EXPECT_TRUE(G_ccw.isApprox(G_cw));
  EXPECT_TRUE(h_ccw.isApprox(h_cw));
}

TEST(Robot, RejectsDegenerateAndNonConvexFootprints) {
  Mat G;
  Vec h;
  EXPECT_THROW(genInequalFromVertex(polygon({Vec2(0, 0), Vec2(1, 0)}), G, h),
               std::invalid_argument);
  // Arrow head: the vertex at (1, 0.5) points inwards.
  EXPECT_THROW(
      genInequalFromVertex(polygon({Vec2(0, 0), Vec2(2, 0), Vec2(1, 0.5),
                                    Vec2(2, 1), Vec2(0, 1)}),
                           G, h),
      std::invalid_argument);
}

TEST(Robot, DiffRectangleMatchesTheConfiguredSize) {
  const Robot r =
      Robot::diffRectangle(10, 0.1, Vec2(2, 1), Vec2(4, 3), 1.6, 2.0);
  EXPECT_NEAR(r.vertices.row(0).maxCoeff() - r.vertices.row(0).minCoeff(), 1.6,
              1e-12);
  EXPECT_NEAR(r.vertices.row(1).maxCoeff() - r.vertices.row(1).minCoeff(), 2.0,
              1e-12);
  EXPECT_TRUE(r.acce_bound.isApprox(Vec2(0.4, 0.3)));  // max_acce * dt
}

TEST(Robot, LinearizationIsExactAtTheNominalPoint) {
  const Robot r =
      Robot::diffRectangle(10, 0.1, Vec2(2, 1), Vec2(4, 3), 0.5, 0.5);
  const Vec3 s(1.0, -2.0, 0.7);
  const Vec2 u(1.5, -0.4);

  Mat33 A;
  Mat32 B;
  Vec3 C;
  r.linearize(s, u, A, B, C);
  EXPECT_TRUE((A * s + B * u + C).isApprox(r.propagate(s, u), 1e-12));

  const Vec3 next = r.propagate(s, u);
  EXPECT_NEAR(next(0), 1.0 + 0.1 * 1.5 * std::cos(0.7), 1e-12);
  EXPECT_NEAR(next(1), -2.0 + 0.1 * 1.5 * std::sin(0.7), 1e-12);
  EXPECT_NEAR(next(2), 0.7 - 0.04, 1e-12);
}
