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

#include <memory>
#include <vector>

#include "neupan/initial_path.hpp"

using namespace neupan;
using PathPoint = InitialPath::PathPoint;

namespace {

constexpr int kT = 10;
constexpr double kDt = 0.1;

std::shared_ptr<const Robot> robot() {
  return std::make_shared<const Robot>(
      Robot::diffRectangle(kT, kDt, Vec2(2, 1), Vec2(2, 2), 0.5, 0.5));
}

InitialPath::Options spacing(double interval) {
  InitialPath::Options o;
  o.interval = interval;
  return o;
}

// Walks the robot along +x in `step` increments, returning the first x at
// which checkArrive reports arrival (or `limit` if it never does).
double driveUntilArrived(InitialPath& ip, double from, double step,
                         double limit) {
  for (double x = from; x < limit; x += step)
    if (ip.checkArrive(Vec3(x, 0, 0))) return x;
  return limit;
}

}  // namespace

TEST(InitialPath, LineCurveIsDensifiedToTheInterval) {
  InitialPath ip(robot(), 1.0, spacing(0.25));
  ip.setIpathWithWaypoints({Vec3(0, 0, 0), Vec3(1, 0, 0), Vec3(1, 0, 0)});

  const auto& path = ip.initialPath();
  ASSERT_EQ(path.size(), 5u);  // 0, .25, .5, .75, 1; duplicate skipped
  for (size_t i = 0; i < path.size(); ++i) {
    EXPECT_NEAR(path[i].x(), 0.25 * i, 1e-12);
    EXPECT_NEAR(path[i](2), 0.0, 1e-12);  // heading along the segment
    EXPECT_EQ(path[i](3), 1.0);           // forward gear
  }
}

TEST(InitialPath, ExternalPathIsResampledToTheSameSpacing) {
  InitialPath ip(robot(), 1.0, spacing(0.4));
  // Sparse path as A* + Douglas-Peucker would publish it (2 m spacing).
  ip.setInitialPath(
      {PathPoint(0, 0, 0, 1), PathPoint(2, 0, 0, 1), PathPoint(4, 0, 0, 1)});
  const auto& path = ip.initialPath();
  EXPECT_EQ(path.size(), 11u);
  EXPECT_NEAR(path.back().x(), 4.0, 1e-12);
  EXPECT_THROW(ip.setInitialPath({PathPoint(0, 0, 0, 1)}),
               std::invalid_argument);
}

TEST(InitialPath, ArrivalLatchesUntilANewPathIsSet) {
  InitialPath ip(robot(), 1.0, spacing(0.1));
  ip.updateInitialPathFromGoal(Vec3(0, 0, 0), Vec3(2, 0, 0));

  EXPECT_FALSE(ip.checkArrive(Vec3(0, 0, 0)));
  const double arrived_at = driveUntilArrived(ip, 0.0, 0.05, 3.0);
  EXPECT_NEAR(arrived_at, 2.0, 0.1);
  EXPECT_TRUE(ip.arrived());

  // Settling past the end must not flicker the arrival off.
  EXPECT_TRUE(ip.checkArrive(Vec3(2.3, 0, 0)));

  const int revision = ip.revision();
  ip.updateInitialPathFromGoal(Vec3(2.3, 0, 0), Vec3(5, 0, 0));
  EXPECT_GT(ip.revision(), revision);
  EXPECT_FALSE(ip.arrived());
  EXPECT_FALSE(ip.checkArrive(Vec3(2.3, 0, 0)));
}

TEST(InitialPath, ReplaceKeepsProgressWhileSetRestarts) {
  InitialPath ip(robot(), 1.0, spacing(0.1));
  ip.updateInitialPathFromGoal(Vec3(0, 0, 0), Vec3(4, 0, 0));
  driveUntilArrived(ip, 0.0, 0.05, 1.5);  // progress to x ~ 1.5

  const Vec3 state(1.5, 0, 0);
  const std::vector<PathPoint> replan{PathPoint(0, 0, 0, 1),
                                      PathPoint(4, 0, 0, 1)};
  ip.replaceInitialPath(replan, state);
  const NomRef kept = ip.generateNomRefState(state, Mat2X::Zero(2, kT), 1.0);
  EXPECT_NEAR(kept.ref_s(0, 0), 1.5, 0.1);

  ip.setInitialPath(replan);
  const NomRef restarted =
      ip.generateNomRefState(state, Mat2X::Zero(2, kT), 1.0);
  EXPECT_NEAR(restarted.ref_s(0, 0), 0.0, 1e-12);
}

TEST(InitialPath, ReferenceAdvancesAtRefSpeedAndStopsAtTheEnd) {
  InitialPath ip(robot(), 1.0, spacing(0.1));
  ip.updateInitialPathFromGoal(Vec3(0, 0, 0), Vec3(0.5, 0, 0));
  ASSERT_FALSE(ip.checkArrive(Vec3(0, 0, 0)));

  // ref_speed * dt == interval: one path point per step.
  const NomRef nr =
      ip.generateNomRefState(Vec3(0, 0, 0), Mat2X::Zero(2, kT), 1.0);
  ASSERT_EQ(nr.ref_s.cols(), kT + 1);
  ASSERT_EQ(nr.nom_s.cols(), kT + 1);
  for (int t = 0; t <= 5; ++t) EXPECT_NEAR(nr.ref_s(0, t), 0.1 * t, 1e-9);
  for (int t = 6; t <= kT; ++t) EXPECT_NEAR(nr.ref_s(0, t), 0.5, 1e-9);

  // Reference speed drops to zero once the reference runs off the end.
  EXPECT_DOUBLE_EQ(nr.ref_us(0), 1.0);
  EXPECT_DOUBLE_EQ(nr.ref_us(kT - 1), 0.0);

  // Zero nominal velocity keeps the nominal states at the start.
  EXPECT_TRUE(nr.nom_s.isApprox(Mat3X::Zero(3, kT + 1)));
}
