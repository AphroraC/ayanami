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

#include <stdexcept>

#include "neupan/safety_supervisor.hpp"

using namespace neupan;
using Level = SafetySupervisor::Level;

namespace {

NeuPANPlanner::Info cycle(bool solved, bool stop = false, bool arrive = false) {
  NeuPANPlanner::Info info;
  info.solved = solved;
  info.stop = stop;
  info.arrive = arrive;
  return info;
}

const Vec2 kDrive(0.5, 0.1);

}  // namespace

TEST(SafetySupervisor, ScanStalenessStartsWithTheFirstScan) {
  SafetySupervisor sup({});
  EXPECT_FALSE(sup.scanStale(100.0));  // never had a scan
  sup.recordScan(10.0);
  EXPECT_FALSE(sup.scanStale(10.4));
  EXPECT_TRUE(sup.scanStale(10.6));
}

TEST(SafetySupervisor, StopsAfterTheSolverFailureGrace) {
  SafetySupervisor::Options opts;
  opts.solver_fail_grace = 2;
  SafetySupervisor sup(opts);

  double t = 0.0;
  EXPECT_EQ(sup.filter(cycle(false), kDrive, t += 0.1), kDrive);
  EXPECT_EQ(sup.filter(cycle(false), kDrive, t += 0.1), kDrive);
  EXPECT_FALSE(sup.solverGaveUp());
  EXPECT_EQ(sup.filter(cycle(false), kDrive, t += 0.1), Vec2::Zero());
  EXPECT_TRUE(sup.solverGaveUp());
  EXPECT_EQ(sup.status(cycle(false)).level, Level::Error);

  // One good solve clears it.
  EXPECT_EQ(sup.filter(cycle(true), kDrive, t += 0.1), kDrive);
  EXPECT_EQ(sup.consecutiveUnsolved(), 0);
}

TEST(SafetySupervisor, ArrivalDoesNotCountAsAFailedSolve) {
  SafetySupervisor::Options opts;
  opts.solver_fail_grace = 0;
  SafetySupervisor sup(opts);
  EXPECT_EQ(sup.filter(cycle(false, false, true), kDrive, 0.0), Vec2::Zero());
  EXPECT_EQ(sup.consecutiveUnsolved(), 0);
  EXPECT_EQ(sup.status(cycle(false, false, true)).level, Level::Ok);
}

TEST(SafetySupervisor, StopHoldsTheRobotAndStillCountsTheSolve) {
  SafetySupervisor sup({});
  EXPECT_EQ(sup.filter(cycle(false, true), kDrive, 0.0), Vec2::Zero());
  EXPECT_EQ(sup.consecutiveUnsolved(), 1);
  EXPECT_EQ(sup.status(cycle(true, true)).level, Level::Warn);
}

TEST(SafetySupervisor, FlagsAStallAndClearsItOnProgress) {
  SafetySupervisor::Options opts;
  opts.stall_timeout = 1.0;
  SafetySupervisor sup(opts);
  const Vec2 idle(0.01, 0.0);

  // The first cycle starts the clock; counting from time zero would flag a
  // stall right away (the "start bug").
  sup.filter(cycle(true), idle, 5.0);
  EXPECT_FALSE(sup.stalled());
  sup.filter(cycle(true), idle, 5.9);
  EXPECT_FALSE(sup.stalled());
  sup.filter(cycle(true), idle, 6.1);
  EXPECT_TRUE(sup.stalled());
  EXPECT_STREQ(sup.status(cycle(true)).message, "stalled");

  sup.filter(cycle(true), kDrive, 6.2);
  EXPECT_FALSE(sup.stalled());

  // Holding still at the goal is not a stall.
  sup.filter(cycle(false, false, true), idle, 20.0);
  EXPECT_FALSE(sup.stalled());
}

TEST(SafetySupervisor, RejectsNonsenseOptions) {
  SafetySupervisor::Options opts;
  opts.scan_timeout = 0.0;
  EXPECT_THROW(SafetySupervisor{opts}, std::invalid_argument);
  opts = {};
  opts.solver_fail_grace = -1;
  EXPECT_THROW(SafetySupervisor{opts}, std::invalid_argument);
}
