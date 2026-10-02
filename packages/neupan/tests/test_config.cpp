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
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include "neupan/initial_path.hpp"
#include "neupan/math_utils.hpp"
#include "neupan/neupan_planner.hpp"

using namespace neupan;

namespace {

const std::string kModel = std::string(NEUPAN_MODEL_DIR) + "/diff_default.bin";

std::string writeYaml(const std::string& name, const std::string& body) {
  const auto path = std::filesystem::temp_directory_path() / name;
  std::ofstream(path) << body;
  return path.string();
}

// Footprint of diff_default.bin, so the checkpoint check passes.
const char* kRobotYaml = "robot: {kinematics: diff, length: 1.6, width: 2.0}\n";

using Mutation = std::function<void(NeuPANPlanner::Config&)>;

bool rejected(const Mutation& mutate) {
  NeuPANPlanner::Config c;
  mutate(c);
  try {
    NeuPANPlanner::validate(c);
  } catch (const std::runtime_error&) {
    return true;
  }
  return false;
}

}  // namespace

TEST(Config, DefaultsAreValid) {
  EXPECT_NO_THROW(NeuPANPlanner::validate(NeuPANPlanner::Config{}));
}

TEST(Config, RejectsSelfContradictoryMargins) {
  EXPECT_TRUE(rejected([](auto& c) { c.adjust.d_min = -0.1; }));
  EXPECT_TRUE(rejected([](auto& c) { c.adjust.d_max = 0.05; }));
  EXPECT_TRUE(rejected([](auto& c) {
    c.adjust.d_min = 0.0;
    c.adjust.d_max = 0.0;
  }));
  EXPECT_TRUE(rejected([](auto& c) { c.collision_threshold = 0.0; }));
}

TEST(Config, RejectsValuesThatHangOrDisableAvoidance) {
  EXPECT_TRUE(rejected([](auto& c) { c.ref_speed = 0.0; }));
  EXPECT_TRUE(rejected([](auto& c) { c.ref_speed = -1.0; }));
  EXPECT_TRUE(rejected(
      [](auto& c) { c.ref_speed = std::numeric_limits<double>::quiet_NaN(); }));
  EXPECT_TRUE(rejected([](auto& c) { c.max_speed(1) = 0.0; }));
  EXPECT_TRUE(rejected([](auto& c) { c.max_acce(0) = -1.0; }));
  EXPECT_TRUE(rejected([](auto& c) { c.width = 0.0; }));
  EXPECT_TRUE(rejected([](auto& c) { c.pan.iter_num = 0; }));
  EXPECT_TRUE(rejected([](auto& c) { c.pan.dune_max_num = 0; }));
  EXPECT_TRUE(rejected([](auto& c) { c.adjust.max_num = 0; }));
  EXPECT_TRUE(rejected([](auto& c) { c.ipath.ind_range = 0; }));
}

TEST(Config, InitialPathRejectsNonPositiveInterval) {
  const auto robot = std::make_shared<const Robot>(
      Robot::diffRectangle(10, 0.1, Vec2(1, 1), Vec2(1, 1), 0.5, 0.5));
  InitialPath::Options opts;
  EXPECT_THROW(InitialPath(robot, 0.0, opts), std::invalid_argument);
  opts.interval = 0.2;  // an explicit spacing makes ref_speed irrelevant here
  EXPECT_NO_THROW(InitialPath(robot, 0.0, opts));
}

TEST(MathUtils, WrapToPi) {
  EXPECT_DOUBLE_EQ(wrapToPi(0.5), 0.5);
  EXPECT_NEAR(wrapToPi(kPi + 0.5), -kPi + 0.5, 1e-12);
  EXPECT_NEAR(wrapToPi(-kPi - 0.5), kPi - 0.5, 1e-12);
  EXPECT_NEAR(wrapToPi(1e6), std::remainder(1e6, 2 * kPi), 1e-9);
  EXPECT_LE(std::abs(wrapToPi(123.456)), kPi);
  // The old subtract loop never returned for inf.
  EXPECT_TRUE(std::isnan(wrapToPi(std::numeric_limits<double>::infinity())));
}

TEST(Config, FromYamlReadsTheOptionalOsqpBlock) {
  const auto path = writeYaml(
      "neupan_osqp.yaml",
      std::string(kRobotYaml) +
          "osqp: {max_iter: 500, time_limit: 0.015, polish: false}\n");
  const NeuPANPlanner planner = NeuPANPlanner::fromYaml(path, kModel);
  const NRMPParams::Solver& s = planner.config().adjust.solver;
  EXPECT_EQ(s.max_iter, 500);
  EXPECT_DOUBLE_EQ(s.time_limit, 0.015);
  EXPECT_FALSE(s.polish);
  EXPECT_DOUBLE_EQ(s.eps_abs, NRMPParams::Solver{}.eps_abs);  // untouched
}

TEST(Config, FromYamlRejectsUnportedFeatures) {
  EXPECT_THROW(
      NeuPANPlanner::fromYaml(
          writeYaml("neupan_acker.yaml", "robot: {kinematics: acker}\n"),
          kModel),
      std::runtime_error);
  EXPECT_THROW(
      NeuPANPlanner::fromYaml(writeYaml("neupan_nockpt.yaml", kRobotYaml), ""),
      std::runtime_error);
}
