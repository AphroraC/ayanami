// rosemap_escape_recovery / rosemap_wait_recovery tests: direct-constructed
// modules over synthetic snapshots + the test TF source. Escape semantics
// locked: immediate exit in open field, uphill direction near a wall,
// plateau fallback, tick budget, cancel. Wait: duration/cancel semantics.

#include <gtest/gtest.h>

#include <rosemap_escape_recovery/rosemap_escape_recovery.hpp>
#include <rosemap_wait_recovery/rosemap_wait_recovery.hpp>

#include <ayanami/rosemap/core/edt.hpp>
#include <ayanami/rosemap/core/rosemap_map_context.hpp>
#include <ayanami/utility/config.hpp>
#include "test_transform_source.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
#include <numbers>

namespace {

using ayanami::ErrorCode;
using ayanami::RecoveryCycleData;
using ayanami::RecoveryStatus;
using ayanami::rosemap::RosemapMapContext;
using RosemapSnapshot = ayanami::rosemap::RosemapSnapshot;

constexpr int k_size = 100;   // cells
constexpr double k_res = 0.1;
constexpr double k_origin = -5.0;

/// @brief 10x10 m empty field (optionally with a wall cell block at the
/// world origin).
struct FieldFixture {
  RosemapSnapshot snapshot;

  explicit FieldFixture(bool with_wall = false) {
    snapshot.frame = "map";
    snapshot.resolution = k_res;
    snapshot.size_x = k_size;
    snapshot.size_y = k_size;
    snapshot.origin_x = k_origin;
    snapshot.origin_y = k_origin;
    snapshot.max_distance = 3.0;
    snapshot.occupied.assign(static_cast<std::size_t>(k_size) * k_size, 0);
    if (with_wall) {
      // A 3x3 block at cells 48..50 (world ~ (-0.2, -0.2)..(0.2, 0.2)).
      for (int y = 48; y <= 50; ++y) {
        for (int x = 48; x <= 50; ++x) {
          snapshot.occupied[static_cast<std::size_t>(y) * k_size + x] = 1;
        }
      }
    }
    const bool edt_ok = ayanami::rosemap::signedEdt2d(
        snapshot.occupied, k_size, k_size, k_res, 3.0, snapshot.distance);
    EXPECT_TRUE(edt_ok);
  }
};

class RosemapRecoveryTest : public ::testing::Test {
 protected:
  void SetUp() override {
    ayanami::testing::testTransforms();
    ayanami::testing::testTransforms().set(
        "map", "base_link", Eigen::Isometry3d::Identity());
    context = std::make_shared<RosemapMapContext>();
    context->setFrames("map", "base_link");
  }

  const RosemapSnapshot& publish(const FieldFixture& fixture) {
    current = std::make_shared<RosemapSnapshot>(fixture.snapshot);
    context->publishSnapshot(current);
    return *current;
  }

  /// @brief Place the robot at a world position with a given yaw.
  void placeRobot(double x, double y, double yaw) {
    Eigen::Isometry3d map_from_base = Eigen::Isometry3d::Identity();
    map_from_base.translation().x() = x;
    map_from_base.translation().y() = y;
    map_from_base.linear() =
        Eigen::Isometry3d(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()))
            .linear();
    ayanami::testing::testTransforms().set("map", "base_link", map_from_base);
  }

  static ayanami::Config loadParamJson(const std::string& json) {
    const auto dir = std::filesystem::temp_directory_path() /
                     "rosemap_recovery_test_cfg";
    std::filesystem::create_directories(dir);
    const auto path = dir / "module.json";
    std::ofstream ofs(path);
    ofs << json;
    ofs.close();
    return ayanami::Config(path.string());
  }

  RosemapMapContext::Ptr context;
  std::shared_ptr<const RosemapSnapshot> current;
};

// ---- escape ----

TEST_F(RosemapRecoveryTest, EscapeSucceedsImmediatelyInOpenField) {
  publish(FieldFixture(false));
  placeRobot(-3.0, -3.0, 0.0);  // far from everything

  ayanami::modules::RosemapEscapeRecovery recovery;
  const ayanami::Config empty_config;
  recovery.configure(
      ayanami::ModuleContext{"rosemap_escape_recovery", empty_config},
      context);
  recovery.setCyclePeriod(0.01);
  recovery.start();

  RecoveryCycleData cycle;
  auto result = recovery.tick(cycle);
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_EQ(cycle.status, RecoveryStatus::SUCCESS);
}

TEST_F(RosemapRecoveryTest, EscapeRunsUphillAwayFromWall) {
  publish(FieldFixture(true));
  // Robot 0.5 m east of the block: clearance ~0.4... place at 0.35 east
  // (clearance well below the 0.4 exit) facing +x.
  placeRobot(0.35, 0.0, 0.0);

  const ayanami::Config params = loadParamJson(R"({
    "rosemap_escape_recovery": {
      "exit_distance": 0.4,
      "sweep_radius": 0.2,
      "resample_interval": 1000,
      "max_ticks": 100
    }
  })");
  ayanami::modules::RosemapEscapeRecovery recovery;
  recovery.configure(ayanami::ModuleContext{"rosemap_escape_recovery", params},
                     context);
  recovery.setCyclePeriod(0.01);
  recovery.start();

  RecoveryCycleData cycle;
  auto result = recovery.tick(cycle);
  ASSERT_TRUE(result.ok) << result.message;
  ASSERT_TRUE(cycle.velocity.has_value());
  // Uphill = +x (away from the wall); robot yaw 0 -> body x is world x.
  EXPECT_GT(cycle.velocity->twist.linear.x(), 0.3);
  EXPECT_NEAR(cycle.velocity->twist.linear.y(), 0.0, 1e-6);
  EXPECT_EQ(cycle.status, RecoveryStatus::RUNNING);

  // After enough running ticks the robot is still near the wall (nothing
  // moves it in the test), so the tick budget eventually fails.
  for (int i = 0; i < 200; ++i) {
    result = recovery.tick(cycle);
    if (!result.ok) {
      break;
    }
  }
  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.error, ErrorCode::RECOVERY_FAILED);
}

TEST_F(RosemapRecoveryTest, EscapeDirectionRotatesIntoBodyFrame) {
  publish(FieldFixture(true));
  // Robot east of the wall facing -y (yaw = -pi/2): the world +x escape
  // reads as BODY -y.
  placeRobot(0.35, 0.0, -1.57079632679489661923);

  ayanami::modules::RosemapEscapeRecovery recovery;
  recovery.configure(ayanami::ModuleContext{
                         "rosemap_escape_recovery", ayanami::Config{}},
                     context);
  recovery.setCyclePeriod(0.01);
  recovery.start();

  RecoveryCycleData cycle;
  auto result = recovery.tick(cycle);
  ASSERT_TRUE(result.ok) << result.message;
  ASSERT_TRUE(cycle.velocity.has_value());
  EXPECT_GT(std::abs(cycle.velocity->twist.linear.y()), 0.3);
  EXPECT_LT(std::abs(cycle.velocity->twist.linear.x()), 0.05);
}

TEST_F(RosemapRecoveryTest, EscapeCancelTakesEffect) {
  publish(FieldFixture(true));
  placeRobot(0.35, 0.0, 0.0);

  ayanami::modules::RosemapEscapeRecovery recovery;
  recovery.configure(ayanami::ModuleContext{
                         "rosemap_escape_recovery", ayanami::Config{}},
                     context);
  recovery.start();
  recovery.cancel();

  RecoveryCycleData cycle;
  auto result = recovery.tick(cycle);
  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.error, ErrorCode::CANCELED);
}

// ---- wait ----

TEST_F(RosemapRecoveryTest, WaitRunsForConfiguredDuration) {
  const ayanami::Config params = loadParamJson(R"({
    "rosemap_wait_recovery": {"wait_duration": 0.05}
  })");
  ayanami::modules::RosemapWaitRecovery recovery;
  recovery.configure(ayanami::ModuleContext{"rosemap_wait_recovery", params},
                     nullptr);
  recovery.setCyclePeriod(0.01);
  recovery.start();

  int running_ticks = 0;
  RecoveryCycleData cycle;
  while (true) {
    auto result = recovery.tick(cycle);
    ASSERT_TRUE(result.ok);
    if (cycle.status == RecoveryStatus::SUCCESS) {
      break;
    }
    EXPECT_FALSE(cycle.velocity.has_value());
    ++running_ticks;
    ASSERT_LT(running_ticks, 100);
  }
  EXPECT_NEAR(running_ticks, 5, 1);
}

TEST_F(RosemapRecoveryTest, WaitCancelTakesEffect) {
  ayanami::modules::RosemapWaitRecovery recovery;
  recovery.configure(ayanami::ModuleContext{"rosemap_wait_recovery",
                                            ayanami::Config{}},
                     nullptr);
  recovery.setCyclePeriod(0.01);
  recovery.start();
  recovery.cancel();

  RecoveryCycleData cycle;
  auto result = recovery.tick(cycle);
  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.error, ErrorCode::CANCELED);
}

}  // namespace
