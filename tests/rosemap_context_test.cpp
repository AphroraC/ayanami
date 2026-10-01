// Rosemap family core tests: exact signed EDT, the family query contract
// (quadratic B-spline + analytic gradient), the snapshot-swap map context,
// and the RosemapNavigationServer observation pipeline (mark / raytrace
// clear / rolling recentre / reset command / family hook typing).
//
// The server test runs in inline mode (no writer thread): submissions are
// processed synchronously on the caller's thread, the same pipeline the
// writer drives in production.

#include <gtest/gtest.h>

#include <atomic>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>

#include <ayanami/abstract/components/navigation_server_base.hpp>
#include <ayanami/rosemap/core/edt.hpp>
#include <ayanami/rosemap/core/rosemap_command.hpp>
#include <ayanami/rosemap/core/rosemap_map_context.hpp>
#include <ayanami/rosemap/core/rosemap_planner.hpp>
#include <ayanami/rosemap/components/rosemap_navigation_server.hpp>
#include <ayanami/utility/config.hpp>
#include "test_transform_source.hpp"

namespace {

using ayanami::ErrorCode;
using ayanami::ResultT;
using ayanami::rosemap::RosemapMapContext;
using ayanami::rosemap::RosemapNavigationServer;
using ayanami::rosemap::RosemapQueryStatus;
using ayanami::rosemap::RosemapSnapshot;

// ---------------- EDT ----------------

TEST(RosemapEdt, SingleObstacleDistanceMatchesExactEuclidean) {
  // 1 = obstacle at the center of a 41x41 grid at 0.1 m resolution.
  const int n = 41;
  std::vector<unsigned char> occupied(static_cast<std::size_t>(n) * n, 0);
  occupied[static_cast<std::size_t>(20) * n + 20] = 1;
  std::vector<float> field;
  ASSERT_TRUE(ayanami::rosemap::signedEdt2d(occupied, n, n, 0.1, 3.0, field));
  const auto at = [&](int x, int y) {
    return field[static_cast<std::size_t>(y) * n + x];
  };
  // Obstacle cell: penetration depth = -1 * d_free = -0.1.
  EXPECT_NEAR(at(20, 20), -0.1f, 1e-3f);
  // 5 cells straight out: +0.5 m; 3 diagonal cells: 0.3*sqrt(2).
  EXPECT_NEAR(at(25, 20), 0.5f, 2e-3f);
  EXPECT_NEAR(at(23, 23), 0.3f * std::sqrt(2.0f), 2e-3f);
  // Clamp plateau at the corner (distance ~ 2.83 m > 3.0? no: clamp 3.0).
  EXPECT_NEAR(at(0, 0), 2.828f, 2e-3f);
}

TEST(RosemapEdt, ObstacleInteriorCarriesNegativeDepth) {
  // A 5x5 obstacle block: the center sits 2 cells (0.2 m) from the nearest
  // free cell.
  const int n = 21;
  std::vector<unsigned char> occupied(static_cast<std::size_t>(n) * n, 0);
  for (int y = 8; y <= 12; ++y) {
    for (int x = 8; x <= 12; ++x) {
      occupied[static_cast<std::size_t>(y) * n + x] = 1;
    }
  }
  std::vector<float> field;
  ASSERT_TRUE(ayanami::rosemap::signedEdt2d(occupied, n, n, 0.1, 3.0, field));
  EXPECT_NEAR(field[static_cast<std::size_t>(10) * n + 10], -0.3f, 1e-3f);
  EXPECT_NEAR(field[static_cast<std::size_t>(8) * n + 8], -0.1f, 1e-3f);
  // Free space right outside the block: +0.1.
  EXPECT_NEAR(field[static_cast<std::size_t>(10) * n + 13], 0.1f, 1e-3f);
}

TEST(RosemapEdt, MaxDistanceClampBoundsTheField) {
  const int n = 11;
  std::vector<unsigned char> occupied(static_cast<std::size_t>(n) * n, 0);
  occupied[0] = 1;
  std::vector<float> field;
  ASSERT_TRUE(ayanami::rosemap::signedEdt2d(occupied, n, n, 0.1, 0.5, field));
  // Far corner: raw ~1.41 m, clamped to 0.5.
  EXPECT_FLOAT_EQ(field[static_cast<std::size_t>(10) * n + 10], 0.5f);
}

// ---------------- Query interpolation ----------------

TEST(RosemapQuery, CellCenterReadsExactValueAndGradientPointsAway) {
  RosemapSnapshot snapshot;
  snapshot.frame = "map";
  snapshot.resolution = 0.1;
  snapshot.size_x = 41;
  snapshot.size_y = 41;
  snapshot.origin_x = -2.0;
  snapshot.origin_y = -2.0;
  snapshot.max_distance = 3.0;
  snapshot.occupied.assign(static_cast<std::size_t>(41) * 41, 0);
  snapshot.occupied[static_cast<std::size_t>(20) * 41 + 20] = 1;
  ASSERT_TRUE(ayanami::rosemap::signedEdt2d(
      snapshot.occupied, 41, 41, 0.1, 3.0, snapshot.distance));

  // Obstacle center in world coordinates: exact grid value, zero gradient.
  const auto center = ayanami::rosemap::sampleDistance(snapshot, 0.05, 0.05);
  ASSERT_EQ(center.status, RosemapQueryStatus::kOk);
  EXPECT_NEAR(center.distance, -0.1, 1e-3);
  EXPECT_NEAR(center.gradient.norm(), 0.0, 1e-6);

  // 0.5 m to the right of the obstacle: distance ~ +0.5, gradient +x.
  const auto east = ayanami::rosemap::sampleDistance(snapshot, 0.55, 0.05);
  ASSERT_EQ(east.status, RosemapQueryStatus::kOk);
  EXPECT_NEAR(east.distance, 0.5, 0.02);
  EXPECT_NEAR(east.gradient.x(), 1.0, 0.05);
  EXPECT_NEAR(east.gradient.y(), 0.0, 1e-6);
}

TEST(RosemapQuery, OutOfFieldReturnsStatusAndMaxDistance) {
  RosemapSnapshot snapshot;
  snapshot.resolution = 0.1;
  snapshot.size_x = 5;
  snapshot.size_y = 5;
  snapshot.max_distance = 2.0;
  snapshot.occupied.assign(25, 0);
  ASSERT_TRUE(ayanami::rosemap::signedEdt2d(snapshot.occupied, 5, 5, 0.1, 2.0,
                                            snapshot.distance));
  const auto out = ayanami::rosemap::sampleDistance(snapshot, 100.0, 0.0);
  EXPECT_EQ(out.status, RosemapQueryStatus::kOutOfField);
  EXPECT_DOUBLE_EQ(out.distance, 2.0);
}

// ---------------- Context ----------------

TEST(RosemapContext, SnapshotSwapQueryAndFreshness) {
  auto context = std::make_shared<RosemapMapContext>();
  context->setFrames("map", "base_link");
  context->setStaleTimeout(0.0);

  // No snapshot yet: not current, queries degrade to out-of-field.
  EXPECT_FALSE(context->current());
  EXPECT_EQ(context->queryRosemap(0.0, 0.0).status,
            RosemapQueryStatus::kOutOfField);
  // Empty path is invalid regardless.
  ayanami::nav_msgs::Path empty_path;
  EXPECT_FALSE(context->isPathValid(empty_path));
  // No snapshot: validity degrades to true (nothing contradicts it).
  ayanami::nav_msgs::Path some_path;
  some_path.header.frame_id = "map";
  some_path.poses.resize(2);
  EXPECT_TRUE(context->isPathValid(some_path));

  auto snapshot = std::make_shared<RosemapSnapshot>();
  snapshot->frame = "map";
  snapshot->resolution = 0.1;
  snapshot->size_x = 41;
  snapshot->size_y = 41;
  snapshot->origin_x = -2.0;
  snapshot->origin_y = -2.0;
  snapshot->max_distance = 3.0;
  snapshot->occupied.assign(static_cast<std::size_t>(41) * 41, 0);
  snapshot->occupied[static_cast<std::size_t>(20) * 41 + 20] = 1;
  ASSERT_TRUE(ayanami::rosemap::signedEdt2d(snapshot->occupied, 41, 41, 0.1,
                                            3.0, snapshot->distance));
  context->publishSnapshot(snapshot);

  EXPECT_TRUE(context->current());
  EXPECT_EQ(context->currentSnapshot()->sequence, 0u);
  EXPECT_NEAR(context->queryRosemap(0.05, 0.05).distance, -0.1, 1e-3);

  // Path through the obstacle -> invalid; clear path -> valid. The probe
  // starts at the point nearest the robot (identity TF: base_link at
  // origin), so place the robot off-path via the robot pose in map frame
  // (identity -> robot at (0,0), which lies between the two probe starts).
  ayanami::nav_msgs::Path blocked;
  blocked.header.frame_id = "map";
  for (int i = 0; i <= 20; ++i) {
    ayanami::geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position = Eigen::Vector3d(-0.5 + 0.05 * i, 0.05, 0.0);
    blocked.poses.push_back(pose);
  }
  EXPECT_FALSE(context->isPathValid(blocked));

  ayanami::nav_msgs::Path clear_path;
  clear_path.header.frame_id = "map";
  for (int i = 0; i <= 5; ++i) {
    ayanami::geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position = Eigen::Vector3d(-0.5 + 0.2 * i, -0.8, 0.0);
    clear_path.poses.push_back(pose);
  }
  EXPECT_TRUE(context->isPathValid(clear_path));
}

// ---------------- Server pipeline (inline writer mode) ----------------

std::filesystem::path makeServerConfigDir() {
  const auto dir = std::filesystem::temp_directory_path() /
                   "ayanami_rosemap_context_test";
  std::filesystem::remove_all(dir);
  std::filesystem::create_directories(dir);
  {
    std::ofstream ofs(dir / "config.json");
    ofs << "{\n  \"global\": {\n"
        << "    \"config_navigation\": \"navigation.json\"\n  }\n}\n";
  }
  {
    std::ofstream ofs(dir / "navigation.json");
    ofs << "{\n"
        << "  \"navigation\": {\n"
        << "    \"robot_frame\": \"base_link\",\n"
        << "    \"velocity_timeout\": 0.25\n"
        << "  },\n"
        << "  \"rosemap_field\": {\n"
        << "    \"field_frame\": \"map\",\n"
        << "    \"resolution\": 0.1,\n"
        << "    \"size_x\": 4.0,\n"
        << "    \"size_y\": 4.0,\n"
        << "    \"update_frequency\": 0.0,\n"
        << "    \"max_distance\": 3.0,\n"
        << "    \"obstacle_min_range\": 0.05,\n"
        << "    \"obstacle_max_range\": 30.0,\n"
        << "    \"raytrace_max_range\": 30.0,\n"
        << "    \"min_obstacle_z\": -1.0,\n"
        << "    \"max_obstacle_z\": 2.0\n"
        << "  }\n}\n";
  }
  return dir;
}

class RosemapServerPipeline : public ::testing::Test {
 protected:
  void SetUp() override {
    ayanami::testing::testTransforms();
    ayanami::testing::testTransforms().set(
        "map", "base_link", Eigen::Isometry3d::Identity());
    Eigen::Isometry3d map_from_laser = Eigen::Isometry3d::Identity();
    map_from_laser.translation().x() = 0.1;
    ayanami::testing::testTransforms().set("map", "laser", map_from_laser);

    server = std::make_shared<RosemapNavigationServer>();
    server->configureCoreMaps(makeServerConfigDir().string());
  }

  /// @brief A single-beam scan aimed at `angle` (cell-center geometry in
  /// the callers keeps the assertions exact).
  ayanami::sensor_msgs::LaserScan makeScan(double range, double angle) {
    ayanami::sensor_msgs::LaserScan scan;
    scan.header.frame_id = "laser";
    scan.angle_min = angle;
    scan.angle_max = angle;
    scan.angle_increment = 0.01;
    scan.range_min = 0.05;
    scan.range_max = 30.0;
    scan.ranges.assign(1, range);
    return scan;
  }

  std::shared_ptr<RosemapNavigationServer> server;
};

TEST_F(RosemapServerPipeline, InitialSnapshotIsEmptyAndCurrent) {
  const auto snapshot = server->fieldContext()->currentSnapshot();
  ASSERT_NE(snapshot, nullptr);
  EXPECT_TRUE(snapshot->valid());
  // 4x4 m at 0.1 -> 40x40 cells.
  EXPECT_EQ(snapshot->size_x, 40);
  EXPECT_EQ(snapshot->size_y, 40);
  EXPECT_TRUE(server->fieldContext()->current());
  // Nothing marked: the whole field reads +max_distance near the border.
  EXPECT_FLOAT_EQ(snapshot->distance.front(), 3.0f);
}

TEST_F(RosemapServerPipeline, LaserHitMarksAndDistanceFieldForms) {
  // A single hit at world (1.15, 0.05) -- a cell center at 0.1 resolution
  // (laser at (0.1, 0)).
  const double range = std::hypot(1.05, 0.05);
  const double angle = std::atan2(0.05, 1.05);
  server->handleObstacleLaserScan(makeScan(range, angle));
  server->waitForMapIdle(2.0);

  const auto snapshot = server->fieldContext()->currentSnapshot();
  ASSERT_NE(snapshot, nullptr);
  // Hit cell occupied; distance at the hit = -0.1 (interior depth).
  double mx = 0.0;
  double my = 0.0;
  ASSERT_TRUE(snapshot->worldToMap(1.15, 0.05, mx, my));
  EXPECT_EQ(snapshot->occupiedAt(static_cast<int>(std::lround(mx)),
                                 static_cast<int>(std::lround(my))),
            1);
  const auto at_hit = server->fieldContext()->queryRosemap(1.15, 0.05);
  EXPECT_NEAR(at_hit.distance, -0.1, 1e-3);
  // 0.5 m in front of the hit, on a cell center: distance = +0.5, gradient
  // -x (downhill toward the obstacle).
  const auto west = server->fieldContext()->queryRosemap(0.65, 0.05);
  EXPECT_NEAR(west.distance, 0.5, 1e-3);
  EXPECT_LT(west.gradient.x(), -0.9);
  // A path crossing the obstacle is invalid; the family probe rejects it.
  ayanami::nav_msgs::Path crossing;
  crossing.header.frame_id = "map";
  for (int i = 0; i <= 26; ++i) {
    ayanami::geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position = Eigen::Vector3d(0.2 + 0.05 * i, 0.0, 0.0);
    crossing.poses.push_back(pose);
  }
  EXPECT_FALSE(server->isPathValid(crossing));
}

TEST_F(RosemapServerPipeline, RaytraceClearsMarksWhenObstacleDisappears) {
  const double range = std::hypot(1.05, 0.05);
  const double angle = std::atan2(0.05, 1.05);
  server->handleObstacleLaserScan(makeScan(range, angle));
  server->waitForMapIdle(2.0);
  ASSERT_LT(server->fieldContext()->queryRosemap(1.15, 0.05).distance, 0.0)
      << "obstacle should be marked";

  // The obstacle is gone: a max-range (no-hit) ray sweeps the same sector.
  server->handleObstacleLaserScan(makeScan(30.0, angle));
  server->waitForMapIdle(2.0);
  EXPECT_GT(server->fieldContext()->queryRosemap(1.15, 0.05).distance, 0.0);
}

TEST_F(RosemapServerPipeline, ResetCommandDropsAllMarks) {
  server->handleObstacleLaserScan(makeScan(1.0, 0.0));
  server->waitForMapIdle(2.0);
  const std::uint64_t sequence_before = server->snapshotSequence();

  ayanami::rosemap::ResetFieldCommand command;
  EXPECT_TRUE(server->commandPort()->submit(command));
  EXPECT_GT(server->snapshotSequence(), sequence_before);
  EXPECT_GT(server->fieldContext()->queryRosemap(1.1, 0.0).distance, 0.0);
}

TEST_F(RosemapServerPipeline, WindowRecentresOnRobotMotion) {
  const double origin_before =
      server->fieldContext()->currentSnapshot()->origin_x;
  // Robot walks 1.5 m east (past the 1.0 m recentre threshold).
  Eigen::Isometry3d map_from_base = Eigen::Isometry3d::Identity();
  map_from_base.translation().x() = origin_before + 1.5;
  ayanami::testing::testTransforms().set("map", "base_link", map_from_base);
  server->handleObstacleLaserScan(makeScan(0.2, 0.0));
  server->waitForMapIdle(2.0);
  const double origin_after =
      server->fieldContext()->currentSnapshot()->origin_x;
  // The window recentred so the robot sits near its center.
  EXPECT_NEAR(origin_after + 2.0,
              map_from_base.translation().x(), 0.21);
}

// ---------------- Family hook typing ----------------

class FakeRosemapPlanner final : public ayanami::rosemap::RosemapPlanner {
 public:
  void configure(const ayanami::ModuleContext&,
                 RosemapMapContext::ConstPtr map) override {
    configured = true;
    map_context = std::move(map);
  }
  ResultT<void> makePlan(const ayanami::geometry_msgs::PoseStamped&,
                         const ayanami::geometry_msgs::PoseStamped&, double,
                         ayanami::nav_msgs::Path&) override {
    return ResultT<void>::failure(ErrorCode::INTERNAL_ERROR, "fake");
  }
  bool configured{false};
  RosemapMapContext::ConstPtr map_context;
};

class PlainPlanner final : public ayanami::abstract::AbstractPlanner {
 public:
  ResultT<void> makePlan(const ayanami::geometry_msgs::PoseStamped&,
                         const ayanami::geometry_msgs::PoseStamped&, double,
                         ayanami::nav_msgs::Path&) override {
    return ResultT<void>::failure(ErrorCode::INTERNAL_ERROR, "plain");
  }
};

TEST_F(RosemapServerPipeline, FamilyHooksInjectContextAndRejectForeign) {
  auto rosemap_planner = std::make_shared<FakeRosemapPlanner>();
  ayanami::Config empty_config;
  EXPECT_TRUE(server->initializePlannerModule("p", empty_config,
                                              rosemap_planner));
  EXPECT_TRUE(rosemap_planner->configured);
  ASSERT_NE(rosemap_planner->map_context, nullptr);
  EXPECT_EQ(rosemap_planner->map_context->mapFamily(), "rosemap");
  EXPECT_EQ(rosemap_planner->map_context->fieldFrame(), "map");

  auto plain = std::make_shared<PlainPlanner>();
  EXPECT_FALSE(server->initializePlannerModule("q", empty_config, plain));
}

}  // namespace
