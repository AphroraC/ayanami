// local_window_rolling_test: the local rolling costmap window must follow the robot.
//
// Background bug (observed live in rviz): the robot froze after driving ~1.5m west -- the local rolling window
// was pinned to the world origin (origin stuck at (-1.5,-1.5)); after hitting the west edge of its own window,
// all forward controller trajectories were out-of-bounds/invalid and the robot could only jitter in place.
//
// This test locks the semantics: updateMapUnlocked / handleObstacle* roll the local window centered on the robot
// pose (LayeredCostmap::updateMap -> updateOrigin).

#include <gtest/gtest.h>

#include "test_transform_source.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <thread>

#include <Eigen/Geometry>

#include <ayanami/costmap/components/costmap_navigation_server.hpp>
#include <ayanami/abstract/components/robot_state_controller.hpp>
#include <ayanami/costmap/components/costmap_commands.hpp>
#include <ayanami/messages/sensor_msgs.hpp>

using ayanami::testing::testTransforms;
namespace {

std::filesystem::path makeRollingConfigDir() {
  auto dir = std::filesystem::temp_directory_path() / "ayanami_rolling_test";
  std::filesystem::create_directories(dir);
  {
    std::ofstream out(dir / "config.json");
    out << R"({"global": {"config_path": "", "config_navigation": "navigation.json"}})";
  }
  {
    std::ofstream out(dir / "navigation.json");
    // INLINE layer blocks (static composition): global = static+inflation,
    // local = obstacle+inflation with a laserscan source.
    out << R"({
      "navigation": {"robot_frame": "base_link", "velocity_timeout": 5.0},
      "costmap": {"global_frame": "map", "local_frame": "odom",
                   "resolution": 0.05, "robot_radius": 0.12},
      "costmap_global": {"rolling_window": false, "track_unknown": false,
                          "static_layer": {"size_x": 20, "size_y": 20,
                                            "resolution": 0.05},
                          "inflation_layer": {"inflation_radius": 0.3,
                                               "inscribed_radius": 0.1,
                                               "cost_scaling_factor": 2.0}},
      "costmap_local": {"size_x": 60, "size_y": 60, "resolution": 0.05,
                         "rolling_window": true, "track_unknown": false,
                         "update_frequency": 10.0,
                         "observation_keep_time": 0.5,
                         "obstacle_layer": {"combination": "overwrite",
                                             "footprint_clearing": true,
                                             "observation_sources": ["scan"],
                                             "scan": {"type": "laserscan",
                                                       "obstacle_max_range": 3.0,
                                                       "raytrace_max_range": 3.5}},
                         "inflation_layer": {"inflation_radius": 0.2,
                                              "inscribed_radius": 0.1,
                                              "cost_scaling_factor": 2.0}}
    })";
  }
  return dir;
}

ayanami::sensor_msgs::LaserScan makeScan(float range = 2.0f) {
  ayanami::sensor_msgs::LaserScan scan;
  scan.header.frame_id = "laser";
  scan.angle_min = -3.14159;
  scan.angle_max = 3.14159;
  scan.angle_increment = 0.05;
  scan.range_min = 0.1;
  scan.range_max = 3.5;
  // A uniform ring of hits (no inf), triggering the mark + raytrace pipeline.
  for (int i = 0; i < 126; ++i) {
    scan.ranges.push_back(range);
  }
  return scan;
}

TEST(LocalWindowRolling, localWindowFollowsRobotPose) {
  auto dir = makeRollingConfigDir();
  ayanami::costmap::CostmapNavigationServer server;
  server.configureCoreMaps(dir.string());

  const double initial_origin_x =
      server.localCostmap()->get().getCostmap()->getOriginX();
  const double initial_origin_y =
      server.localCostmap()->get().getCostmap()->getOriginY();

  // The robot (base_link) is at odom (3,3) with the laser co-located with the body.
  auto& rsc = ayanami::components::RobotStateController::instance();
  rsc.initialize("map", "map", "base_link", 5.0);
  Eigen::Isometry3d t_odom_base = Eigen::Isometry3d::Identity();
  t_odom_base.translation() = Eigen::Vector3d(3.0, 3.0, 0.0);
  testTransforms().set("odom", "base_link", ayanami::Timestamp::latest(),
                   t_odom_base, false);
  testTransforms().set("odom", "laser", ayanami::Timestamp::latest(),
                   Eigen::Isometry3d::Identity(), false);
  // Both costmaps share the map frame: provide the map<-odom edge so the
  // map->odom lookups resolve.
  testTransforms().set("map", "odom", ayanami::Timestamp::latest(),
                       Eigen::Isometry3d::Identity(), false);

  server.handleObstacleLaserScan(makeScan());

  const double origin_x =
      server.localCostmap()->get().getCostmap()->getOriginX();
  const double origin_y =
      server.localCostmap()->get().getCostmap()->getOriginY();

  GTEST_SUCCEED() << "origin: initial=(" << initial_origin_x << ","
                  << initial_origin_y << ") after=(" << origin_x << ","
                  << origin_y << ")";

  // Rolling semantics: the window center follows the robot (3,3) -> origin ~ (1.5,1.5).
  EXPECT_NEAR(origin_x, 1.5, 0.11) << "local window did not roll to robot";
  EXPECT_NEAR(origin_y, 1.5, 0.11) << "local window did not roll to robot";
}

// ---- Map-representation policy: laser marks reach the LOCAL map only; the global map is static+inflation and a global-targeted clear degrades to a no-op ----

TEST(LocalWindowRolling, laserMarksLocalOnlyAndGlobalStaysStatic) {
  auto dir = makeRollingConfigDir();
  ayanami::costmap::CostmapNavigationServer server;
  server.configureCoreMaps(dir.string());

  auto& rsc = ayanami::components::RobotStateController::instance();
  rsc.initialize("map", "map", "base_link", 5.0);
  // Robot at the origin (global map 1m x 1m; the 0.3m scan endpoints are all inside bounds).
  // fast_tf caveats: (1) a dynamic-edge set does not overwrite the same timestamp (emplace semantics);
  // timestamps must increase, otherwise old samples left by test 1 cannot be overwritten; (2) trees persist across tests,
  // so mount via (map,odom) (the same path as live 50Hz sync); mounting (map,base_link) directly
  // triggers "cannot merge unconnected trees".
  Eigen::Isometry3d t_map_base = Eigen::Isometry3d::Identity();
  int64_t stamp_ns = 1000000000;  // Starting at 1s, incremented per edge
  auto next_stamp = [&stamp_ns] {
    return ayanami::Timestamp(std::chrono::nanoseconds(stamp_ns += 1000000));
  };
 // All three edges are written with equal stamps each cycle (mirroring live 50Hz sync): under latest common time semantics
 // multi-hop chains sample one instant consistently; staggered-stamp single-sample edges would lack coverage outside the common instant and fail.
  const auto cycle_stamp = next_stamp();
  testTransforms().set("odom", "base_link", cycle_stamp, t_map_base, false);
  testTransforms().set("odom", "laser", cycle_stamp,
                   Eigen::Isometry3d::Identity(), false);
  testTransforms().set("map", "odom", cycle_stamp,
                   Eigen::Isometry3d::Identity(), false);

  auto countGlobalLethal = [&server] {
    const auto* grid =
        server.globalCostmap()->get().getCostmap();
    int lethal = 0;
    for (unsigned int j = 0; j < grid->getSizeInCellsY(); ++j) {
      for (unsigned int i = 0; i < grid->getSizeInCellsX(); ++i) {
        if (grid->getCost(i, j) >= 100) {
          ++lethal;
        }
      }
    }
    return lethal;
  };
  auto countLocalLethal = [&server] {
    const auto* grid = server.localCostmap()->get().getCostmap();
    int lethal = 0;
    for (unsigned int j = 0; j < grid->getSizeInCellsY(); ++j) {
      for (unsigned int i = 0; i < grid->getSizeInCellsX(); ++i) {
        if (grid->getCost(i, j) >= 100) {
          ++lethal;
        }
      }
    }
    return lethal;
  };

  // Baseline before mapping (static layer 20x20 all empty -> no lethal)
  const int baseline = countGlobalLethal();

  // Inject a 0.3m ring scan -> dynamic marks appear on the LOCAL map only;
  // the global map (static+inflation) must not receive any marks.
  server.handleObstacleLaserScan(makeScan(0.3f));
  EXPECT_GT(countLocalLethal(), 0) << "scan marks did not reach the local map";
  EXPECT_EQ(countGlobalLethal(), baseline)
      << "the global map must stay static (no dynamic marks)";

  // A global-targeted clear command is still accepted (no-op for a static
  // map), and the global content is unchanged.
  ayanami::costmap::ClearCostmapCommand global_clear;
  global_clear.affected_maps = "global";
  global_clear.clear_radius = 0.0;
  global_clear.enabled = true;
  EXPECT_TRUE(server.executeClearCommand(global_clear));
  EXPECT_EQ(countGlobalLethal(), baseline);

  // A local clear still clears the local dynamic layer.
  ayanami::costmap::ClearCostmapCommand local_clear;
  local_clear.affected_maps = "local";
  local_clear.clear_radius = 0.0;
  local_clear.enabled = true;
  EXPECT_TRUE(server.executeClearCommand(local_clear));
  EXPECT_EQ(countLocalLethal(), 0)
      << "local dynamic layer not cleared by the clear command";
}

// ---- Rolling decoupling: spinOnce periodic rolling follows the robot even without a scan stream ----

TEST(LocalWindowRolling, rollingWindowFollowsWithoutSensorStream) {
  auto dir = makeRollingConfigDir();
  ayanami::costmap::CostmapNavigationServer server;
  server.configureCoreMaps(dir.string());

  const double initial_origin_x =
      server.localCostmap()->get().getCostmap()->getOriginX();
  const double initial_origin_y =
      server.localCostmap()->get().getCostmap()->getOriginY();

 // Robot placed at (2, 2) with no scan injected
  auto& rsc = ayanami::components::RobotStateController::instance();
  rsc.initialize("map", "map", "base_link", 5.0);
  int64_t stamp_ns = 3000000000;
  // All edges share one stamp per cycle (latest-common-time semantics:
  // staggered single-sample edges have no coverage at the common instant).
  ayanami::Timestamp cycle_stamp(
      std::chrono::nanoseconds(stamp_ns += 1000000));
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.translation() = Eigen::Vector3d(2.0, 2.0, 0.0);
  testTransforms().set("odom", "base_link", cycle_stamp, t, false);
  testTransforms().set("odom", "laser", cycle_stamp,
                   Eigen::Isometry3d::Identity(), false);
  testTransforms().set("map", "odom", cycle_stamp,
                   Eigen::Isometry3d::Identity(), false);

 // Repeated periodic rolling calls (throttled internally by update_frequency): the window must follow
  for (int i = 0; i < 3; ++i) {
    server.updateRollingWindows();
    std::this_thread::sleep_for(std::chrono::milliseconds(110));
  }

  const double origin_x = server.localCostmap()->get().getCostmap()->getOriginX();
  const double origin_y = server.localCostmap()->get().getCostmap()->getOriginY();
  GTEST_SUCCEED() << "origin: initial=(" << initial_origin_x << ","
                  << initial_origin_y << ") after=(" << origin_x << ","
                  << origin_y << ")";
  EXPECT_NEAR(origin_x, 0.5, 0.11)
      << "rolling window did not follow robot without sensor stream";
  EXPECT_NEAR(origin_y, 0.5, 0.11)
      << "rolling window did not follow robot without sensor stream";
}

// ---- isPathValid starts at the closest point (nav2-aligned): occupancy on the already-traveled segment is not invalid ----
// The occupied cells are set on a directly built global wrapper instead of
// being stamped through the observation pipeline: the global grid gates
// the far path segments on its own (the local overlay adds near-window
// gating on top, absent here -- no local wrapper in this fixture).

TEST(LocalWindowRolling, isPathValidStartsAtClosestPointToRobot) {
  auto dir = makeRollingConfigDir();
  ayanami::costmap::CostmapNavigationServer server;
  server.configureCoreMaps(dir.string());

  auto& rsc = ayanami::components::RobotStateController::instance();
  rsc.initialize("map", "map", "base_link", 5.0);
 // The time base must be later than tests executed earlier in this process (fast_tf latest takes the newest samples)
  int64_t stamp_ns = 4000000000;
  auto next_stamp = [&stamp_ns] {
    return ayanami::Timestamp(std::chrono::nanoseconds(stamp_ns += 1000000));
  };
 // All three edges written with equal stamps per cycle (mirrors live 50Hz sync).
  auto set_base = [&](double x, double y) {
    const auto stamp = next_stamp();
    Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
    t.translation() = Eigen::Vector3d(x, y, 0.0);
    testTransforms().set("odom", "base_link", stamp, t, false);
    testTransforms().set("odom", "laser", stamp,
                     Eigen::Isometry3d::Identity(), false);
    testTransforms().set("map", "odom", stamp,
                     Eigen::Isometry3d::Identity(), false);
  };

 // Path: x from 0.1 to 0.9, one point every 0.1m (global map 1m x 1m @0.05m)
  ayanami::nav_msgs::Path path;
  path.header.frame_id = "map";
  for (int i = 1; i <= 9; ++i) {
    ayanami::geometry_msgs::PoseStamped pt;
    pt.header.frame_id = "map";
    pt.pose.position = Eigen::Vector3d(0.1 * i, 0.0, 0.0);
    path.poses.push_back(pt);
  }

 // Occupy the global map at the path start (world (0.2, 0) -> cell (4, 0)).
  {
    auto* grid = server.globalCostmap()->get().getCostmap();
    auto write_lock = server.globalCostmap()->lockForWrite();
    grid->setCost(4, 0, costmap_2d::LETHAL_OBSTACLE);
  }

 // The robot has advanced to x~0.8 (path end): the occupied start lies
 // before the "closest point" -> valid
  set_base(0.8, 0.0);
  EXPECT_TRUE(server.costmapContext()->isPathValid(path))
      << "occupied already-passed segment must not invalidate the path";

 // The robot stands ON the occupied start cell: the at-robot point is
 // exempt (it is where the robot already is -- failing here wedged a
 // band-grazing tracker into the stop-into-recovery deadlock); the verdict
 // flips to valid and FollowPath keeps driving it out.
  set_base(0.2, 0.0);
  EXPECT_TRUE(server.costmapContext()->isPathValid(path))
      << "the point at the robot's own position is exempt (it is already there)";

 // An occupied cell AHEAD of the robot, beyond the body radius, still gates:
 // add a lethal cell at world (0.6,0) (cell (12,0)); from the robot at the
 // path start it is 0.4m away -- probed and invalidating.
  {
    auto* grid = server.globalCostmap()->get().getCostmap();
    auto write_lock = server.globalCostmap()->lockForWrite();
    grid->setCost(12, 0, costmap_2d::LETHAL_OBSTACLE);
  }
  EXPECT_FALSE(server.costmapContext()->isPathValid(path))
      << "occupied segment beyond the body radius must invalidate the path";
}

// ---- Mapping timescales: the LOCAL map is immune to map<-odom corrections ----
// Scenario: the robot spins in place while the map<-odom correction translates by (0.3,0) between 5s and 6s (an AMCL
// correction evolution). Two ring scans are stamped at 5.05s / 5.25s. The local map lives in the ODOM frame, whose
// sensor edges the correction does not touch -- so BOTH scans must land at the SAME local cells. A latest-time or
// map-frame stamping implementation would shift the marks by the correction (or mark at the frame origin).

TEST(LocalWindowRolling, localMarksAreImmuneToLocalizationCorrection) {
  auto dir = makeRollingConfigDir();
  ayanami::costmap::CostmapNavigationServer server;
  server.configureCoreMaps(dir.string());

  auto& rsc = ayanami::components::RobotStateController::instance();
  rsc.initialize("map", "map", "base_link", 5.0);
 // The fast_tf dynamic-edge buffer keeps only the most recent 500ms (dynamic_transform default dur_) --
 // all TF samples and scan stamps must fit inside a single 500ms window.
 // Timeline: 5.000 base/laser/map=id | 5.100 base/laser/map=id |
 // 5.200 base/laser/map=(0.3,0) | 5.300 same | 5.400 same (newest)
 // scan A @5.050: stamp-time queries; scan B @5.250 with the shifted correction.
  auto identity = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d t_m_o = Eigen::Isometry3d::Identity();
  t_m_o.translation() = Eigen::Vector3d(0.3, 0.0, 0.0);
  int64_t stamp_ns = 5000000000;
  auto at = [&](double offset_s) {
    return ayanami::Timestamp(
        std::chrono::nanoseconds(stamp_ns +
                                 static_cast<int64_t>(offset_s * 1e9)));
  };
  for (double offset : {0.0, 0.1}) {
    testTransforms().set("odom", "base_link", at(offset), identity, false);
    testTransforms().set("odom", "laser", at(offset), identity, false);
    testTransforms().set("map", "odom", at(offset), identity, false);
  }
  for (double offset : {0.2, 0.3, 0.4}) {
    testTransforms().set("odom", "base_link", at(offset), identity, false);
    testTransforms().set("odom", "laser", at(offset), identity, false);
    testTransforms().set("map", "odom", at(offset), t_m_o, false);
  }

  auto countCell = [&server](unsigned int mx, unsigned int my) {
    return server.localCostmap()->get().getCostmap()->getCost(mx, my);
  };
  // The bearing-0 endpoint (world x=0.59998) sits on a cell rounding
  // boundary, so assert on a NEIGHBORHOOD: a lethal mark must exist around
  // world (0.6, 0) (cells 40..44, rows 29..31).
  auto hasLethalNear = [&](unsigned int mx_begin, unsigned int mx_end) {
    for (unsigned int my = 29; my <= 31; ++my) {
      for (unsigned int mx = mx_begin; mx <= mx_end; ++mx) {
        if (countCell(mx, my) >= costmap_2d::LETHAL_OBSTACLE) {
          return true;
        }
      }
    }
    return false;
  };

 // The local rolling map (3m x 3m @0.05m) centers on the robot at odom (0,0):
 // origin (-1.5,-1.5). Bearing-0 endpoint world (0.59998, 0.005) marks around
 // local cell (42, 30) -- for BOTH scans, because the correction moved the
 // map<-odom edge only and the local marks live in odom.
  ayanami::sensor_msgs::LaserScan scan_a = makeScan(0.6f);
  scan_a.header.stamp = at(0.05);
  server.handleObstacleLaserScan(scan_a);
  EXPECT_TRUE(hasLethalNear(40, 44))
      << "scan A (stamp 5.05s) mark missing at acquisition-time position";

  ayanami::sensor_msgs::LaserScan scan_b = makeScan(0.6f);
  scan_b.header.stamp = at(0.25);
  server.handleObstacleLaserScan(scan_b);
  EXPECT_TRUE(hasLethalNear(40, 44))
      << "scan B (stamp 5.25s) must land at the SAME local cells: the "
         "map<-odom correction must not shift odom-frame marks";
  EXPECT_FALSE(hasLethalNear(46, 50))
      << "a map-frame-style correction shift (x+0.3 -> cells ~48) must NOT "
         "appear in the local map";
}

}  // namespace

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
