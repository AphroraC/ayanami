// local_window_rolling_test: the local rolling costmap window must follow the robot.
//
// Background bug (observed live in rviz): the robot froze after driving ~1.5m west -- the local rolling window
// was pinned to the world origin (origin stuck at (-1.5,-1.5)); after hitting the west edge of its own window,
// all forward DWB trajectories were out-of-bounds/invalid and the robot could only jitter in place.
//
// This test locks the semantics: updateMapUnlocked / handleObstacle* roll the local window centered on the robot
// pose (LayeredCostmap::updateMap -> updateOrigin).

#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <thread>

#include <Eigen/Geometry>

#include <ayanami/costmap/components/costmap_navigation_server.hpp>
#include <ayanami/abstract/components/robot_state_controller.hpp>
#include <ayanami/costmap/core/costmap_command.hpp>
#include <ayanami/messages/sensor_msgs.hpp>

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
    out << R"({
      "navigation": {"robot_frame": "base_link", "velocity_timeout": 5.0},
      "costmap": {"global_frame": "map", "local_frame": "odom",
                   "resolution": 0.05, "robot_radius": 0.12},
      "costmap_global": {"size_x": 20, "size_y": 20, "rolling_window": false,
                          "track_unknown": false, "inflation_radius": 0.3,
                          "inscribed_radius": 0.1, "cost_scaling_factor": 2.0},
      "costmap_local": {"size_x": 60, "size_y": 60, "resolution": 0.05,
                         "rolling_window": true, "track_unknown": false,
                         "inflation_radius": 0.2, "inscribed_radius": 0.1,
                         "cost_scaling_factor": 2.0, "obstacle_range": 3.0,
                         "raytrace_range": 3.5}
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
  rsc.setTransform("odom", "base_link", ayanami::Timestamp::latest(),
                   t_odom_base, false);
  rsc.setTransform("odom", "laser", ayanami::Timestamp::latest(),
                   Eigen::Isometry3d::Identity(), false);
  // Both costmaps share the map frame: provide the map<-odom edge so the
  // map-frame robot pose resolves.
  rsc.setTransform("map", "odom", ayanami::Timestamp::latest(),
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

// ---- Localization correction jump: the global dynamic layer is cleared wholesale (static layer kept) ----

TEST(LocalWindowRolling, localizationJumpClearsGlobalDynamicLayer) {
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
  rsc.setTransform("odom", "base_link", cycle_stamp, t_map_base, false);
  rsc.setTransform("odom", "laser", cycle_stamp,
                   Eigen::Isometry3d::Identity(), false);
  rsc.setTransform("map", "odom", cycle_stamp,
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

  // Baseline before mapping (static layer 20x20 all empty -> no lethal)
  const int baseline = countGlobalLethal();

  // Inject a 0.3m ring scan -> dynamic marks appear on the global map (endpoints stay within the 1m bounds)
  server.handleObstacleLaserScan(makeScan(0.3f));
  const int after_mark = countGlobalLethal();
  EXPECT_GT(after_mark, baseline) << "scan marks did not reach global map";

  // Localization correction jump handling: clear the global dynamic layer (clear_radius<=0 = the whole dynamic layer)
  ayanami::costmap::ClearCostmapCommand command;
  command.affected_maps = "global";
  command.clear_radius = 0.0;
  command.enabled = true;
  EXPECT_TRUE(server.executeClearCommand(command));

  EXPECT_LE(countGlobalLethal(), baseline)
      << "global dynamic layer not cleared on localization jump";
  {
    const auto* grid = server.globalCostmap()->get().getCostmap();
    int n100 = 0, n253 = 0, n_other = 0;
    for (unsigned int j = 0; j < grid->getSizeInCellsY(); ++j) {
      for (unsigned int i = 0; i < grid->getSizeInCellsX(); ++i) {
        const auto c = grid->getCost(i, j);
        if (c >= 100) ++n100;
        else if (c == 253) ++n253;
        else if (c != 0) ++n_other;
      }
    }
    GTEST_SUCCEED() << "post-clear histogram: >=100=" << n100
                    << " inscribed=" << n253 << " other=" << n_other
                    << " baseline=" << baseline;
  }
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
  rsc.setTransform("odom", "base_link", cycle_stamp, t, false);
  rsc.setTransform("odom", "laser", cycle_stamp,
                   Eigen::Isometry3d::Identity(), false);
  rsc.setTransform("map", "odom", cycle_stamp,
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
 // All three edges written with equal stamps per cycle (mirrors live 50Hz sync; see the localizationJump comment).
  auto set_base = [&](double x, double y) {
    const auto stamp = next_stamp();
    Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
    t.translation() = Eigen::Vector3d(x, y, 0.0);
    rsc.setTransform("odom", "base_link", stamp, t, false);
    rsc.setTransform("odom", "laser", stamp,
                     Eigen::Isometry3d::Identity(), false);
    rsc.setTransform("map", "odom", stamp,
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

 // Drop a ring of obstacles at the path start (already-traveled segment): robot placed at (0.2, 0)
  set_base(0.2, 0.0);
  server.handleObstacleLaserScan(makeScan(0.25f));

 // The robot has advanced to x~0.8 (path end): the start obstacles lie before the "closest point" -> valid
  set_base(0.8, 0.0);
  EXPECT_TRUE(server.isPathValid(path))
      << "occupied already-passed segment must not invalidate the path";

 // The robot is still at the start: the closest point is the occupied start -> invalid
  set_base(0.2, 0.0);
  EXPECT_FALSE(server.isPathValid(path))
      << "occupied segment at the robot's position must invalidate the path";
}

// ---- Mapping timing: scans are queried by stamp (pose at capture time), not latest ----
// Scenario: the robot spins in place while the map<-odom correction translates by (0.3,0) between 5s and 6s (an AMCL
// correction evolution). The same ring scan (radius 0.6) is stamped 5s / 6s: the marks must land at the map pose of
// their own capture time -- using latest would stamp both frames with the newest corrected pose (rotational skew).

TEST(LocalWindowRolling, obstacleMarksFollowScanStampNotLatest) {
  auto dir = makeRollingConfigDir();
  ayanami::costmap::CostmapNavigationServer server;
  server.configureCoreMaps(dir.string());

  auto& rsc = ayanami::components::RobotStateController::instance();
  rsc.initialize("map", "map", "base_link", 5.0);
 // The fast_tf dynamic-edge buffer keeps only the most recent 500ms (dynamic_transform default dur_) --
 // all TF samples and scan stamps must fit inside a single 500ms window.
 // Timeline: 5.000 base/laser/map=id | 5.100 base/laser/map=id |
 // 5.200 base/laser/map=(0.3,0) | 5.300 same | 5.400 same (newest)
 // scan A @5.050: map<-odom interpolation = id -> mark near (0.6, 0)
 // scan B @5.250: map<-odom interpolation = (0.3, 0) -> mark near (0.9, 0)
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
    rsc.setTransform("odom", "base_link", at(offset), identity, false);
    rsc.setTransform("odom", "laser", at(offset), identity, false);
    rsc.setTransform("map", "odom", at(offset), identity, false);
  }
  for (double offset : {0.2, 0.3, 0.4}) {
    rsc.setTransform("odom", "base_link", at(offset), identity, false);
    rsc.setTransform("odom", "laser", at(offset), identity, false);
    rsc.setTransform("map", "odom", at(offset), t_m_o, false);
  }

  auto countCell = [&server](unsigned int mx, unsigned int my) {
    return server.globalCostmap()->get().getCostmap()->getCost(mx, my);
  };

 // Scan A @5.050s: ring 0.6, map<-odom interpolation = id -> bearing 0 marks cell (11,0)
  ayanami::sensor_msgs::LaserScan scan_a = makeScan(0.6f);
  scan_a.header.stamp = at(0.05);
  server.handleObstacleLaserScan(scan_a);
  EXPECT_EQ(countCell(11, 0), costmap_2d::LETHAL_OBSTACLE)
      << "scan A (stamp 5.05s) mark missing at acquisition-time position";
  EXPECT_LT(countCell(18, 0), costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
      << "cell 18 must not be marked before scan B (latest-time semantics would mark it early)";

 // Scan B @5.250s: same ring, map<-odom interpolation = (0.3,0) -> cell (18,0)
  ayanami::sensor_msgs::LaserScan scan_b = makeScan(0.6f);
  scan_b.header.stamp = at(0.25);
  server.handleObstacleLaserScan(scan_b);
  EXPECT_EQ(countCell(17, 0), costmap_2d::LETHAL_OBSTACLE)
      << "scan B (stamp 5.25s) mark missing at acquisition-time position"
      " (bearing-0 endpoint x=0.89998 -> cell 17)";
  EXPECT_LT(countCell(11, 0), costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
      << "cell 11 was ray-cleared by scan B (stamp-time stamping semantics)";
}

}  // namespace

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
