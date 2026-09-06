// local_window_rolling_test: 局部滚动代价地图窗口必须跟随机器人。
//
// 背景 bug (rviz 实测): 机器人向西行驶 ~1.5m 后卡死 —— local rolling 窗口
// 钉死在世界原点 (origin 恒 (-1.5,-1.5)), 机器人撞上自己窗口的西边缘后,
// DWB 所有前进轨迹出界无效, 只能原地抖动。
//
// 本测试锁死语义: updateMapUnlocked / handleObstacle* 以机器人位姿为中心
// 滚动 local 窗口 (LayeredCostmap::updateMap → updateOrigin)。

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
  // 一圈均匀命中 (无 inf), 触发 mark + raytrace 管线。
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

  // 机器人 (base_link) 在 odom (3,3), laser 与车体同位。
  auto& rsc = ayanami::components::RobotStateController::instance();
  rsc.initialize("map", "odom", "base_link", 5.0);
  Eigen::Isometry3d t_odom_base = Eigen::Isometry3d::Identity();
  t_odom_base.translation() = Eigen::Vector3d(3.0, 3.0, 0.0);
  rsc.setTransform("odom", "base_link", ayanami::Timestamp::latest(),
                   t_odom_base, false);
  rsc.setTransform("odom", "laser", ayanami::Timestamp::latest(),
                   Eigen::Isometry3d::Identity(), false);

  server.handleObstacleLaserScan(makeScan());

  const double origin_x =
      server.localCostmap()->get().getCostmap()->getOriginX();
  const double origin_y =
      server.localCostmap()->get().getCostmap()->getOriginY();

  GTEST_SUCCEED() << "origin: initial=(" << initial_origin_x << ","
                  << initial_origin_y << ") after=(" << origin_x << ","
                  << origin_y << ")";

  // 滚动语义: 窗口中心跟随机器人 (3,3) → origin ≈ (1.5,1.5)。
  EXPECT_NEAR(origin_x, 1.5, 0.11) << "local window did not roll to robot";
  EXPECT_NEAR(origin_y, 1.5, 0.11) << "local window did not roll to robot";
}

// ---- 定位修正跳变: 全局动态层整体清空 (静态层保留) ----

TEST(LocalWindowRolling, localizationJumpClearsGlobalDynamicLayer) {
  auto dir = makeRollingConfigDir();
  ayanami::costmap::CostmapNavigationServer server;
  server.configureCoreMaps(dir.string());

  auto& rsc = ayanami::components::RobotStateController::instance();
  rsc.initialize("map", "odom", "base_link", 5.0);
  // 机器人置于原点 (全局图 1m x 1m, 扫描端点 0.3m 全部在界内)。
  // fast_tf 注意事项: (1) 动态边 set 对同一时间戳不覆盖 (emplace 语义),
  // 必须用递增时间戳, 否则测试 1 留下的旧样本无法被覆盖; (2) 树跨测试残留,
  // 必须经 (map,odom) 挂载 (与 live 50Hz 同步同路径), 直接 (map,base_link)
  // 会触发 "cannot merge unconnected trees"。
  Eigen::Isometry3d t_map_base = Eigen::Isometry3d::Identity();
  int64_t stamp_ns = 1000000000;  // 1s 起, 每条边递增
  auto next_stamp = [&stamp_ns] {
    return ayanami::Timestamp(std::chrono::nanoseconds(stamp_ns += 1000000));
  };
 // 每周期三边等戳写入 (镜像 live 50Hz 同步): latest common time 语义下
 // 多跳链路统一切刻采样, 交错戳的单样本边会在公共时刻外无覆盖而失败。
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

  // 落图前基线 (static 层 20x20 全空 → 无 lethal)
  const int baseline = countGlobalLethal();

  // 注入一圈 0.3m 扫描 → 全局图出现动态标记 (端点保持在 1m 界内)
  server.handleObstacleLaserScan(makeScan(0.3f));
  const int after_mark = countGlobalLethal();
  EXPECT_GT(after_mark, baseline) << "scan marks did not reach global map";

  // 定位修正跳变处理: 清空全局动态层 (clear_radius<=0 = 整图动态层)
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

// ---- 滚动解耦: 无 scan 流时 spinOnce 周期滚动仍跟随机器人 ----

TEST(LocalWindowRolling, rollingWindowFollowsWithoutSensorStream) {
  auto dir = makeRollingConfigDir();
  ayanami::costmap::CostmapNavigationServer server;
  server.configureCoreMaps(dir.string());

  const double initial_origin_x =
      server.localCostmap()->get().getCostmap()->getOriginX();
  const double initial_origin_y =
      server.localCostmap()->get().getCostmap()->getOriginY();

 // 机器人置于 (2, 2), 不注入任何 scan
  auto& rsc = ayanami::components::RobotStateController::instance();
  rsc.initialize("map", "odom", "base_link", 5.0);
  int64_t stamp_ns = 3000000000;
  auto next_stamp = [&stamp_ns] {
    return ayanami::Timestamp(std::chrono::nanoseconds(stamp_ns += 1000000));
  };
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.translation() = Eigen::Vector3d(2.0, 2.0, 0.0);
  rsc.setTransform("odom", "base_link", next_stamp(), t, false);
  rsc.setTransform("odom", "laser", next_stamp(),
                   Eigen::Isometry3d::Identity(), false);
  rsc.setTransform("map", "odom", next_stamp(),
                   Eigen::Isometry3d::Identity(), false);

 // 多次调用周期滚动 (内部按 update_frequency 节流): 窗口必须跟随
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

// ---- isPathValid 最近点起点 (nav2 对齐): 已走过路段的占用不判无效 ----

TEST(LocalWindowRolling, isPathValidStartsAtClosestPointToRobot) {
  auto dir = makeRollingConfigDir();
  ayanami::costmap::CostmapNavigationServer server;
  server.configureCoreMaps(dir.string());

  auto& rsc = ayanami::components::RobotStateController::instance();
  rsc.initialize("map", "odom", "base_link", 5.0);
 // 时基必须晚于本进程内更早执行的测试 (fast_tf latest 取最新样本)
  int64_t stamp_ns = 4000000000;
  auto next_stamp = [&stamp_ns] {
    return ayanami::Timestamp(std::chrono::nanoseconds(stamp_ns += 1000000));
  };
 // 每周期三边等戳写入 (镜像 live 50Hz 同步, 见 localizationJump 注释)。
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

 // 路径: x 从 0.1 到 0.9, 每 0.1m 一点 (全局图 1m x 1m @0.05m)
  ayanami::nav_msgs::Path path;
  path.header.frame_id = "map";
  for (int i = 1; i <= 9; ++i) {
    ayanami::geometry_msgs::PoseStamped pt;
    pt.header.frame_id = "map";
    pt.pose.position = Eigen::Vector3d(0.1 * i, 0.0, 0.0);
    path.poses.push_back(pt);
  }

 // 在路径起点 (已走过段) 落一圈障碍: 机器人置 (0.2, 0)
  set_base(0.2, 0.0);
  server.handleObstacleLaserScan(makeScan(0.25f));

 // 机器人已前进到 x≈0.8 (路径末端): 起点障碍在"最近点"之前 → 有效
  set_base(0.8, 0.0);
  EXPECT_TRUE(server.isPathValid(path))
      << "occupied already-passed segment must not invalidate the path";

 // 机器人仍在起点: 最近点即被占据的起点 → 无效
  set_base(0.2, 0.0);
  EXPECT_FALSE(server.isPathValid(path))
      << "occupied segment at the robot's position must invalidate the path";
}

// ---- 落图时序: scan 戳查询 (采集时刻位姿) 而非 latest ----
// 场景: 机器人原地旋转, map<-odom 修正在 5s->6s 间平移 (0.3,0) (AMCL 修正
// 演化)。同一圆环 scan (半径 0.6) 分别标注 5s / 6s: 标记必须落在各自采集
// 时刻的 map 位姿 —— 用 latest 会把两帧都烙在最新修正位姿上 (旋转歪斜)。

TEST(LocalWindowRolling, obstacleMarksFollowScanStampNotLatest) {
  auto dir = makeRollingConfigDir();
  ayanami::costmap::CostmapNavigationServer server;
  server.configureCoreMaps(dir.string());

  auto& rsc = ayanami::components::RobotStateController::instance();
  rsc.initialize("map", "odom", "base_link", 5.0);
 // fast_tf 动态边缓冲只保留最近 500ms (dynamic_transform 默认 dur_) ——
 // 所有 TF 样本与 scan 戳必须收进单个 500ms 窗口。
 // 时间轴: 5.000 base/laser/map=id | 5.100 base/laser/map=id |
 // 5.200 base/laser/map=(0.3,0) | 5.300 同 | 5.400 同 (newest)
 // scan A @5.050: map<-odom 插值 = id -> mark (0.6, 0) 附近
 // scan B @5.250: map<-odom 插值 = (0.3, 0) -> mark (0.9, 0) 附近
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

 // Scan A @5.050s: 圆环 0.6, map<-odom 插值 = id -> bearing 0 标记 cell (11,0)
  ayanami::sensor_msgs::LaserScan scan_a = makeScan(0.6f);
  scan_a.header.stamp = at(0.05);
  server.handleObstacleLaserScan(scan_a);
  EXPECT_EQ(countCell(11, 0), costmap_2d::LETHAL_OBSTACLE)
      << "scan A (stamp 5.05s) mark missing at acquisition-time position";
  EXPECT_LT(countCell(18, 0), costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
      << "cell 18 must not be marked before scan B (latest 语义会提前烙上)";

 // Scan B @5.250s: 同一圆环, map<-odom 插值 = (0.3,0) -> cell (18,0)
  ayanami::sensor_msgs::LaserScan scan_b = makeScan(0.6f);
  scan_b.header.stamp = at(0.25);
  server.handleObstacleLaserScan(scan_b);
  EXPECT_EQ(countCell(17, 0), costmap_2d::LETHAL_OBSTACLE)
      << "scan B (stamp 5.25s) mark missing at acquisition-time position"
      " (bearing-0 端点 x=0.89998 -> cell 17)";
  EXPECT_LT(countCell(11, 0), costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
      << "cell 11 was ray-cleared by scan B (按戳落图语义)";
}

}  // namespace

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
