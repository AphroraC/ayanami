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
#include <memory>

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
  rsc.setTransform("odom", "base_link", next_stamp(), t_map_base, false);
  rsc.setTransform("odom", "laser", next_stamp(),
                   Eigen::Isometry3d::Identity(), false);
  rsc.setTransform("map", "odom", next_stamp(),
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

}  // namespace

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
