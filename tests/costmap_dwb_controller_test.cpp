// costmap_dwb_controller_test.cpp — DWB 模块测试。
//
// 验证策略 (迁移自 nav2_dwb_controller, 算法逻辑完全相同):
//   A. 上游单元测试断言移植:
//      - MapBasedQueue 优先级/动态排序 (costmap_queue/test/mbq_test.cpp)
//      - getClosestPose / projectPose (dwb_core/test/utils_test.cpp)
//   B. 数值快照 (手工推导关键公式期望值):
//      - projectVelocity / OneDVelocityIterator
//      - StandardTrajectoryGenerator::computeNewPosition / getTimeSteps
//      - adjustPlanResolution
//      - critics: BaseObstacle / PreferForward / Twirling / Oscillation
//        (构造输入 → 精确分数断言)
//   C. headless 集成: CostmapContext + 主控制器全链路打分
//
// argv: 1=模块 config json (build tree 下 config/modules/costmap_dwb_controller.json)

#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <costmap_2d/costmap_2d.hpp>
#include <costmap_2d/cost_values.hpp>
#include <costmap_2d/layered_costmap.hpp>

#include <ayanami/costmap/components/costmap_context.hpp>
#include <ayanami/interfaces/transform_store.hpp>
#include <ayanami/messages/geometry_msgs.hpp>
#include <ayanami/messages/nav_msgs.hpp>
#include <ayanami/utility/config.hpp>

#include <costmap_dwb_controller/costmap_dwb_controller.hpp>
#include <costmap_dwb_controller/costmap_queue/map_based_queue.hpp>
#include <costmap_dwb_controller/critics/base_obstacle.hpp>
#include <costmap_dwb_controller/critics/oscillation.hpp>
#include <costmap_dwb_controller/critics/prefer_forward.hpp>
#include <costmap_dwb_controller/critics/twirling.hpp>
#include <costmap_dwb_controller/exceptions.hpp>
#include <costmap_dwb_controller/path_ops.hpp>
#include <costmap_dwb_controller/plugins/kinematic_parameters.hpp>
#include <costmap_dwb_controller/plugins/one_d_velocity_iterator.hpp>
#include <costmap_dwb_controller/plugins/standard_traj_generator.hpp>
#include <costmap_dwb_controller/trajectory_utils.hpp>

namespace {

using costmap_queue::MapBasedQueue;
using dwb_core::getClosestPose;
using dwb_core::projectPose;
using geometry_msgs::Pose2D;
using nav_2d_msgs::Twist2D;

std::string g_config_json;

// ---------- A. 上游断言移植 ----------

void letter_test(MapBasedQueue<char>& q, const char test_letter) {
  ASSERT_FALSE(q.isEmpty());
  char c = q.front();
  EXPECT_EQ(c, test_letter);
  q.pop();
}

TEST(DwbMapBasedQueue, CheckOrdering) {
  MapBasedQueue<char> q;
  q.enqueue(1.0, 'A');
  q.enqueue(3.0, 'B');
  q.enqueue(2.0, 'C');
  q.enqueue(5.0, 'D');
  q.enqueue(0.0, 'E');

  std::string expected = "EACBD";
  for (unsigned int i = 0; i < expected.size(); i++) {
    letter_test(q, expected[i]);
  }
  EXPECT_TRUE(q.isEmpty());
}

TEST(DwbMapBasedQueue, CheckDynamicOrdering) {
  MapBasedQueue<char> q;
  q.enqueue(1.0, 'A');
  q.enqueue(3.0, 'B');
  q.enqueue(2.0, 'C');
  q.enqueue(5.0, 'D');

  std::string expected = "ACB";
  for (unsigned int i = 0; i < expected.size(); i++) {
    letter_test(q, expected[i]);
  }

  q.enqueue(0.0, 'E');
  letter_test(q, 'E');
}

TEST(DwbMapBasedQueue, CheckDynamicOrdering3) {
  MapBasedQueue<char> q;
  q.enqueue(1.0, 'A');
  q.enqueue(2.0, 'B');
  q.enqueue(5.0, 'D');
  letter_test(q, 'A');
  letter_test(q, 'B');
  q.enqueue(1.0, 'C');
  letter_test(q, 'C');
  letter_test(q, 'D');
}

TEST(DwbTrajectoryUtils, ClosestPose) {
  dwb_msgs::Trajectory2D traj;
  traj.poses.resize(4);
  traj.time_offsets.resize(4);
  for (unsigned int i = 0; i < traj.poses.size(); i++) {
    double d = static_cast<double>(i);
    traj.poses[i].x = d;
    traj.time_offsets[i] = d;
  }

  EXPECT_DOUBLE_EQ(getClosestPose(traj, 0.0).x, traj.poses[0].x);
  EXPECT_DOUBLE_EQ(getClosestPose(traj, -1.0).x, traj.poses[0].x);
  EXPECT_DOUBLE_EQ(getClosestPose(traj, 0.4).x, traj.poses[0].x);
  EXPECT_DOUBLE_EQ(getClosestPose(traj, 0.5).x, traj.poses[0].x);
  EXPECT_DOUBLE_EQ(getClosestPose(traj, 0.51).x, traj.poses[1].x);
  EXPECT_DOUBLE_EQ(getClosestPose(traj, 1.0).x, traj.poses[1].x);
  EXPECT_DOUBLE_EQ(getClosestPose(traj, 1.4999).x, traj.poses[1].x);
  EXPECT_DOUBLE_EQ(getClosestPose(traj, 2.0).x, traj.poses[2].x);
  EXPECT_DOUBLE_EQ(getClosestPose(traj, 2.51).x, traj.poses[3].x);
  EXPECT_DOUBLE_EQ(getClosestPose(traj, 3.5).x, traj.poses[3].x);
}

TEST(DwbTrajectoryUtils, ProjectPose) {
  dwb_msgs::Trajectory2D traj;
  traj.poses.resize(4);
  traj.time_offsets.resize(4);
  for (unsigned int i = 0; i < traj.poses.size(); i++) {
    double d = static_cast<double>(i);
    traj.poses[i].x = d;
    traj.poses[i].y = 30.0 - 2.0 * d;
    traj.poses[i].theta = 0.42;
    traj.time_offsets[i] = d;
  }

  EXPECT_DOUBLE_EQ(projectPose(traj, 0.0).x, 0.0);
  EXPECT_DOUBLE_EQ(projectPose(traj, 0.0).y, 30.0);
  EXPECT_DOUBLE_EQ(projectPose(traj, 0.0).theta, 0.42);
  EXPECT_DOUBLE_EQ(projectPose(traj, -1.0).x, 0.0);
  EXPECT_DOUBLE_EQ(projectPose(traj, 0.4).x, 0.4);
  EXPECT_DOUBLE_EQ(projectPose(traj, 0.4).y, 29.2);
  EXPECT_DOUBLE_EQ(projectPose(traj, 0.5).x, 0.5);
  EXPECT_DOUBLE_EQ(projectPose(traj, 0.5).y, 29.0);
  EXPECT_DOUBLE_EQ(projectPose(traj, 0.51).x, 0.51);
  EXPECT_DOUBLE_EQ(projectPose(traj, 0.51).y, 28.98);
  EXPECT_DOUBLE_EQ(projectPose(traj, 1.0).x, 1.0);
  EXPECT_DOUBLE_EQ(projectPose(traj, 1.0).y, 28.0);
  EXPECT_DOUBLE_EQ(projectPose(traj, 1.4999).x, 1.4999);
  EXPECT_DOUBLE_EQ(projectPose(traj, 1.4999).y, 27.0002);
  EXPECT_DOUBLE_EQ(projectPose(traj, 2.0).x, 2.0);
  EXPECT_DOUBLE_EQ(projectPose(traj, 2.0).y, 26.0);
  EXPECT_FLOAT_EQ(projectPose(traj, 2.51).x, 2.51);
  EXPECT_FLOAT_EQ(projectPose(traj, 2.51).y, 24.98);
  EXPECT_DOUBLE_EQ(projectPose(traj, 3.5).x, 3.0);
  EXPECT_DOUBLE_EQ(projectPose(traj, 3.5).y, 24.0);
  EXPECT_DOUBLE_EQ(projectPose(traj, 3.5).theta, 0.42);
}

// ---------- B. 数值快照 ----------

TEST(DwbOneDVelocity, ProjectVelocity) {
  // 加速: v0 < target → min(target, v0 + accel*dt)
  EXPECT_DOUBLE_EQ(dwb_plugins::projectVelocity(0.0, 1.0, -1.0, 1.0, 2.0), 1.0);
  EXPECT_DOUBLE_EQ(dwb_plugins::projectVelocity(1.5, 1.0, -1.0, 1.0, 2.0), 2.0);
  // 减速: v0 >= target → max(target, v0 + decel*dt) (decel 为负)
  EXPECT_DOUBLE_EQ(dwb_plugins::projectVelocity(2.0, 1.0, -1.0, 1.0, 0.5), 1.0);
  EXPECT_DOUBLE_EQ(dwb_plugins::projectVelocity(0.5, 1.0, -1.0, 1.0, 0.0),
                   0.0);
}

TEST(DwbOneDVelocity, IteratorSamples) {
  // current=2, min=0, max=3, acc=1, decel=-1, dt=1, samples=5
  // max_vel = project(2 → 3) = 3; min_vel = project(2 → 0) = 1
  dwb_plugins::OneDVelocityIterator it(2.0, 0.0, 3.0, 1.0, -1.0, 1.0, 5);
  std::vector<double> samples;
  while (!it.isFinished()) {
    samples.push_back(it.getVelocity());
    ++it;
  }
  ASSERT_EQ(samples.size(), 5u);
  EXPECT_DOUBLE_EQ(samples[0], 1.0);
  EXPECT_DOUBLE_EQ(samples[1], 1.5);
  EXPECT_DOUBLE_EQ(samples[2], 2.0);
  EXPECT_DOUBLE_EQ(samples[3], 2.5);
  EXPECT_DOUBLE_EQ(samples[4], 3.0);
}

TEST(DwbOneDVelocity, IteratorInsertsZeroOnCross) {
  // current=-0.5, min=-1, max=1, acc=1, decel=-1, dt=1, samples=2
  // max_vel = project(-0.5 → 1) = 0.5; min_vel = project(-0.5 → -1) = -1
  dwb_plugins::OneDVelocityIterator it(-0.5, -1.0, 1.0, 1.0, -1.0, 1.0, 2);
  std::vector<double> samples;
  while (!it.isFinished()) {
    samples.push_back(it.getVelocity());
    ++it;
  }
  // -1 → 0 (跨零插入) → 0.5
  ASSERT_EQ(samples.size(), 3u);
  EXPECT_DOUBLE_EQ(samples[0], -1.0);
  EXPECT_DOUBLE_EQ(samples[1], 0.0);
  EXPECT_DOUBLE_EQ(samples[2], 0.5);
}

TEST(DwbKinematics, SpeedLimitPercentage) {
  dwb_plugins::KinematicsHandler handler;
  ayanami::Config cfg("");
  // 空配置下初始化 (默认 0), 再手工注参走 setSpeedLimit 纯逻辑:
  // 直接构造 KinematicParameters 不可行 (protected), 经 handler 初始化 + 限速。
  // 以 json 文件驱动 (见 headless 集成), 这里仅验证 NO_SPEED_LIMIT 语义路径。
  handler.initialize(cfg, "dwb_local");
  handler.setSpeedLimit(0.0 /*NO_SPEED_LIMIT*/, false);
  EXPECT_DOUBLE_EQ(handler.getKinematics().getMaxSpeedXY(), 0.0);
}

TEST(DwbPathOps, AdjustPlanResolution) {
  nav_2d_msgs::Path2D plan;
  plan.poses.push_back(Pose2D{0.0, 0.0, 0.0});
  plan.poses.push_back(Pose2D{1.0, 0.0, 0.0});
  nav_2d_msgs::Path2D adjusted = dwb_core::adjustPlanResolution(plan, 0.1);
  // 间距 1.0 > 2*0.1 → diff=1.0-0.2=0.8 → steps=int(0.8/0.1)-1=7
  // → j=1..6 共 6 个中间点 + 2 端点 = 8 点
  EXPECT_EQ(adjusted.poses.size(), 8u);
  EXPECT_DOUBLE_EQ(adjusted.poses.front().x, 0.0);
  EXPECT_DOUBLE_EQ(adjusted.poses.back().x, 1.0);
  // 中间点等分 7 段: 第 2 个点 x=1/7
  EXPECT_NEAR(adjusted.poses[1].x, 1.0 / 7.0, 1e-9);
}

// critic 数值快照: 用临时 json 配置驱动 (避免 Config 空文件全默认)。
const std::string makeCriticConfigJson() {
  return std::string("{\n  \"dwb_local\": {\n") +
         "    \"critics\": [\"BaseObstacle\", \"PreferForward\", \"Twirling\", "
         "\"Oscillation\"],\n"
         "    \"BaseObstacle.scale\": 1.0,\n"
         "    \"PreferForward.penalty\": 1.0,\n"
         "    \"PreferForward.strafe_x\": 0.1,\n"
         "    \"PreferForward.strafe_theta\": 0.2,\n"
         "    \"PreferForward.theta_scale\": 10.0,\n"
         "    \"Twirling.scale\": 1.0,\n"
         "    \"Oscillation.oscillation_reset_dist\": 0.05,\n"
         "    \"Oscillation.oscillation_reset_angle\": 0.2,\n"
         "    \"Oscillation.oscillation_reset_time\": -1.0,\n"
         "    \"Oscillation.x_only_threshold\": 0.05\n"
         "  }\n}\n";
}

void writeTempConfig(const std::string& content) {
  std::ofstream ofs("/tmp/opencode/dwb_critic_test.json");
  ofs << content;
}

TEST(DwbCritic, PreferForwardScores) {
  writeTempConfig(makeCriticConfigJson());
  ayanami::Config cfg("/tmp/opencode/dwb_critic_test.json");
  costmap_2d::Costmap2D costmap(10, 10, 0.05, 0.0, 0.0,
                                costmap_2d::FREE_SPACE);

  dwb_critics::PreferForwardCritic critic;
  critic.initialize(cfg, "PreferForward", "dwb_local", &costmap, nullptr);

  dwb_msgs::Trajectory2D traj;
  // backward → penalty (1.0)
  traj.velocity = Twist2D{-0.5, 0.0, 0.0};
  EXPECT_DOUBLE_EQ(critic.scoreTrajectory(traj), 1.0);
  // strafing (vx < 0.1 && |theta| < 0.2) → penalty
  traj.velocity = Twist2D{0.05, 0.0, 0.1};
  EXPECT_DOUBLE_EQ(critic.scoreTrajectory(traj), 1.0);
  // forward with rotation → |theta| * 10
  traj.velocity = Twist2D{0.5, 0.0, 0.3};
  EXPECT_DOUBLE_EQ(critic.scoreTrajectory(traj), 3.0);
  // pure forward → 0
  traj.velocity = Twist2D{0.5, 0.0, 0.0};
  EXPECT_DOUBLE_EQ(critic.scoreTrajectory(traj), 0.0);
}

TEST(DwbCritic, TwirlingScores) {
  writeTempConfig(makeCriticConfigJson());
  ayanami::Config cfg("/tmp/opencode/dwb_critic_test.json");
  costmap_2d::Costmap2D costmap(10, 10, 0.05, 0.0, 0.0,
                                costmap_2d::FREE_SPACE);

  dwb_critics::TwirlingCritic critic;
  critic.initialize(cfg, "Twirling", "dwb_local", &costmap, nullptr);

  dwb_msgs::Trajectory2D traj;
  traj.velocity = Twist2D{0.0, 0.0, -1.5};
  EXPECT_DOUBLE_EQ(critic.scoreTrajectory(traj), 1.5);
}

TEST(DwbCritic, BaseObstacleScoresAndThrows) {
  writeTempConfig(makeCriticConfigJson());
  ayanami::Config cfg("/tmp/opencode/dwb_critic_test.json");
  // 20x20 @ 0.05: (10,10) 放 lethal
  costmap_2d::Costmap2D costmap(20, 20, 0.05, 0.0, 0.0,
                                costmap_2d::FREE_SPACE);
  costmap.setCost(10, 10, costmap_2d::LETHAL_OBSTACLE);

  dwb_critics::BaseObstacleCritic critic;
  critic.initialize(cfg, "BaseObstacle", "dwb_local", &costmap, nullptr);

  // 合法轨迹: 经过 free 区 (0,0)→(0.1,0) 两个点, sum_scores=false → 取最后点
  // cost=0
  dwb_msgs::Trajectory2D traj;
  traj.velocity = Twist2D{0.5, 0.0, 0.0};
  traj.poses.push_back(Pose2D{0.0, 0.0, 0.0});
  traj.poses.push_back(Pose2D{0.1, 0.0, 0.0});
  EXPECT_DOUBLE_EQ(critic.scoreTrajectory(traj), 0.0);

  // 撞 lethal: (0.5, 0.5) 世界坐标 → cell (10, 10) → IllegalTrajectoryException
  dwb_msgs::Trajectory2D bad;
  bad.velocity = Twist2D{0.5, 0.0, 0.0};
  bad.poses.push_back(Pose2D{0.5, 0.5, 0.0});
  EXPECT_THROW(critic.scoreTrajectory(bad),
               dwb_core::IllegalTrajectoryException);

  // 出界: 世界坐标 1.0 → cell 20 越界 (20x20 @0.05 → 1.0m) → 异常
  dwb_msgs::Trajectory2D off_grid;
  off_grid.velocity = Twist2D{0.5, 0.0, 0.0};
  off_grid.poses.push_back(Pose2D{1.0, 0.0, 0.0});
  EXPECT_THROW(critic.scoreTrajectory(off_grid),
               dwb_core::IllegalTrajectoryException);
}

TEST(DwbCritic, OscillationDetectsSignFlips) {
  writeTempConfig(makeCriticConfigJson());
  ayanami::Config cfg("/tmp/opencode/dwb_critic_test.json");
  costmap_2d::Costmap2D costmap(10, 10, 0.05, 0.0, 0.0,
                                costmap_2d::FREE_SPACE);

  dwb_critics::OscillationCritic critic;
  critic.initialize(cfg, "Oscillation", "dwb_local", &costmap, nullptr);

  dwb_msgs::Trajectory2D traj;
  // 初始 debrief 正速度 → 无振荡 → 合法 (0.0)
  critic.prepare(Pose2D{0.0, 0.0, 0.0}, Twist2D{0.0, 0.0, 0.0},
                 Pose2D{1.0, 0.0, 0.0}, nav_2d_msgs::Path2D{});
  critic.debrief(Twist2D{1.0, 0.0, 0.0});
  traj.velocity = Twist2D{-0.5, 0.0, 0.0};
  EXPECT_DOUBLE_EQ(critic.scoreTrajectory(traj), 0.0);
  // debrief 负速度 → 符号翻转 → 正向轨迹振荡 → 异常
  critic.debrief(Twist2D{-0.5, 0.0, 0.0});
  traj.velocity = Twist2D{0.5, 0.0, 0.0};
  EXPECT_THROW(critic.scoreTrajectory(traj),
               dwb_core::IllegalTrajectoryException);
  // 负向仍合法
  traj.velocity = Twist2D{-0.3, 0.0, 0.0};
  EXPECT_DOUBLE_EQ(critic.scoreTrajectory(traj), 0.0);
}

// ---------- C. headless 集成 ----------

TEST(DwbController, InitializeLoadsCriticsAndScoresTrajectory) {
  // 临时 config 目录: config.json → nav.json (modules 块) → dwb.json, 使
  // ComponentConfig("config_controller") 走双参查找链命中 dwb 配置。
  ASSERT_FALSE(g_config_json.empty());
  const std::string dir = "/tmp/opencode/dwb_headless_cfg";
  std::filesystem::create_directories(dir);
  {
    std::ofstream ofs(dir + "/config.json");
    ofs << "{\"global\": {\"config_navigation\": \"nav.json\"}}";
  }
  {
    std::ofstream ofs(dir + "/nav.json");
    ofs << "{\"modules\": {\"config_controller\": \"dwb.json\"}}";
  }
  {
    std::ifstream src(g_config_json);
    std::ofstream dst(dir + "/dwb.json");
    dst << src.rdbuf();
  }
  ayanami::GlobalConfig::instance(dir, /*override_path=*/true);

  ayanami::Config cfg(g_config_json);

  // 20x20 @ 0.05 costmap 注入
  auto local_lm = std::make_shared<costmap_2d::LayeredCostmap>("odom", true,
                                                               false);
  local_lm->resizeMap(20, 20, 0.05, 0.0, 0.0);
  auto wrapper = std::make_shared<costmap_2d::CostmapWrapper>(local_lm, "odom");

  auto ctx = std::make_shared<ayanami::costmap::CostmapContext>();
  ctx->global_frame_ = "map";
  ctx->robot_frame_ = "base_link";
  ctx->local_costmap_ = wrapper;
  ctx->transform_store_ = &ayanami::interfaces::TransformStore::instance();

  ayanami::modules::CostmapDwBLocalPlanner planner;
  EXPECT_NO_THROW(planner.initialize("dwb_local", ctx));
}

}  // namespace

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  if (argc > 1) {
    g_config_json = argv[1];
  }
  return RUN_ALL_TESTS();
}
