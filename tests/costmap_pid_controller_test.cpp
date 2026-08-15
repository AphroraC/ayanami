// Task 23: modules/costmap_pid_controller (CostmapPidController: 3 轴 PID +
// 加速度限制 + Twist 清零 + 同 goal 不重置)。
//
// 覆盖:
// - 首帧 linear.x 是 max_acc*dt 的小 ramp (不是脏内存/尖峰)
// - 角速度 ramp: setPlan 后首帧 |angular.z| <= max_angular_accel*dt, 后续逐帧爬坡
// - 同 plan 重复 setPlan 不重置 PID/last_vel 状态; 换不同 goal 才 reset
// - isGoalReached: dist < dist_tol → true 且该帧 cmd_vel 零速度
// - 多帧模拟 → 到达 goal → 零速度停止
// - cancel() 中途 → 返回 100+ CANCELED outcome + 零速度
// - Twist 清零: linear.z / angular.x / angular.y 恒 0, 无 NaN/脏内存
// - module.cpp 导出 create_controller_module 符号 (dlopen 验证)

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <string>

#include <Eigen/Geometry>

#include <ayanami/core/abstract_controller.hpp>
#include <ayanami/messages/actions.hpp>
#include <ayanami/messages/geometry_msgs.hpp>
#include <ayanami/messages/nav_msgs.hpp>
#include <ayanami/modules/costmap_pid_controller/costmap_pid_controller.hpp>
#include <ayanami/utility/config.hpp>
#include <ayanami/utility/load_module.hpp>

namespace {

using ayanami::abstract::AbstractController;
using ayanami::geometry_msgs::PoseStamped;
using ayanami::geometry_msgs::TwistStamped;
using ayanami::modules::CostmapPidController;
using ayanami::nav_msgs::Path;

std::string g_module_so;
std::filesystem::path g_config_dir;

// 与 tests/main 写入的临时 costmap_pid_controller.json 数值一致。
constexpr double kMaxAcc = 1.0;
constexpr double kMaxAngAccel = 1.0;
constexpr double kDt = 0.05;
constexpr double kPi = 3.14159265358979323846;

PoseStamped makePose(double x, double y, double yaw = 0.0) {
  auto p = PoseStamped::fromIsometry("map", Eigen::Isometry3d::Identity());
  p.pose.translation() = Eigen::Vector3d(x, y, 0.0);
  p.pose.linear() =
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return p;
}

// 沿 X 轴的 10 个 pose, 从 (0,0) 到 (goal_x, 0), 朝向 +X。
Path makeXPath(double goal_x = 9.0) {
  Path p;
  p.header.frame_id = "map";
  for (int i = 0; i < 10; ++i) {
    p.poses.push_back(makePose(static_cast<double>(i) / 9.0 * goal_x, 0.0));
  }
  return p;
}

std::shared_ptr<CostmapPidController> makeController() {
  return std::make_shared<CostmapPidController>();
}

TwistStamped compute(const std::shared_ptr<CostmapPidController>& ctl,
                     const PoseStamped& robot, uint32_t& outcome,
                     std::string& message) {
  TwistStamped cmd;
  outcome = ctl->computeVelocityCommands(robot, TwistStamped{}, cmd, message);
  return cmd;
}

std::string configContent(const std::string& mode) {
  std::string content = "{\n";
  content += "  \"costmap_pid_controller\": {\n";
  content += "    \"max_v\": 2.0,\n";
  content += "    \"max_w\": 1.0,\n";
  content += "    \"max_acc\": 1.0,\n";
  content += "    \"max_decel\": 1.0,\n";
  content += "    \"max_angular_accel\": 1.0,\n";
  content += "    \"kp_v\": 1.0,\n";
  content += "    \"ki_v\": 0.0,\n";
  content += "    \"kd_v\": 0.0,\n";
  content += "    \"kp_w\": 1.0,\n";
  content += "    \"ki_w\": 0.0,\n";
  content += "    \"kd_w\": 0.0,\n";
  content += "    \"dist_decay\": 2.0,\n";
  content += "    \"goal_tolerance\": 0.1,\n";
  content += "    \"slide_window\": 1.0,\n";
  content += "    \"default_dt\": 0.05,\n";
  content += "    \"heading_deadband\": 0.08,\n";
  content += "    \"rotate_in_place_angle\": 0.45,\n";
  content += "    \"mode\": \"" + mode + "\"\n";
  content += "  }\n";
  content += "}\n";
  return content;
}

void writeConfig(const std::string& mode) {
  std::ofstream ofs(g_config_dir / "costmap_pid_controller.json");
  ofs << configContent(mode);
}

}  // namespace

// ---------- 首帧 ramp (Twist 清零 + 加速度限制) ----------

TEST(CostmapPidControllerTest, FirstFrameLinearRampsNotSpike) {
  auto ctl = makeController();
  ASSERT_TRUE(ctl->setPlan(makeXPath()));

  uint32_t outcome = 0;
  std::string message;
  const TwistStamped cmd = compute(ctl, makePose(0.0, 0.0), outcome, message);

  EXPECT_EQ(outcome, ayanami::actions::ExePathResult::SUCCESS);
  // 必须是 max_acc*dt 的小 ramp 值, 不是脏内存大尖峰
  EXPECT_TRUE(std::isfinite(cmd.twist.linear.x()));
  EXPECT_GT(cmd.twist.linear.x(), 0.0);
  EXPECT_NEAR(cmd.twist.linear.x(), kMaxAcc * kDt, 1e-9);
  EXPECT_LT(cmd.twist.linear.x(), 0.2);
}

// ---------- 角速度 ramp ----------

TEST(CostmapPidControllerTest, AngularVelocityRampsWithMaxAngularAccel) {
  auto ctl = makeController();
  ASSERT_TRUE(ctl->setPlan(makeXPath()));

  // 车体朝向 +Y (与 +X 路径成 90°), 需要大角速度纠偏 → 验证首帧被 ramp 限住。
  const PoseStamped robot = makePose(0.0, 0.0, kPi / 2.0);
  uint32_t outcome = 0;
  std::string message;
  double prev_mag = 0.0;
  for (int i = 0; i < 5; ++i) {
    const TwistStamped cmd = compute(ctl, robot, outcome, message);
    ASSERT_EQ(outcome, ayanami::actions::ExePathResult::SUCCESS);
    const double mag = std::abs(cmd.twist.angular.z());
    EXPECT_GT(mag, 0.0);
    if (i == 0) {
      // 首帧必须受 max_angular_accel 限幅, 不是 PID 全值 (≈ kp_w*error ≈ π/2)
      EXPECT_LE(mag, kMaxAngAccel * kDt + 1e-9);
    }
    // 逐帧单调爬坡 → 证明是 ramping 而不是一次性大角速度
    EXPECT_GT(mag, prev_mag);
    prev_mag = mag;
  }
}

// ---------- 同 goal 不重置 / 新 goal 重置 ----------

TEST(CostmapPidControllerTest, RepeatSetPlanDoesNotResetState) {
  auto ctl = makeController();
  const Path plan = makeXPath();
  ASSERT_TRUE(ctl->setPlan(plan));

  const PoseStamped robot = makePose(0.5, 0.0);
  uint32_t outcome = 0;
  std::string message;
  const TwistStamped c1 = compute(ctl, robot, outcome, message);
  ASSERT_EQ(outcome, ayanami::actions::ExePathResult::SUCCESS);
  const TwistStamped c2 = compute(ctl, robot, outcome, message);
  ASSERT_EQ(outcome, ayanami::actions::ExePathResult::SUCCESS);
  ASSERT_GT(c2.twist.linear.x(), c1.twist.linear.x());

  // 同 plan 重复 setPlan → 不重置: ramp 连续 0.05→0.10→0.15
  ASSERT_TRUE(ctl->setPlan(plan));
  const TwistStamped c3 = compute(ctl, robot, outcome, message);
  ASSERT_EQ(outcome, ayanami::actions::ExePathResult::SUCCESS);

  EXPECT_NEAR(c1.twist.linear.x(), kMaxAcc * kDt, 1e-9);
  EXPECT_NEAR(c2.twist.linear.x(), 2.0 * kMaxAcc * kDt, 1e-9);
  EXPECT_NEAR(c3.twist.linear.x(), 3.0 * kMaxAcc * kDt, 1e-9);
  EXPECT_GT(c3.twist.linear.x(), c1.twist.linear.x());
}

TEST(CostmapPidControllerTest, NewGoalSetPlanResetsState) {
  auto ctl = makeController();
  ASSERT_TRUE(ctl->setPlan(makeXPath()));

  const PoseStamped robot = makePose(0.5, 0.0);
  uint32_t outcome = 0;
  std::string message;
  const TwistStamped c1 = compute(ctl, robot, outcome, message);
  ASSERT_EQ(outcome, ayanami::actions::ExePathResult::SUCCESS);
  const TwistStamped c2 = compute(ctl, robot, outcome, message);
  ASSERT_EQ(outcome, ayanami::actions::ExePathResult::SUCCESS);
  ASSERT_GT(c2.twist.linear.x(), c1.twist.linear.x());

  // 换不同 goal → 必须 reset: last_linear_vel_ 归零, 重新从 ramp 起步
  Path plan2 = makeXPath();
  plan2.poses.back() = makePose(12.0, 0.0);
  ASSERT_TRUE(ctl->setPlan(plan2));
  const TwistStamped c3 = compute(ctl, robot, outcome, message);
  ASSERT_EQ(outcome, ayanami::actions::ExePathResult::SUCCESS);

  EXPECT_NEAR(c3.twist.linear.x(), kMaxAcc * kDt, 1e-9);
  EXPECT_LT(c3.twist.linear.x(), c2.twist.linear.x());
}

// ---------- isGoalReached + 零速度 ----------

TEST(CostmapPidControllerTest, GoalReachedStopsWithZeroVelocity) {
  auto ctl = makeController();
  ASSERT_TRUE(ctl->setPlan(makeXPath()));

  // 直接放 robot 到接近 goal 处 (dist < goal_tolerance=0.1)。
  const PoseStamped near_goal = makePose(8.95, 0.0);
  uint32_t outcome = 0;
  std::string message;
  const TwistStamped cmd = compute(ctl, near_goal, outcome, message);

  EXPECT_EQ(outcome, ayanami::actions::ExePathResult::SUCCESS);
  EXPECT_NEAR(cmd.twist.linear.x(), 0.0, 1e-9);
  EXPECT_NEAR(cmd.twist.linear.y(), 0.0, 1e-9);
  EXPECT_NEAR(cmd.twist.linear.z(), 0.0, 1e-9);
  EXPECT_NEAR(cmd.twist.angular.z(), 0.0, 1e-9);
  EXPECT_TRUE(ctl->isGoalReached(0.2, 0.2));
}

// ---------- 多帧模拟收敛到 goal 停止 ----------

TEST(CostmapPidControllerTest, MultiFrameReachesGoalAndStops) {
  auto ctl = makeController();
  ASSERT_TRUE(ctl->setPlan(makeXPath()));

  PoseStamped robot = makePose(0.0, 0.0);
  uint32_t outcome = 0;
  std::string message;
  double prev_x = 0.0;
  bool stopped = false;
  for (int i = 0; i < 2000 && !stopped; ++i) {
    const TwistStamped cmd = compute(ctl, robot, outcome, message);
    ASSERT_EQ(outcome, ayanami::actions::ExePathResult::SUCCESS) << message;
    if (cmd.twist.linear.x() < 1e-6) {
      stopped = true;
      break;
    }
    // 手动 simulate: 沿车体 x 前进。
    robot.pose.translation().x() += cmd.twist.linear.x() * kDt;
    EXPECT_GT(robot.pose.translation().x(), prev_x);
    prev_x = robot.pose.translation().x();
  }

  EXPECT_TRUE(stopped);
  EXPECT_GT(robot.pose.translation().x(), 8.0);  // 已贴近 goal
  EXPECT_TRUE(ctl->isGoalReached(0.2, 0.2));
}

// ---------- cancel ----------

TEST(CostmapPidControllerTest, CancelReturnsCanceledOutcomeAndZero) {
  auto ctl = makeController();
  ASSERT_TRUE(ctl->setPlan(makeXPath()));

  const PoseStamped robot = makePose(0.0, 0.0);
  uint32_t outcome = 0;
  std::string message;
  const TwistStamped c1 = compute(ctl, robot, outcome, message);
  ASSERT_EQ(outcome, ayanami::actions::ExePathResult::SUCCESS);

  ASSERT_TRUE(ctl->cancel());
  const TwistStamped cmd = compute(ctl, robot, outcome, message);
  EXPECT_GE(outcome, 100u);  // controller 错误段 100+
  EXPECT_NEAR(cmd.twist.linear.x(), 0.0, 1e-9);
  EXPECT_NEAR(cmd.twist.linear.y(), 0.0, 1e-9);
  EXPECT_NEAR(cmd.twist.linear.z(), 0.0, 1e-9);
  EXPECT_NEAR(cmd.twist.angular.z(), 0.0, 1e-9);
}

// ---------- Twist 清零 (脏内存防护) ----------

TEST(CostmapPidControllerTest, TwistClearedNoDirtyMemory) {
  auto ctl = makeController();
  ASSERT_TRUE(ctl->setPlan(makeXPath()));

  const PoseStamped robot = makePose(0.0, 0.0);
  TwistStamped cmd;
  // 污染 cmd_vel (历史脏内存 bug 场景)。
  cmd.twist.linear = Eigen::Vector3d(std::nan(""), -1e300, 42.0);
  cmd.twist.angular = Eigen::Vector3d(
      std::numeric_limits<double>::infinity(), 7.5, 3.14);
  cmd.header.frame_id = "garbage";

  uint32_t outcome = 0;
  std::string message;
  const uint32_t result =
      ctl->computeVelocityCommands(robot, TwistStamped{}, cmd, message);

  EXPECT_EQ(result, ayanami::actions::ExePathResult::SUCCESS);
  EXPECT_TRUE(std::isfinite(cmd.twist.linear.x()));
  EXPECT_NEAR(cmd.twist.linear.z(), 0.0, 1e-12);
  EXPECT_NEAR(cmd.twist.angular.x(), 0.0, 1e-12);
  EXPECT_NEAR(cmd.twist.angular.y(), 0.0, 1e-12);
  EXPECT_TRUE(std::isfinite(cmd.twist.angular.z()));
  EXPECT_EQ(cmd.header.frame_id, robot.header.frame_id);
}

// ---------- 全向模式使用横向轴 ----------

TEST(CostmapPidControllerTest, OmniModeUsesLateralAxis) {
  writeConfig("omni");
  auto ctl = makeController();
  writeConfig("diff");  // 恢复, 不影响后续测试

  Path p;
  p.header.frame_id = "map";
  for (int i = 0; i < 10; ++i) {
    p.poses.push_back(makePose(0.0, static_cast<double>(i)));  // 沿 +Y
  }
  ASSERT_TRUE(ctl->setPlan(p));

  uint32_t outcome = 0;
  std::string message;
  const TwistStamped cmd = compute(ctl, makePose(0.0, 0.0), outcome, message);

  EXPECT_EQ(outcome, ayanami::actions::ExePathResult::SUCCESS);
  // 全向: 横向轴 (linear.y) 承担前进, 首帧被 max_acc*dt ramp 限住。
  EXPECT_NEAR(cmd.twist.linear.y(), kMaxAcc * kDt, 1e-9);
  EXPECT_NEAR(cmd.twist.linear.x(), 0.0, 1e-9);
}

// ---------- module.cpp 工厂符号 ----------

TEST(CostmapPidControllerTest, ModuleFactoryCreatesDerivedController) {
  ASSERT_FALSE(g_module_so.empty());
  void* handle = ayanami::open_so(g_module_so);
  ASSERT_NE(handle, nullptr);
  using Factory = ayanami::abstract::AbstractController* (*)();
  void* symbol = ayanami::load_symbol(handle, "create_controller_module");
  ASSERT_NE(symbol, nullptr);
  auto factory = reinterpret_cast<Factory>(symbol);
  ayanami::abstract::AbstractController* controller = factory();
  ASSERT_NE(controller, nullptr);
  // 返回真实派生实例。
  EXPECT_NE(dynamic_cast<CostmapPidController*>(controller), nullptr);
  delete controller;
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  if (argc > 1) {
    g_module_so = argv[1];
  }

  // 临时 config 目录: 控制模块参数 (与测试常量一致)。
  g_config_dir =
      std::filesystem::temp_directory_path() / "ayanami_costmap_pid_test";
  std::filesystem::remove_all(g_config_dir);
  std::filesystem::create_directories(g_config_dir);
  {
    std::ofstream ofs(g_config_dir / "config.json");
    ofs << R"({"global": { "costmap_pid_controller": "costmap_pid_controller.json" }})";
  }
  writeConfig("diff");
  ayanami::GlobalConfig::instance(g_config_dir.string(), true);

  return RUN_ALL_TESTS();
}
