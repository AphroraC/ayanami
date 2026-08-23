// Task 3 (M2 阶段 A 收尾): minor carry wiring 综合 5 项。
//
// 覆盖 (brief Step 1 verbatim):
// - SmacPlannerFallsBackToRobotPose: makePlan(start 留空) → costmap_context
//   getRobotPose fallback → path 首位 pose ≈ (1, 2, pi/4) (mock transform)。
// - RobotFrameSingleSourceFromCostmapNavigationJson: robot_frame 单一来源改为
//   costmap_navigation.json::navigation::robot_frame (costmap.json 不再读)。
// - ConfigureCoreMapsTwiceWarns: 二次 configureCoreMaps spdlog warn + 不替换
//   costmap_context_ (同一 ptr)。
// - PidSignatureCoarseSameGoalNoReset: 同 goal 同 size 仅中间点稍异的 plan,
//   粗签名 (goal+size) 相同 → setPlan 不重置 PID 状态 (ramp 连续)。

#include <gtest/gtest.h>

#include <atomic>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <sstream>
#include <string>

#include <Eigen/Geometry>

#include <costmap_2d/costmap_2d.hpp>
#include <costmap_2d/layered_costmap.hpp>
#include <costmap_2d_layers/static_layer.hpp>
#include <costmap_server/costmap_wrapper.hpp>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/ostream_sink.h>

#include <ayanami/costmap/components/costmap_context.hpp>
#include <ayanami/costmap/components/costmap_navigation_server.hpp>
#include <ayanami/interfaces/transform_store.hpp>
#include <ayanami/messages/failure.hpp>
#include <ayanami/messages/geometry_msgs.hpp>
#include <ayanami/messages/nav_msgs.hpp>
#include <costmap_pid_controller/costmap_pid_controller.hpp>
#include <costmap_smac_planner/costmap_smac_planner.hpp>
#include <ayanami/utility/config.hpp>

namespace {

using ayanami::FailureCode;
using ayanami::costmap::CostmapContext;
using ayanami::costmap::CostmapNavigationServer;
using ayanami::geometry_msgs::PoseStamped;
using ayanami::geometry_msgs::TwistStamped;
using ayanami::modules::CostmapPidController;
using ayanami::modules::CostmapSmacPlanner;
using ayanami::nav_msgs::Path;

constexpr double kMaxAcc = 1.0;
constexpr double kDt = 0.05;
constexpr double kPi = 3.14159265358979323846;

std::filesystem::path g_module_dir;  // pid/smac 模块配置目录

// ---------------- 通用工具 ----------------

void write_file(const std::filesystem::path& path,
                const std::string& content) {
  std::ofstream ofs(path);
  ofs << content;
}

std::filesystem::path make_unique_dir(const std::string& tag) {
  auto dir = std::filesystem::temp_directory_path() /
             ("ayanami_minor_carry_" + tag);
  std::filesystem::remove_all(dir);
  std::filesystem::create_directories(dir);
  return dir;
}

// 让 GlobalConfig 指向模块配置目录 (pid/smac loadConfig 用)。
void resetModuleConfig() {
  ayanami::GlobalConfig::instance(g_module_dir.string(), true);
}

PoseStamped makePose(double x, double y, double yaw = 0.0) {
  auto p = PoseStamped::fromIsometry("map", Eigen::Isometry3d::Identity());
  p.pose.translation() = Eigen::Vector3d(x, y, 0.0);
  p.pose.linear() =
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return p;
}

// ---------------- Test 1: smac fallback ----------------

// NxN @ res, origin=(0,0), 全 free 的 LayeredCostmap + StaticLayer。
std::shared_ptr<costmap_2d::CostmapWrapper> makeWrapper(unsigned int size,
                                                        double res) {
  auto lm = std::make_shared<costmap_2d::LayeredCostmap>(
      "map", /*rolling_window=*/false, /*track_unknown=*/false);
  lm->resizeMap(size, size, res, 0.0, 0.0);
  costmap_2d::Costmap2D layer_map(size, size, res, 0.0, 0.0,
                                  costmap_2d::FREE_SPACE);
  auto static_layer = std::make_shared<costmap_2d_layers::StaticLayer>();
  static_layer->setMap(layer_map);
  lm->addLayer(static_layer);
  lm->updateMap(0.0, 0.0, 0.0);
  return std::make_shared<costmap_2d::CostmapWrapper>(lm, "map");
}

PoseStamped cellCenter(unsigned int mx, unsigned int my, double res = 0.5) {
  return makePose((static_cast<double>(mx) + 0.5) * res,
                  (static_cast<double>(my) + 0.5) * res);
}

// ---------------- Test 2/3: configureCoreMaps ----------------

// 临时配置目录: costmap_navigation.json 含 robot_frame + costmap 段 (原
// costmap.json 参数已并入 navigation 文件)。static_map 留空 (不需要真实地图文件)。
std::filesystem::path make_navigation_config_dir(
    const std::string& robot_frame) {
  auto dir = make_unique_dir("nav_cfg_" + robot_frame);
  write_file(dir / "config.json",
             "{\n  \"global\": {\n"
             "    \"config_navigation\": \"costmap_navigation.json\"\n"
             "  }\n}\n");
  write_file(dir / "costmap_navigation.json",
             "{\n  \"navigation\": {\n"
             "    \"robot_frame\": \"" + robot_frame + "\"\n"
             "  },\n"
             "  \"costmap\": {\n"
             "    \"global_frame\": \"map\",\n"
             "    \"local_frame\": \"odom\",\n"
             "    \"resolution\": 0.5\n"
             "  },\n"
             "  \"costmap_global\": {\n"
             "    \"size_x\": 20,    \"size_y\": 20,\n"
             "    \"rolling_window\": false,\n"
             "    \"track_unknown\": false,\n"
             "    \"static_map\": \"\",\n"
             "    \"inflation_radius\": 0.6,\n"
             "    \"inscribed_radius\": 0.3,\n"
             "    \"cost_scaling_factor\": 3.0\n"
             "  },\n"
             "  \"costmap_local\": {\n"
             "    \"size_x\": 20,    \"size_y\": 20,\n"
             "    \"rolling_window\": true,\n"
             "    \"track_unknown\": false,\n"
             "    \"inflation_radius\": 0.6,\n"
             "    \"inscribed_radius\": 0.3,\n"
             "    \"cost_scaling_factor\": 3.0\n"
             "  }\n"
             "}\n");
  return dir;
}

// ---------------- Test 4: pid 粗签名 ----------------

std::string pidConfigContent() {
  return "{\n  \"costmap_pid_controller\": {\n"
         "    \"max_v\": 2.0,\n    \"max_w\": 1.0,\n"
         "    \"max_acc\": 1.0,\n    \"max_decel\": 1.0,\n"
         "    \"max_angular_accel\": 1.0,\n"
         "    \"kp_v\": 1.0,\n    \"ki_v\": 0.0,\n    \"kd_v\": 0.0,\n"
         "    \"kp_w\": 1.0,\n    \"ki_w\": 0.0,\n    \"kd_w\": 0.0,\n"
         "    \"dist_decay\": 2.0,\n    \"goal_tolerance\": 0.1,\n"
         "    \"slide_window\": 1.0,\n    \"default_dt\": 0.05,\n"
         "    \"heading_deadband\": 0.08,\n"
         "    \"rotate_in_place_angle\": 0.45,\n"
         "    \"mode\": \"diff\"\n"
         "  }\n}\n";
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
                     const PoseStamped& robot, FailureCode& failure,
                     std::string& message) {
  TwistStamped cmd;
  // goal_checker 传空: 触发 PID 空指针防御 (回退 config goal_tolerance), 控制器
  // 只负责生成速度, 到达判定由 execution 层的 goal_checker 负责。
  failure = ctl->computeVelocityCommands(robot, TwistStamped{}, cmd, message,
                                         nullptr);
  return cmd;
}

}  // namespace

// ---------------- Test 1 ----------------

TEST(MinorCarryWiring, SmacPlannerFallsBackToRobotPose) {
  resetModuleConfig();

  auto wrapper = makeWrapper(20, 0.5);  // 20x20 @ 0.5 = 10m x 10m, 全 free
  auto ctx = std::make_shared<CostmapContext>();
  ctx->global_costmap_ = wrapper;
  ctx->global_frame_ = "map";
  ctx->robot_frame_ = "base_link";
  ctx->transform_store_ = &ayanami::interfaces::TransformStore::instance();

  // mock robot pose (1, 2, pi/4) in map frame。
  auto& store = ayanami::interfaces::TransformStore::instance();
  Eigen::Isometry3d T_robot = Eigen::Isometry3d::Identity();
  T_robot.translation() = Eigen::Vector3d(1.0, 2.0, 0.0);
  T_robot.linear() =
      Eigen::AngleAxisd(kPi / 4.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  store.setTransform("map", "base_link", ayanami::Timestamp::latest(),
                     T_robot);

  auto planner = std::make_shared<CostmapSmacPlanner>();
  planner->initialize("test_smac", ctx);

  // start 留空 (默认构造: translation 全零 + frame_id 空)。
  const PoseStamped start;
  const PoseStamped goal = cellCenter(17, 17);
  Path plan;
  double cost = 0.0;
  std::string message;
  const FailureCode failure =
      planner->makePlan(start, goal, 0.1, plan, cost, message);

  EXPECT_EQ(failure, FailureCode::None) << message;
  ASSERT_FALSE(plan.poses.empty());
  // 首位 pose == fallback 的 robot pose (1, 2, pi/4)。
  EXPECT_NEAR(plan.poses.front().pose.translation().x(), 1.0, 1e-6);
  EXPECT_NEAR(plan.poses.front().pose.translation().y(), 2.0, 1e-6);
  const double yaw = std::atan2(plan.poses.front().pose.linear()(1, 0),
                                plan.poses.front().pose.linear()(0, 0));
  EXPECT_NEAR(yaw, kPi / 4.0, 1e-6);
}

// ---------------- Test 2 ----------------

TEST(MinorCarryWiring, RobotFrameSingleSourceFromCostmapNavigationJson) {
  const auto dir = make_navigation_config_dir("base_footprint");
  CostmapNavigationServer server;
  server.configureCoreMaps(dir.string());

  ASSERT_NE(server.costmapContext(), nullptr);
  // costmap.json 无 robot_frame → 单一来源 = costmap_navigation.json。
  EXPECT_EQ(server.costmapContext()->getRobotFrame(), "base_footprint");
  // RobotStateProvider 也用同一 robot_frame。
  EXPECT_EQ(server.robotStateProvider().getRobotFrame(), "base_footprint");
}

// ---------------- Test 3 ----------------

TEST(MinorCarryWiring, ConfigureCoreMapsTwiceWarns) {
  const auto dir = make_navigation_config_dir("base_footprint");
  CostmapNavigationServer server;
  server.configureCoreMaps(dir.string());
  const auto ctx1 = server.costmapContext();
  ASSERT_NE(ctx1, nullptr);

  // 挂一个 ostream sink 捕获 spdlog 输出 (默认 logger)。
  std::ostringstream oss;
  auto sink = std::make_shared<spdlog::sinks::ostream_sink_mt>(oss);
  spdlog::default_logger()->sinks().push_back(sink);

  server.configureCoreMaps(dir.string());

  spdlog::default_logger()->sinks().pop_back();

  // 二次 configure 不替换 context (同一 ptr)。
  EXPECT_EQ(server.costmapContext(), ctx1);
  // 触发 spdlog warn。
  EXPECT_NE(oss.str().find("called twice"), std::string::npos);
}

// ---------------- Test 4 ----------------

TEST(MinorCarryWiring, PidSignatureCoarseSameGoalNoReset) {
  resetModuleConfig();
  auto ctl = makeController();

  const Path plan1 = makeXPath(9.0);
  ASSERT_TRUE(ctl->setPlan(plan1));

  const PoseStamped robot = makePose(0.5, 0.0);
  FailureCode failure = FailureCode::None;
  std::string message;
  const TwistStamped c1 = compute(ctl, robot, failure, message);
  ASSERT_EQ(failure, FailureCode::None);
  const TwistStamped c2 = compute(ctl, robot, failure, message);
  ASSERT_EQ(failure, FailureCode::None);
  ASSERT_GT(c2.twist.linear.x(), c1.twist.linear.x());

  // 同 goal (9,0) 同 size (10), 仅中间点稍移 y → 粗签名 (goal+size) 相同。
  Path plan2 = plan1;
  for (int i = 1; i + 1 < static_cast<int>(plan2.poses.size()); ++i) {
    plan2.poses[i].pose.translation().y() += 0.005;
  }
  ASSERT_TRUE(ctl->setPlan(plan2));

  const TwistStamped c3 = compute(ctl, robot, failure, message);
  ASSERT_EQ(failure, FailureCode::None);

  // 不重置: ramp 连续 (c3 ≈ 3*kMaxAcc*kDt), 而不是掉回 kMaxAcc*kDt。
  EXPECT_GT(c3.twist.linear.x(), c2.twist.linear.x());
  EXPECT_NEAR(c3.twist.linear.x(), 3.0 * kMaxAcc * kDt, 1e-9);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);

  // 模块配置目录 (pid/smac loadConfig)。
  g_module_dir = make_unique_dir("modules");
  write_file(g_module_dir / "config.json",
             "{\n  \"global\": {\n"
             "    \"config_navigation\": \"navigation.json\"\n"
             "  }\n}\n");
  write_file(g_module_dir / "navigation.json",
             "{\n  \"modules\": {\n"
             "    \"config_controller\": \"costmap_pid_controller.json\",\n"
             "    \"config_planner\": \"costmap_smac_planner.json\"\n"
             "  }\n}\n");
  write_file(g_module_dir / "costmap_pid_controller.json", pidConfigContent());
  write_file(g_module_dir / "costmap_smac_planner.json",
             "{\n  \"costmap_smac_planner\": {\n"
             "    \"use_diagonal\": true,    \"tolerance\": 0.0,\n"
             "    \"cost_factor\": 0.5,    \"lethal_cost\": 254,\n"
             "    \"neutral_cost\": 1.0,    \"use_final_approach\": false\n"
             "  }\n}\n");
  resetModuleConfig();

  return RUN_ALL_TESTS();
}
