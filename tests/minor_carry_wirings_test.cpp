// (M2 phase A wrap-up): 5 wiring integration items.
//
// Coverage (brief Step 1 verbatim):
// - SmacPlannerFallsBackToRobotPose: makePlan(start left empty) -> costmap_context
// getRobotPose fallback -> the first path pose ~ (1, 2, pi/4) (mock transform).
// - RobotFrameSingleSourceFromCostmapNavigationJson: the robot_frame single source moves to
// costmap_navigation.json::navigation::robot_frame (costmap.json is no longer read).
// - ConfigureCoreMapsTwiceWarns: a second configureCoreMaps logs an spdlog warn + keeps the
// costmap_context (same ptr).
// - PidSignatureCoarseSameGoalNoReset: plans with the same goal and size but slightly different interior points share
// the same coarse signature (goal+size) -> setPlan does not reset the PID state (continuous ramp).

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

#include <ayanami/abstract/components/robot_state_controller.hpp>
#include <ayanami/costmap/components/costmap_map_context.hpp>
#include <ayanami/costmap/components/costmap_navigation_server.hpp>
#include <ayanami/messages/failure.hpp>
#include <ayanami/types.hpp>
#include <ayanami/messages/geometry_msgs.hpp>
#include <ayanami/messages/nav_msgs.hpp>
#include <costmap_pid_controller/costmap_pid_controller.hpp>
#include <costmap_smac_planner/costmap_smac_planner.hpp>
#include <ayanami/utility/config.hpp>

namespace {

using ayanami::ErrorCode;
using ayanami::ResultT;
using ayanami::costmap::CostmapMapContext;
using ayanami::costmap::CostmapNavigationServer;
using ayanami::geometry_msgs::PoseStamped;
using ayanami::geometry_msgs::TwistStamped;
using ayanami::modules::CostmapPidController;
using ayanami::modules::CostmapSmacPlanner;
using ayanami::nav_msgs::Path;

constexpr double k_max_acc = 1.0;
constexpr double k_dt = 0.05;
constexpr double k_pi = 3.14159265358979323846;

std::filesystem::path g_module_dir;  // pid/smac module config directory

// ---------------- Common utilities ----------------

void write_file(const std::filesystem::path& path,
                const std::string& content) {
  std::ofstream ofs(path);
  ofs << content;
}

std::filesystem::path makeUniqueDir(const std::string& tag) {
  auto dir = std::filesystem::temp_directory_path() /
             ("ayanami_minor_carry" + tag);
  std::filesystem::remove_all(dir);
  std::filesystem::create_directories(dir);
  return dir;
}

/// Points GlobalConfig at the module config directory (used by pid/smac loadConfig).
void resetModuleConfig() {
  ayanami::GlobalConfig::instance(g_module_dir.string(), true);
}

PoseStamped makePose(double x, double y, double yaw = 0.0) {
  auto p = PoseStamped::fromIsometry("map", Eigen::Isometry3d::Identity());
  p.pose.position = Eigen::Vector3d(x, y, 0.0);
  p.pose.orientation =
      Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
  return p;
}

// ---------------- Test 1: smac fallback ----------------

// An NxN @ res, origin=(0,0), all-free LayeredCostmap + StaticLayer.
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

// Temp config dir: costmap_navigation.json contains robot_frame + the costmap section (the old
// costmap.json parameters were merged into the navigation file). static_map left empty (no real map file needed).
std::filesystem::path makeNavigationConfigDir(
    const std::string& robot_frame) {
  auto dir = makeUniqueDir("nav_cfg" + robot_frame);
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

// ---------------- Test 4: pid coarse signature ----------------

std::string pidConfigContent() {
  return "{\n  \"costmap_pid_controller\": {\n"
         "    \"max_vel_x\": 2.0,\n    \"max_vel_theta\": 1.0,\n"
         "    \"max_acc_x\": 1.0,\n    \"max_decel_x\": 1.0,\n"
         "    \"max_acc_theta\": 1.0,\n"
         "    \"kp_v\": 1.0,\n    \"ki_v\": 0.0,\n    \"kd_v\": 0.0,\n"
         "    \"kp_w\": 1.0,\n    \"ki_w\": 0.0,\n    \"kd_w\": 0.0,\n"
         "    \"dist_decay\": 2.0,\n    \"goal_tolerance\": 0.1,\n"
         "    \"slide_window\": 1.0,\n    \"default_dt\": 0.05,\n"
         "    \"heading_deadband\": 0.08,\n"
         "    \"rotate_in_place_angle\": 0.45,\n"
         "    \"mode\": \"diff\"\n"
         "  }\n}\n";
}

/// Ten poses along the X axis, from (0,0) to (goal_x, 0), facing +X.
Path makeXPath(double goal_x = 9.0) {
  Path p;
  p.header.frame_id = "map";
  for (int i = 0; i < 10; ++i) {
    p.poses.push_back(makePose(static_cast<double>(i) / 9.0 * goal_x, 0.0));
  }
  return p;
}

std::shared_ptr<CostmapPidController> makeController() {
  auto ctl = std::make_shared<CostmapPidController>();
  // New API: parameters are injected via PluginContext.config (this module json).
  ctl->configure(ayanami::PluginContext{
                     "test_pid",
                     ayanami::Config((g_module_dir /
                                      "costmap_pid_controller.json").string())},
                 std::make_shared<CostmapMapContext>());
  return ctl;
}

TwistStamped compute(const std::shared_ptr<CostmapPidController>& ctl,
                     const PoseStamped& robot, ErrorCode& error,
                     std::string& message) {
  TwistStamped cmd;
  const auto result = ctl->computeVelocityCommand(robot, TwistStamped{}, cmd);
  error = result.error;
  message = result.message;
  return result.ok ? cmd : TwistStamped{};
}

}  // namespace

// ---------------- Test 1 ----------------

TEST(MinorCarryWiring, smacPlannerFallsBackToRobotPose) {
  resetModuleConfig();

  auto wrapper = makeWrapper(20, 0.5);  // 20x20 @ 0.5 = 10m x 10m, all free
  auto ctx = std::make_shared<CostmapMapContext>();
  ctx->setGlobalCostmap(wrapper);
  ctx->setLocalCostmap(wrapper);
  ctx->setFrames("map", "map", "base_link");

  // Mock robot pose (1, 2, pi/4) in the map frame.
  auto& rsc = ayanami::components::RobotStateController::instance();
  Eigen::Isometry3d T_robot = Eigen::Isometry3d::Identity();
  T_robot.translation() = Eigen::Vector3d(1.0, 2.0, 0.0);
  T_robot.linear() =
      Eigen::AngleAxisd(k_pi / 4.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  rsc.setTransform("map", "base_link", ayanami::Timestamp::latest(),
                   T_robot);

  auto planner = std::make_shared<CostmapSmacPlanner>();
  planner->configure(ayanami::PluginContext{"test_smac", ayanami::Config("")},
                     ctx->getSharedConstPtr());

  // start left empty (default constructed: all-zero translation + empty frame_id).
  const PoseStamped start;
  const PoseStamped goal = cellCenter(17, 17);
  Path plan;
  const ResultT<void> result = planner->makePlan(start, goal, 0.1, plan);

  ASSERT_TRUE(result.ok) << result.message;
  ASSERT_FALSE(plan.poses.empty());
  // The first pose == the fallback robot pose (1, 2, pi/4).
  EXPECT_NEAR(plan.poses.front().pose.position.x(), 1.0, 1e-6);
  EXPECT_NEAR(plan.poses.front().pose.position.y(), 2.0, 1e-6);
  const auto& q = plan.poses.front().pose.orientation;
  const double yaw = std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                                1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
  EXPECT_NEAR(yaw, k_pi / 4.0, 1e-6);
}

// ---------------- Test 2 ----------------

TEST(MinorCarryWiring, robotFrameSingleSourceFromCostmapNavigationJson) {
  const auto dir = makeNavigationConfigDir("base_footprint");
  CostmapNavigationServer server;
  server.configureCoreMaps(dir.string());

  ASSERT_NE(server.costmapContext(), nullptr);
  // costmap.json has no robot_frame -> single source = costmap_navigation.json.
  EXPECT_EQ(server.costmapContext()->getRobotFrame(), "base_footprint");
  // RobotStateController is the single configured robot-state source.
  EXPECT_EQ(ayanami::components::RobotStateController::instance().getRobotFrame(),
            "base_footprint");
}

// ---------------- Test 3 ----------------

TEST(MinorCarryWiring, configureCoreMapsTwiceWarns) {
  const auto dir = makeNavigationConfigDir("base_footprint");
  CostmapNavigationServer server;
  server.configureCoreMaps(dir.string());
  const auto ctx1 = server.costmapContext();
  ASSERT_NE(ctx1, nullptr);

  // Attach an ostream sink to capture spdlog output (default logger).
  std::ostringstream oss;
  auto sink = std::make_shared<spdlog::sinks::ostream_sink_mt>(oss);
  spdlog::default_logger()->sinks().push_back(sink);

  server.configureCoreMaps(dir.string());

  spdlog::default_logger()->sinks().pop_back();

  // A second configure does not replace the context (same ptr).
  EXPECT_EQ(server.costmapContext(), ctx1);
  // Triggers the spdlog warn.
  EXPECT_NE(oss.str().find("called twice"), std::string::npos);
}

// ---------------- Test 4 ----------------

TEST(MinorCarryWiring, pidSignatureCoarseSameGoalNoReset) {
  resetModuleConfig();
  auto ctl = makeController();

  const Path plan1 = makeXPath(9.0);
  ASSERT_TRUE(ctl->setPlan(plan1));

  const PoseStamped robot = makePose(0.5, 0.0);
  ErrorCode failure = ErrorCode::NONE;
  std::string message;
  const TwistStamped c1 = compute(ctl, robot, failure, message);
  ASSERT_EQ(failure, ErrorCode::NONE);
  const TwistStamped c2 = compute(ctl, robot, failure, message);
  ASSERT_EQ(failure, ErrorCode::NONE);
  ASSERT_GT(c2.twist.linear.x(), c1.twist.linear.x());

  // Same goal (9,0) and size (10), only interior points shifted slightly in y -> the coarse signature (goal+size) matches.
  Path plan2 = plan1;
  for (int i = 1; i + 1 < static_cast<int>(plan2.poses.size()); ++i) {
    plan2.poses[i].pose.position.y() += 0.005;
  }
  ASSERT_TRUE(ctl->setPlan(plan2));

  const TwistStamped c3 = compute(ctl, robot, failure, message);
  ASSERT_EQ(failure, ErrorCode::NONE);

  // No reset: the ramp is continuous (c3 ~ 3*k_max_acc*k_dt) instead of dropping back to k_max_acc*k_dt.
  EXPECT_GT(c3.twist.linear.x(), c2.twist.linear.x());
  EXPECT_NEAR(c3.twist.linear.x(), 3.0 * k_max_acc * k_dt, 1e-9);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);

  // Module config directory (strictly single-level index; read explicitly by pid/smac constructors via getConfigPath).
  g_module_dir = makeUniqueDir("modules");
  write_file(g_module_dir / "config.json",
             "{\n  \"global\": {\n"
             "    \"config_navigation\": \"navigation.json\",\n"
             "    \"config_planner\": \"costmap_smac_planner.json\",\n"
             "    \"config_controller\": \"costmap_pid_controller.json\",\n"
             "    \"config_smac_planner\": \"costmap_smac_planner.json\",\n"
             "    \"config_pid_controller\": \"costmap_pid_controller.json\"\n"
             "  }\n}\n");
  write_file(g_module_dir / "navigation.json",
             "{\n  \"navigation\": {}\n}\n");
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
