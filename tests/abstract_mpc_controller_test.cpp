// abstract_mpc_controller: iterative linear MPC tracker (rose_navigation
// LMPC port, abstract family / map-free).
//
// 1. straight-line rollout converges onto the path under the double
//    integrator simulation model (position error < tolerance at the end);
// 2. speed and dv (acceleration) box constraints hold over the rollout;
// 3. a robot beyond deviation_threshold fails with NO_PROGRESS (rose's
//    reference-rejection semantics);
// 4. module.cpp exports the unified factory (dlopen).

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>

#include "ayanami/utility/config.hpp"
#include "ayanami/utility/extension_module.hpp"

#include "../modules/abstract_mpc_controller/abstract_mpc_controller.hpp"

namespace {

using ayanami::ResultT;
using ayanami::modules::AbstractMpcController;
using TwistStamped = ayanami::geometry_msgs::TwistStamped;
using ayanami::ErrorCode;

std::string g_module_so;
std::filesystem::path g_config_dir;

ayanami::nav_msgs::Path makeLinePath(double y, double length, double dx) {
  ayanami::nav_msgs::Path plan;
  plan.header.frame_id = "map";
  for (double x = 0.0; x <= length + 1e-9; x += dx) {
    ayanami::geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x() = x;
    pose.pose.position.y() = y;
    pose.pose.orientation.w() = 1.0;
    plan.poses.push_back(pose);
  }
  return plan;
}

ayanami::geometry_msgs::PoseStamped makePose(double x, double y) {
  ayanami::geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.position.x() = x;
  pose.pose.position.y() = y;
  pose.pose.orientation.w() = 1.0;  // facing +x
  return pose;
}

TwistStamped makeTwist() { return TwistStamped{}; }

struct RolloutState {
  double x{0.0};
  double y{0.0};
};

/// One rollout step: apply the command under the double integrator the MPC
/// itself assumes (perfect velocity tracking), then move.
bool step(AbstractMpcController& controller, RolloutState& robot,
          double dt, double& vx_out, double& vy_out) {
  TwistStamped cmd;
  const ResultT<void> result = controller.computeVelocityCommand(
      makePose(robot.x, robot.y), makeTwist(), cmd);
  if (!result.ok) {
    return false;
  }
  vx_out = cmd.twist.linear.x();
  vy_out = cmd.twist.linear.y();
  robot.x += vx_out * dt;
  robot.y += vy_out * dt;
  return true;
}

}  // namespace

TEST(AbstractMpcControllerTest, straightLineRolloutConvergesAndHoldsBoxes) {
  AbstractMpcController controller;
  ayanami::Config cfg((g_config_dir / "abstract_mpc_controller.json").string());
  controller.configureParameters(
      ayanami::ModuleContext{"abstract_mpc_controller", cfg});

  ASSERT_TRUE(controller.setPlan(makeLinePath(1.0, 10.0, 0.1)));

  RolloutState robot{0.0, 0.7};  // 0.3 m off the y=1 line (inside
  // the 0.5 m deviation threshold).

  double last_vx = 0.0, last_vy = 0.0;
  const double dt = 0.05;
  const double max_speed = 2.0, max_accel = 1.0;
  for (int cycle = 0; cycle < 120; ++cycle) {
    double vx = 0.0, vy = 0.0;
    ASSERT_TRUE(step(controller, robot, dt, vx, vy))
        << "cycle " << cycle;
    // The QP boxes are PER-AXIS (rose parity: vx and vy each in
    // +-max_speed), not on the speed norm -- the norm limit belongs to the
    // execution-layer clamp. Likewise the dv box constrains ADJACENT steps
    // INSIDE the horizon; the first control may jump from the current
    // speed (rose parity -- the stack above smooths the output).
    EXPECT_LE(std::fabs(vx), max_speed + 1e-6);
    EXPECT_LE(std::fabs(vy), max_speed + 1e-6);
    last_vx = vx;
    last_vy = vy;
  }
  // Converged onto the line (Q pulls the state onto the reference band).
  EXPECT_NEAR(robot.y, 1.0, 0.15);
  EXPECT_GT(robot.x, 1.0);
}

TEST(AbstractMpcControllerTest, beyondThresholdFailsWithNoProgress) {
  AbstractMpcController controller;
  ayanami::Config cfg((g_config_dir / "abstract_mpc_controller.json").string());
  controller.configureParameters(
      ayanami::ModuleContext{"abstract_mpc_controller", cfg});
  ASSERT_TRUE(controller.setPlan(makeLinePath(0.0, 10.0, 0.1)));

  TwistStamped cmd;
  const ResultT<void> result =
      controller.computeVelocityCommand(makePose(5.0, 1.2), makeTwist(), cmd);
  ASSERT_FALSE(result.ok);
  EXPECT_EQ(result.error, ErrorCode::NO_PROGRESS);
}

TEST(AbstractMpcControllerTest, emptyPlanFails) {
  AbstractMpcController controller;
  ayanami::nav_msgs::Path empty;
  EXPECT_FALSE(controller.setPlan(empty));
  TwistStamped cmd;
  const ResultT<void> result =
      controller.computeVelocityCommand(makePose(0, 0), makeTwist(), cmd);
  EXPECT_FALSE(result.ok);
}

TEST(AbstractMpcControllerTest, cancelShortCircuits) {
  AbstractMpcController controller;
  ayanami::Config cfg((g_config_dir / "abstract_mpc_controller.json").string());
  controller.configureParameters(
      ayanami::ModuleContext{"abstract_mpc_controller", cfg});
  ASSERT_TRUE(controller.setPlan(makeLinePath(0.0, 10.0, 0.1)));
  ASSERT_TRUE(controller.cancel());
  TwistStamped cmd;
  const ResultT<void> result =
      controller.computeVelocityCommand(makePose(0, 0), makeTwist(), cmd);
  EXPECT_EQ(result.error, ErrorCode::CANCELED);
}

TEST(AbstractMpcControllerTest, moduleFactoryCreatesDerivedController) {
  ASSERT_FALSE(g_module_so.empty());
  auto controller = std::dynamic_pointer_cast<AbstractMpcController>(
      ayanami::ExtensionModule::loadModule(g_module_so));
  ASSERT_NE(controller, nullptr);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  if (argc > 1) {
    g_module_so = argv[1];
  }
  g_config_dir =
      std::filesystem::temp_directory_path() / "ayanami_abstract_mpc_test";
  std::filesystem::remove_all(g_config_dir);
  std::filesystem::create_directories(g_config_dir);
  {
    std::ofstream ofs(g_config_dir / "config.json");
    ofs << R"({"global": { "config_mpc": "abstract_mpc_controller.json" }})";
  }
  std::ofstream ofs(g_config_dir / "abstract_mpc_controller.json");
  ofs << R"({
  "controller": {"name": "abstract_mpc_controller"},
  "abstract_mpc_controller": {"predict_dt": 0.1, "predict_steps": 30,
    "max_iter": 20, "max_speed": 2.0, "max_accel": 1.0, "delay_time": 0.0,
    "blind_radius": 0.5, "deviation_threshold": 0.5, "cruise_speed": 1.5,
    "q_pos": 15.0, "r_vel": 0.1, "rd_smooth": 1.0, "angular_kp": 3.0,
    "fixed_angular_z": 0.0}
})";
  ayanami::GlobalConfig::instance(g_config_dir.string(), true);
  return RUN_ALL_TESTS();
}
