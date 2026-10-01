// rosemap_mpc_controller tests: direct-constructed module over a synthetic
// stamped plan -- straight-line tracking, speed/accel limits, plan hot
// switch, body-frame rotation, deviation gate, and the tail freeze.

#include <gtest/gtest.h>

#include <rosemap_mpc_controller/rosemap_mpc_controller.hpp>

#include <ayanami/utility/config.hpp>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>

namespace {

using ayanami::ErrorCode;
using ayanami::nav_msgs::Path;
using ayanami::rosemap::RosemapMapContext;
using Controller = ayanami::modules::RosemapMpcController;

/// @brief A straight stamped plan along +x (relative stamps at 1 m/s).
Path straightPlan(double length, int segments, double y = 0.0) {
  Path plan;
  plan.header.frame_id = "map";
  for (int i = 0; i <= segments; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(segments);
    ayanami::geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position = Eigen::Vector3d(t * length, y * t, 0.0);
    pose.header.stamp = ayanami::Timestamp(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(t * length)));
    plan.poses.push_back(std::move(pose));
  }
  return plan;
}

class RosemapMpcControllerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    controller = std::make_shared<Controller>();
    context = std::make_shared<RosemapMapContext>();
    const ayanami::Config empty_config;
    controller->configure(
        ayanami::ModuleContext{"rosemap_mpc_controller", empty_config},
        context);
  }

  std::shared_ptr<Controller> controller;
  RosemapMapContext::Ptr context;
};

TEST_F(RosemapMpcControllerTest, TracksStraightPlan) {
  const Path plan = straightPlan(6.0, 60);
  ASSERT_TRUE(controller->setPlan(plan));

  // The robot starts 0.2 m behind the first sample, facing +x.
  ayanami::geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.position = Eigen::Vector3d(-0.2, 0.0, 0.0);
  pose.pose.orientation =
      Eigen::Quaterniond(Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ()));
  ayanami::geometry_msgs::TwistStamped velocity;

  // A few control cycles of tracking from the near start.
  double x = -0.2;
  for (int cycle = 0; cycle < 5; ++cycle) {
    pose.pose.position.x() = x;
    ayanami::geometry_msgs::TwistStamped cmd;
    auto result = controller->computeVelocityCommand(pose, velocity, cmd);
    ASSERT_TRUE(result.ok) << result.message;
    // Commands respect the speed box and push forward (toward the plan).
    EXPECT_GT(cmd.twist.linear.x(), 0.0);
    EXPECT_LE(cmd.twist.linear.x(), 2.5 + 1e-6);
    EXPECT_LE(std::abs(cmd.twist.linear.y()), 2.5 + 1e-6);
    x += cmd.twist.linear.x() * 0.05;
  }
}

TEST_F(RosemapMpcControllerTest, OutputIsRotatedIntoBodyFrame) {
  const Path plan = straightPlan(6.0, 60);
  ASSERT_TRUE(controller->setPlan(plan));

  // Robot at the plan start facing +y (yaw = pi/2): a world +x velocity
  // command must appear as BODY +y.
  ayanami::geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.position = Eigen::Vector3d(0.0, 0.0, 0.0);
  pose.pose.orientation =
      Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 2.0,
                                           Eigen::Vector3d::UnitZ()));
  ayanami::geometry_msgs::TwistStamped velocity;
  ayanami::geometry_msgs::TwistStamped cmd;
  auto result = controller->computeVelocityCommand(pose, velocity, cmd);
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_GT(cmd.twist.linear.x(), 0.0);   // forward toward the plan
  EXPECT_GT(std::abs(cmd.twist.linear.y()), 1e-3);
  EXPECT_LT(cmd.twist.linear.y(), 0.0);
  // Heading mode: the angular command rotates toward the velocity heading.
  EXPECT_LT(cmd.twist.angular.z(), 0.0);
}

TEST_F(RosemapMpcControllerTest, DeviationGateRefuses) {
  const Path plan = straightPlan(6.0, 60);
  ASSERT_TRUE(controller->setPlan(plan));

  ayanami::geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.position = Eigen::Vector3d(0.0, 4.0, 0.0);  // far off the plan
  ayanami::geometry_msgs::TwistStamped velocity;
  ayanami::geometry_msgs::TwistStamped cmd;
  auto result = controller->computeVelocityCommand(pose, velocity, cmd);
  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.error, ErrorCode::NO_VALID_COMMAND);
}

TEST_F(RosemapMpcControllerTest, PlanHotSwitchKeepsCommandsBounded) {
  ASSERT_TRUE(controller->setPlan(straightPlan(6.0, 60)));
  ayanami::geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.position = Eigen::Vector3d(0.0, 0.0, 0.0);
  ayanami::geometry_msgs::TwistStamped velocity;

  ayanami::geometry_msgs::TwistStamped first_cmd;
  auto result = controller->computeVelocityCommand(pose, velocity, first_cmd);
  ASSERT_TRUE(result.ok) << result.message;

  // Switch to a lateral plan at the same state. The acceleration box
  // bounds differences WITHIN a horizon; across plans the guarantee is
  // that the switch succeeds and both outputs respect the speed box (the
  // smoothness across plans comes from the QP warm start + the smoother's
  // near-end freeze upstream).
  ASSERT_TRUE(controller->setPlan(straightPlan(6.0, 60, 3.0)));
  ayanami::geometry_msgs::TwistStamped second_cmd;
  result = controller->computeVelocityCommand(pose, velocity, second_cmd);
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_LE(first_cmd.twist.linear.head<2>().norm(), 2.5 + 1e-6);
  EXPECT_LE(second_cmd.twist.linear.head<2>().norm(), 2.5 + 1e-6);
}

TEST_F(RosemapMpcControllerTest, NearTailCommandsTowardFreezePoint) {
  const Path plan = straightPlan(4.0, 40);
  ASSERT_TRUE(controller->setPlan(plan));

  // Robot exactly at the plan tail: the reference freezes at zero velocity,
  // so the command magnitude must collapse toward zero.
  ayanami::geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.position = Eigen::Vector3d(4.0, 0.0, 0.0);
  ayanami::geometry_msgs::TwistStamped velocity;
  ayanami::geometry_msgs::TwistStamped cmd;
  auto result = controller->computeVelocityCommand(pose, velocity, cmd);
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_LT(cmd.twist.linear.head<2>().norm(), 0.1);
}

TEST_F(RosemapMpcControllerTest, NearTailShortOfGoalKeepsApproaching) {
  // Regression (sentry rosemap run): the past-tail freeze used to seed from
  // the ROBOT position, so the endgame reference was "stay where you are"
  // and the robot stalled ~0.2 m short of the goal until the progress
  // checker aborted the task. The freeze must seed from the plan TAIL.
  const Path plan = straightPlan(4.0, 40);
  ASSERT_TRUE(controller->setPlan(plan));

  ayanami::geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.position = Eigen::Vector3d(3.98, 0.0, 0.0);  // nearest = tail
  pose.pose.orientation =
      Eigen::Quaterniond(Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ()));
  ayanami::geometry_msgs::TwistStamped velocity;
  ayanami::geometry_msgs::TwistStamped cmd;
  auto result = controller->computeVelocityCommand(pose, velocity, cmd);
  ASSERT_TRUE(result.ok) << result.message;
  // The command must keep pushing toward the tail (+x), not collapse (the
  // pre-fix behavior was ~1e-11: freeze on the robot's own position).
  EXPECT_GT(cmd.twist.linear.x(), 0.05);
}

TEST_F(RosemapMpcControllerTest, EndgameRotatesOntoGoalHeading) {
  // The plan tail carries the goal yaw (planner contract); with collapsed
  // translation near the tail, the "heading" mode must rotate the body onto
  // it -- otherwise the goal checker's angle tolerance never closes.
  Path plan = straightPlan(4.0, 40);
  plan.poses.back().pose.orientation =
      Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 2.0,
                                           Eigen::Vector3d::UnitZ()));
  ASSERT_TRUE(controller->setPlan(plan));

  ayanami::geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.position = Eigen::Vector3d(4.0, 0.0, 0.0);  // at the tail
  pose.pose.orientation =
      Eigen::Quaterniond(Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ()));
  ayanami::geometry_msgs::TwistStamped velocity;
  ayanami::geometry_msgs::TwistStamped cmd;
  auto result = controller->computeVelocityCommand(pose, velocity, cmd);
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_NEAR(cmd.twist.linear.head<2>().norm(), 0.0, 0.1);
  EXPECT_NEAR(cmd.twist.angular.z(),
              2.0 * M_PI / 2.0, 0.1);  // gain * (+pi/2) error

  // Far from the tail with collapsed translation (mid-path stall): no spin.
  pose.pose.position = Eigen::Vector3d(0.0, 0.0, 0.0);
  ayanami::geometry_msgs::TwistStamped cmd2;
  result = controller->computeVelocityCommand(pose, velocity, cmd2);
  ASSERT_TRUE(result.ok) << result.message;
  // At the plan START the tracking command is alive, so the heading mode
  // follows the velocity direction; only assert the endgame branch did not
  // fire here (angular sign is velocity-driven, not goal-driven).
  EXPECT_GT(cmd2.twist.linear.head<2>().norm(), 0.1);
}

TEST_F(RosemapMpcControllerTest, NoPlanRefuses) {
  ayanami::geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.position = Eigen::Vector3d(0.0, 0.0, 0.0);
  ayanami::geometry_msgs::TwistStamped velocity;
  ayanami::geometry_msgs::TwistStamped cmd;
  auto result = controller->computeVelocityCommand(pose, velocity, cmd);
  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.error, ErrorCode::NO_VALID_COMMAND);
}

TEST_F(RosemapMpcControllerTest, ParamStructLoadsFromConfig) {
  const auto dir = std::filesystem::temp_directory_path() /
                   "rosemap_mpc_controller_test_cfg";
  std::filesystem::create_directories(dir);
  {
    std::ofstream ofs(dir / "module.json");
    ofs << R"({
      "rosemap_mpc_controller": {
        "max_speed": 1.0,
        "predict_steps": 5
      }
    })";
  }
  ayanami::Config cfg((dir / "module.json").string());
  controller->configureParameters(
      ayanami::ModuleContext{"rosemap_mpc_controller", cfg});

  // Behavior-level check: the tightened speed box caps the command.
  ASSERT_TRUE(controller->setPlan(straightPlan(6.0, 60)));
  ayanami::geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.position = Eigen::Vector3d(0.0, 0.0, 0.0);
  ayanami::geometry_msgs::TwistStamped velocity;
  ayanami::geometry_msgs::TwistStamped cmd;
  auto result = controller->computeVelocityCommand(pose, velocity, cmd);
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_LE(cmd.twist.linear.x(), 1.0 + 1e-6);
}

}  // namespace
