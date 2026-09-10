// modules/costmap_pid_controller (CostmapPidController: 3-axis PID +
// acceleration limits + Twist zeroing + same-goal no-reset + goal_tolerance alignment).
//
// Coverage:
// - the first linear.x frame is a small max_acc*dt ramp (not dirty memory / a spike)
// - angular velocity ramp: the first frame after setPlan has |angular.z| <= max_angular_accel*dt, climbing frame by frame afterwards
// - repeated setPlan with the same plan does not reset PID/last_vel state; only a different goal resets
// - position inside the goal_checker XY tolerance -> linear=0 (on the distanceDecay stop line)
// - position reached but heading not yet -> linear=0 with non-zero angular (only the angular PID aligns to the goal yaw)
// - multi-frame simulation -> goal reached -> stops at zero velocity; the goal_checker finally declares arrival
// - cancel mid-way -> returns ErrorCode::CANCELED + zero velocity
// - Twist zeroing: linear.z / angular.x / angular.y are always 0, no NaN/dirty memory
// - module.cpp exports the unified create_extension_module factory (verified via dlopen)
// - the stop-line velocity XY bound is the tighter of the config goal_tolerance and the arrival
//   contract injected by the execution layer from the goal checker (arrival itself is decided by
//   the execution-layer goal_checker)

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <string>

#include <Eigen/Geometry>

#include <costmap_2d/layered_costmap.hpp>
#include <costmap_server/costmap_wrapper.hpp>

#include <ayanami/abstract/core/abstract_controller.hpp>
#include <ayanami/abstract/core/abstract_goal_checker.hpp>
#include <ayanami/costmap/components/costmap_map_context.hpp>
#include <ayanami/costmap/core/costmap_controller.hpp>
#include <ayanami/messages/failure.hpp>
#include <ayanami/types.hpp>
#include <ayanami/messages/geometry_msgs.hpp>
#include <ayanami/messages/nav_msgs.hpp>
#include <costmap_pid_controller/costmap_pid_controller.hpp>
#include <simple_goal_checker/simple_goal_checker.hpp>
#include <ayanami/utility/config.hpp>
#include <ayanami/utility/extension_module.hpp>

namespace {

using ayanami::abstract::AbstractGoalChecker;
using ayanami::costmap::CostmapMapContext;
using ayanami::geometry_msgs::PoseStamped;
using ayanami::ErrorCode;
using ayanami::geometry_msgs::TwistStamped;
using ayanami::modules::CostmapPidController;
using ayanami::modules::SimpleGoalChecker;
using ayanami::nav_msgs::Path;

std::string g_module_so;
std::filesystem::path g_config_dir;

// Values match the temp costmap_pid_controller.json written by tests/main.
constexpr double k_max_acc = 1.0;
constexpr double k_max_ang_accel = 1.0;
constexpr double k_dt = 0.05;
constexpr double k_pi = 3.14159265358979323846;

PoseStamped makePose(double x, double y, double yaw = 0.0) {
  auto p = PoseStamped::fromIsometry("map", Eigen::Isometry3d::Identity());
  p.pose.position = Eigen::Vector3d(x, y, 0.0);
  p.pose.orientation =
      Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
  return p;
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


std::shared_ptr<costmap_2d::CostmapWrapper> makeWrapper() {
  auto lm = std::make_shared<costmap_2d::LayeredCostmap>(
      "map", /*rolling_window=*/false, /*track_unknown=*/false);
  lm->resizeMap(20, 20, 0.5, 0.0, 0.0);
  return std::make_shared<costmap_2d::CostmapWrapper>(lm, "map");
}

std::shared_ptr<CostmapMapContext> makeContext() {
  auto ctx = std::make_shared<CostmapMapContext>();
  ctx->setGlobalCostmap(makeWrapper());
  ctx->setLocalCostmap(makeWrapper());
  return ctx;
}
std::shared_ptr<CostmapPidController> makeController() {
  auto ctl = std::make_shared<CostmapPidController>();
  // New API: parameters are injected via ModuleContext.config (this module json); the map is received per the family contract.
  ctl->configure(ayanami::ModuleContext{
                     "test_pid",
                     ayanami::Config((g_config_dir /
                                      "costmap_pid_controller.json").string())},
                 makeContext()->getSharedConstPtr());
  return ctl;
}

/// The goal_checker is only for arrival assertions in tests; computeVelocityCommand no longer receives
/// a goal_checker (the PID stop-line XY bounds use config.goal_tolerance).
std::shared_ptr<SimpleGoalChecker> makeGoalChecker() {
  return std::make_shared<SimpleGoalChecker>();
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

std::string configContent(const std::string& mode) {
  std::string content = "{\n";
  content += "  \"test_pid\": {\n";
  content += "    \"max_vel_x\": 2.0,\n";
  content += "    \"max_vel_theta\": 1.0,\n";
  content += "    \"max_acc_x\": 1.0,\n";
  content += "    \"max_decel_x\": 1.0,\n";
  content += "    \"max_acc_theta\": 1.0,\n";
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

// ---------- First-frame ramp (Twist zeroing + acceleration limits) ----------

TEST(CostmapPidControllerTest, firstFrameLinearRampsNotSpike) {
  auto ctl = makeController();
  ASSERT_TRUE(ctl->setPlan(makeXPath()));

  ErrorCode failure = ErrorCode::NONE;
  std::string message;
  const TwistStamped cmd = compute(ctl, makePose(0.0, 0.0), failure, message);

  EXPECT_EQ(failure, ErrorCode::NONE);
  // Must be a small max_acc*dt ramp value, not a dirty-memory spike
  EXPECT_TRUE(std::isfinite(cmd.twist.linear.x()));
  EXPECT_GT(cmd.twist.linear.x(), 0.0);
  EXPECT_NEAR(cmd.twist.linear.x(), k_max_acc * k_dt, 1e-9);
  EXPECT_LT(cmd.twist.linear.x(), 0.2);
}

// ---------- Angular velocity ramp ----------

TEST(CostmapPidControllerTest, angularVelocityRampsWithMaxAngularAccel) {
  auto ctl = makeController();
  ASSERT_TRUE(ctl->setPlan(makeXPath()));

  // The body faces +Y (90 deg off the +X path), needing a large angular correction -> verifies the first frame is capped by the ramp.
  const PoseStamped robot = makePose(0.0, 0.0, k_pi / 2.0);
  ErrorCode failure = ErrorCode::NONE;
  std::string message;
  double prev_mag = 0.0;
  for (int i = 0; i < 5; ++i) {
    const TwistStamped cmd = compute(ctl, robot, failure, message);
    ASSERT_EQ(failure, ErrorCode::NONE);
    const double mag = std::abs(cmd.twist.angular.z());
    EXPECT_GT(mag, 0.0);
    if (i == 0) {
      // The first frame must be capped by max_angular_accel, not the full PID value (~ kp_w*error ~ pi/2)
      EXPECT_LE(mag, k_max_ang_accel * k_dt + 1e-9);
    }
    // Monotonically climbing frame by frame -> proves ramping rather than one big angular step
    EXPECT_GT(mag, prev_mag);
    prev_mag = mag;
  }
}

// ---------- Same goal no-reset / new goal reset ----------

TEST(CostmapPidControllerTest, repeatSetPlanDoesNotResetState) {
  auto ctl = makeController();
  const Path plan = makeXPath();
  ASSERT_TRUE(ctl->setPlan(plan));

  const PoseStamped robot = makePose(0.5, 0.0);
  ErrorCode failure = ErrorCode::NONE;
  std::string message;
  const TwistStamped c1 = compute(ctl, robot, failure, message);
  ASSERT_EQ(failure, ErrorCode::NONE);
  const TwistStamped c2 = compute(ctl, robot, failure, message);
  ASSERT_EQ(failure, ErrorCode::NONE);
  ASSERT_GT(c2.twist.linear.x(), c1.twist.linear.x());

  // Repeated setPlan with the same plan -> no reset: the ramp continues 0.05->0.10->0.15
  ASSERT_TRUE(ctl->setPlan(plan));
  const TwistStamped c3 = compute(ctl, robot, failure, message);
  ASSERT_EQ(failure, ErrorCode::NONE);

  EXPECT_NEAR(c1.twist.linear.x(), k_max_acc * k_dt, 1e-9);
  EXPECT_NEAR(c2.twist.linear.x(), 2.0 * k_max_acc * k_dt, 1e-9);
  EXPECT_NEAR(c3.twist.linear.x(), 3.0 * k_max_acc * k_dt, 1e-9);
  EXPECT_GT(c3.twist.linear.x(), c1.twist.linear.x());
}

TEST(CostmapPidControllerTest, newGoalSetPlanResetsState) {
  auto ctl = makeController();
  ASSERT_TRUE(ctl->setPlan(makeXPath()));

  const PoseStamped robot = makePose(0.5, 0.0);
  ErrorCode failure = ErrorCode::NONE;
  std::string message;
  const TwistStamped c1 = compute(ctl, robot, failure, message);
  ASSERT_EQ(failure, ErrorCode::NONE);
  const TwistStamped c2 = compute(ctl, robot, failure, message);
  ASSERT_EQ(failure, ErrorCode::NONE);
  ASSERT_GT(c2.twist.linear.x(), c1.twist.linear.x());

  // A different goal -> must reset: last_linear_vel zeroed, starting over from the ramp
  Path plan2 = makeXPath();
  plan2.poses.back() = makePose(12.0, 0.0);
  ASSERT_TRUE(ctl->setPlan(plan2));
  const TwistStamped c3 = compute(ctl, robot, failure, message);
  ASSERT_EQ(failure, ErrorCode::NONE);

  EXPECT_NEAR(c3.twist.linear.x(), k_max_acc * k_dt, 1e-9);
  EXPECT_LT(c3.twist.linear.x(), c2.twist.linear.x());
}

// ---------- Stop-line velocity within XY tolerance + goal_checker arrival ----------

// The robot is inside the goal XY tolerance (heading aligned): linear=0, angular=0 (fully stationary),
// all goal_checker conditions satisfied -> arrival declared.
TEST(CostmapPidControllerTest, withinXyToleranceStopsLinearAndCheckerReaches) {
  auto ctl = makeController();
  auto checker = makeGoalChecker();
  ASSERT_TRUE(ctl->setPlan(makeXPath()));

  // Place the robot directly near the goal (within the goal_checker default position_tolerance=0.1).
  const PoseStamped near_goal = makePose(8.95, 0.0);
  ErrorCode failure = ErrorCode::NONE;
  std::string message;
  const TwistStamped cmd = compute(ctl, near_goal, failure, message);

  EXPECT_EQ(failure, ErrorCode::NONE);
  EXPECT_NEAR(cmd.twist.linear.x(), 0.0, 1e-9);
  EXPECT_NEAR(cmd.twist.linear.y(), 0.0, 1e-9);
  EXPECT_NEAR(cmd.twist.linear.z(), 0.0, 1e-9);
  EXPECT_NEAR(cmd.twist.angular.z(), 0.0, 1e-9);
  // Arrival is decided by the goal_checker (position + heading + velocity all satisfied).
  EXPECT_TRUE(checker->isGoalReached(near_goal, makePose(9.0, 0.0),
                                     TwistStamped{}));
}

// Position reached but heading not yet: linear=0 with non-zero angular (only the angular PID aligns to the goal yaw);
// the goal_checker does not declare arrival because the heading error is too large.
TEST(CostmapPidControllerTest, positionReachedHeadingNotYetSpinsInPlace) {
  auto ctl = makeController();
  auto checker = makeGoalChecker();
  ASSERT_TRUE(ctl->setPlan(makeXPath()));

  // The robot is at the goal position but faces +Y (90 deg off the +X goal): within the distance tolerance -> stop line,
  // the angular PID aligns to the goal yaw.
  const PoseStamped at_goal_sideways = makePose(9.0, 0.0, k_pi / 2.0);
  ErrorCode failure = ErrorCode::NONE;
  std::string message;
  const TwistStamped cmd = compute(ctl, at_goal_sideways, failure, message);

  EXPECT_EQ(failure, ErrorCode::NONE);
  EXPECT_NEAR(cmd.twist.linear.x(), 0.0, 1e-9);
  EXPECT_NEAR(cmd.twist.linear.y(), 0.0, 1e-9);
  EXPECT_NEAR(cmd.twist.linear.z(), 0.0, 1e-9);
  EXPECT_NE(std::abs(cmd.twist.angular.z()), 0.0);  // In-place rotation to correct the heading
  // The goal position is reached, but the heading 90 deg > angle_tolerance (0.2 rad) -> not arrived.
  EXPECT_FALSE(checker->isGoalReached(at_goal_sideways, makePose(9.0, 0.0),
                                      TwistStamped{}));
}

// ---------- Multi-frame simulation converges to the goal and stops ----------

TEST(CostmapPidControllerTest, multiFrameReachesGoalAndStops) {
  auto ctl = makeController();
  auto checker = makeGoalChecker();
  ASSERT_TRUE(ctl->setPlan(makeXPath()));

  PoseStamped robot = makePose(0.0, 0.0);
  ErrorCode failure = ErrorCode::NONE;
  std::string message;
  double prev_x = 0.0;
  bool stopped = false;
  for (int i = 0; i < 2000 && !stopped; ++i) {
    const TwistStamped cmd = compute(ctl, robot, failure, message);
    ASSERT_EQ(failure, ErrorCode::NONE) << message;
    if (cmd.twist.linear.x() < 1e-6) {
      stopped = true;
      break;
    }
    // Manual simulation: advance along the body x.
    robot.pose.position.x() += cmd.twist.linear.x() * k_dt;
    EXPECT_GT(robot.pose.position.x(), prev_x);
    prev_x = robot.pose.position.x();
  }

  EXPECT_TRUE(stopped);
  EXPECT_GT(robot.pose.position.x(), 8.0);  // Now close to the goal
  // The goal_checker (the only arrival decision) sees position within tolerance + aligned heading + zero velocity -> arrived.
  EXPECT_TRUE(checker->isGoalReached(robot, makePose(9.0, 0.0),
                                     TwistStamped{}));
}

// ---------- The arrival contract comes from the goal checker (single source) ----------
// The controller's goal_tolerance is a tracking preference, not the contract. A checker whose XY
// tolerance is tighter than that preference is unsatisfiable from the controller's own stop line:
// the robot would align its heading outside the acceptance region, isGoalReached would never fire,
// and the run would die on patience having achieved nothing.

TEST(CostmapPidControllerTest, tighterCheckerToleranceMovesTheStopLine) {
  const PoseStamped near_goal = makePose(8.95, 0.0);  // 0.05 m from the goal
  ErrorCode failure = ErrorCode::NONE;
  std::string message;

  // Baseline: no contract injected -> the controller's own goal_tolerance (0.1 m) applies.
  auto plain = makeController();
  ASSERT_TRUE(plain->setPlan(makeXPath()));
  const TwistStamped stopped = compute(plain, near_goal, failure, message);
  ASSERT_EQ(failure, ErrorCode::NONE);
  ASSERT_NEAR(stopped.twist.linear.x(), 0.0, 1e-9);

  // Contract injected, tighter than the preference -> the controller must keep translating.
  auto guarded = makeController();
  ASSERT_TRUE(guarded->setPlan(makeXPath()));
  guarded->setGoalTolerances(ayanami::PoseTolerance{0.02, 0.2});
  const TwistStamped moving = compute(guarded, near_goal, failure, message);
  EXPECT_EQ(failure, ErrorCode::NONE);
  EXPECT_GT(moving.twist.linear.x(), 1e-6)
      << "controller held station outside the goal checker's tolerance";
}

TEST(CostmapPidControllerTest, tighterCheckerToleranceIsReachable) {
  auto ctl = makeController();
  ctl->setGoalTolerances(ayanami::PoseTolerance{0.02, 0.2});
  ASSERT_TRUE(ctl->setPlan(makeXPath()));

  PoseStamped robot = makePose(0.0, 0.0);
  ErrorCode failure = ErrorCode::NONE;
  std::string message;
  double closest = std::numeric_limits<double>::infinity();
  // Budget note: distanceDecay is quadratic near the goal (v ~ d^2/2), so closing the last
  // few centimetres takes on the order of a hundred control cycles. At 20 Hz that is seconds
  // of real time, which is fine for navigation but needs a generous loop budget here.
  for (int i = 0; i < 6000; ++i) {
    closest = std::min(closest, std::abs(9.0 - robot.pose.position.x()));
    const TwistStamped cmd = compute(ctl, robot, failure, message);
    ASSERT_EQ(failure, ErrorCode::NONE) << message;
    if (cmd.twist.linear.x() < 1e-6) {
      break;
    }
    robot.pose.position.x() += cmd.twist.linear.x() * k_dt;
  }
  // The stop line sits at the injected 0.02 m, not the controller's 0.1 m.
  EXPECT_LT(closest, 0.02) << "robot stopped short of the injected arrival contract";
}

// ---------- cancel ----------

TEST(CostmapPidControllerTest, cancelReturnsCanceledOutcomeAndZero) {
  auto ctl = makeController();
  ASSERT_TRUE(ctl->setPlan(makeXPath()));

  const PoseStamped robot = makePose(0.0, 0.0);
  ErrorCode failure = ErrorCode::NONE;
  std::string message;
  const TwistStamped c1 = compute(ctl, robot, failure, message);
  ASSERT_EQ(failure, ErrorCode::NONE);

  ASSERT_TRUE(ctl->cancel());
  const TwistStamped cmd = compute(ctl, robot, failure, message);
  EXPECT_EQ(failure, ErrorCode::CANCELED);
  EXPECT_NEAR(cmd.twist.linear.x(), 0.0, 1e-9);
  EXPECT_NEAR(cmd.twist.linear.y(), 0.0, 1e-9);
  EXPECT_NEAR(cmd.twist.linear.z(), 0.0, 1e-9);
  EXPECT_NEAR(cmd.twist.angular.z(), 0.0, 1e-9);
}

// ---------- Twist zeroing (dirty-memory protection) ----------

TEST(CostmapPidControllerTest, twistClearedNoDirtyMemory) {
  auto ctl = makeController();
  ASSERT_TRUE(ctl->setPlan(makeXPath()));

  const PoseStamped robot = makePose(0.0, 0.0);
  // Pollute the previous cmd_vel (the old dirty-memory bug scenario): the new API returns a fresh ControllerCommand;
  // the controller must return output on top of an all-fields-zeroed base, with no dirty-memory leakage.
  TwistStamped dirty;
  dirty.twist.linear = Eigen::Vector3d(std::nan(""), -1e300, 42.0);
  dirty.twist.angular = Eigen::Vector3d(
      std::numeric_limits<double>::infinity(), 7.5, 3.14);
  dirty.header.frame_id = "garbage";
  (void)dirty;

  ErrorCode failure = ErrorCode::NONE;
  std::string message;
  TwistStamped cmd;
  const auto result = ctl->computeVelocityCommand(robot, TwistStamped{}, cmd);
  ASSERT_TRUE(result.ok);

  EXPECT_EQ(result.error, ErrorCode::NONE);
  EXPECT_TRUE(std::isfinite(cmd.twist.linear.x()));
  EXPECT_NEAR(cmd.twist.linear.z(), 0.0, 1e-12);
  EXPECT_NEAR(cmd.twist.angular.x(), 0.0, 1e-12);
  EXPECT_NEAR(cmd.twist.angular.y(), 0.0, 1e-12);
  EXPECT_TRUE(std::isfinite(cmd.twist.angular.z()));
  EXPECT_EQ(cmd.header.frame_id, robot.header.frame_id);
}

// ---------- Omni mode uses the lateral axis ----------

TEST(CostmapPidControllerTest, omniModeUsesLateralAxis) {
  writeConfig("omni");
  auto ctl = makeController();
  writeConfig("diff");  // Restore so later tests are unaffected

  Path p;
  p.header.frame_id = "map";
  for (int i = 0; i < 10; ++i) {
    p.poses.push_back(makePose(0.0, static_cast<double>(i)));  // Along +Y
  }
  ASSERT_TRUE(ctl->setPlan(p));

  ErrorCode failure = ErrorCode::NONE;
  std::string message;
  const TwistStamped cmd = compute(ctl, makePose(0.0, 0.0), failure, message);

  EXPECT_EQ(failure, ErrorCode::NONE);
  // Omni: the lateral axis (linear.y) carries the forward motion; the first frame is capped by the max_acc*dt ramp.
  EXPECT_NEAR(cmd.twist.linear.y(), k_max_acc * k_dt, 1e-9);
  EXPECT_NEAR(cmd.twist.linear.x(), 0.0, 1e-9);
}

// ---------- costmap family inheritance + configure ----------

TEST(CostmapPidControllerTest, inheritsCostmapControllerAndInitializes) {
  auto ctl = makeController();

  // Now in the costmap family: dynamic_cast from the AbstractController root reaches CostmapController.
  ayanami::abstract::AbstractController::Ptr base = ctl;
  auto cm =
      std::dynamic_pointer_cast<ayanami::costmap::CostmapController>(base);
  ASSERT_NE(cm, nullptr);

  // The algorithm does not read the map, but implements configure(context, map) per the family contract.
  EXPECT_NO_THROW(cm->configure(
      ayanami::ModuleContext{
          "test_pid",
          ayanami::Config((g_config_dir /
                           "costmap_pid_controller.json").string())},
      makeContext()->getSharedConstPtr()));

  // PID behavior is unaffected after initialize.
  ASSERT_TRUE(ctl->setPlan(makeXPath()));
  ErrorCode failure = ErrorCode::NONE;
  std::string message;
  const TwistStamped cmd = compute(ctl, makePose(0.0, 0.0), failure, message);
  EXPECT_EQ(failure, ErrorCode::NONE);
  EXPECT_NEAR(cmd.twist.linear.x(), k_max_acc * k_dt, 1e-9);
}

// ---------- module.cpp factory symbols ----------

TEST(CostmapPidControllerTest, moduleFactoryCreatesDerivedController) {
  ASSERT_FALSE(g_module_so.empty());
  // Common factory + typed cast: returns the real derived instance.
  auto controller = std::dynamic_pointer_cast<CostmapPidController>(
      ayanami::ExtensionModule::loadModule(g_module_so));
  ASSERT_NE(controller, nullptr);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  if (argc > 1) {
    g_module_so = argv[1];
  }

  // Temp config dir (strictly single-level index): read explicitly by the module constructor via
  // GlobalConfig::getConfigPath("config_pid_controller") (no
  // thread_local injection).
  g_config_dir =
      std::filesystem::temp_directory_path() / "ayanami_costmap_pid_test";
  std::filesystem::remove_all(g_config_dir);
  std::filesystem::create_directories(g_config_dir);
  {
    std::ofstream ofs(g_config_dir / "config.json");
    ofs << R"({"global": { "config_pid_controller": "costmap_pid_controller.json" }})";
  }
  writeConfig("diff");
  ayanami::GlobalConfig::instance(g_config_dir.string(), true);

  return RUN_ALL_TESTS();
}
