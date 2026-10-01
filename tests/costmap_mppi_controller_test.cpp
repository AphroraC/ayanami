// modules/costmap_mppi_controller (MPPI local control in the 4D swerve
// control space).
//
// Coverage:
// - swerve kinematics: pure translation / pure rotation 4D->twist, steer
//   wrap + rate limit, control clamping, 8D inverse consistency
// - mppi core: deterministic sampling under a fixed seed, softmax weighted
//   update (min-cost dominance, equal-cost averaging, non-finite
//   rejection), sequence shift, rollout shape + straight-line + fixed-spin
//   model consistency
// - critics: path follow monotone tracking, goal/goal-angle gating,
//   twirling free-mode gate, twist constraints, deadband, obstacle
//   rejection scale
// - plugin contract: setPlan validation, no-plan error, cancel, first
//   command drives toward the goal, arrival zeroes translation, obstacle
//   avoidance deviates the command, fixed/zero angular modes, goal
//   tolerance bounds the stop radius, factory dlopen
// - performance: full MPPI solve p95 under the 100 Hz budget

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <costmap_2d/costmap_2d.hpp>
#include <costmap_2d/layered_costmap.hpp>
#include <costmap_2d_layers/static_layer.hpp>
#include <costmap_server/costmap_wrapper.hpp>

#include <ayanami/costmap/components/costmap_map_context.hpp>
#include <ayanami/costmap/core/costmap_controller.hpp>
#include <ayanami/messages/failure.hpp>
#include <ayanami/types.hpp>
#include <ayanami/utility/config.hpp>
#include <ayanami/utility/extension_module.hpp>
#include <costmap_mppi_controller/costmap_mppi_controller.hpp>
#include <costmap_mppi_controller/mppi_core.hpp>
#include <costmap_mppi_controller/mppi_critics.hpp>
#include <costmap_mppi_controller/swerve_kinematics.hpp>

#include <algorithm>
#include <chrono>
#include <limits>
#include <iostream>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace {

using ayanami::ErrorCode;
using ayanami::ResultT;
using ayanami::costmap::CostmapMapContext;
using ayanami::geometry_msgs::PoseStamped;
using ayanami::geometry_msgs::TwistStamped;
using ayanami::modules::mppi::ControlSequence;
using ayanami::modules::mppi::State;
using ayanami::modules::mppi::Trajectory;
using ayanami::modules::swerve::Control4D;
using ayanami::modules::swerve::SwerveGeometry;
using ayanami::nav_msgs::Path;
namespace swerve = ayanami::modules::swerve;
namespace mppi = ayanami::modules::mppi;
namespace critics = ayanami::modules::mppi::critics;

std::string g_module_so;

constexpr double k_pi = 3.14159265358979323846;

SwerveGeometry squareGeometry() {
  return SwerveGeometry{0.2, 0.2, 0.2, 0.2, 0.076};
}

swerve::ControlLimits wideLimits() {
  return swerve::ControlLimits{10.0, 2.6, 40.0};  // steer rate / range / wheel
}

// ---------- swerve kinematics ----------

TEST(SwerveKinematicsTest, pureTranslationConvertsToTwist) {
  const auto geometry = squareGeometry();
  const double direction = 0.7;
  const double speed = 1.5;  // m/s per wheel
  Control4D control{direction, direction, speed, speed};
  const swerve::BodyTwist twist = swerve::control4dToTwist(control, geometry);
  EXPECT_NEAR(twist.vx, speed * std::cos(direction), 1e-9);
  EXPECT_NEAR(twist.vy, speed * std::sin(direction), 1e-9);
  EXPECT_NEAR(twist.omega, 0.0, 1e-9);
}

TEST(SwerveKinematicsTest, pureRotationConvertsToTwist) {
  const auto geometry = squareGeometry();
  const double omega = 2.0;
  const double wheel_speed = omega * std::hypot(geometry.l_f, geometry.d_l);
  // Wheel positions: fl = (l_f, d_l), rr = (-l_r, -d_r); the rigid-body
  // wheel velocity is (-py*omega, px*omega).
  const double fl_steer = std::atan2(geometry.l_f * omega,
                                     -geometry.d_l * omega);
  const double rr_steer = std::atan2(-geometry.l_r * omega,
                                     geometry.d_r * omega);
  Control4D control{fl_steer, rr_steer, wheel_speed, wheel_speed};
  const swerve::BodyTwist twist = swerve::control4dToTwist(control, geometry);
  EXPECT_NEAR(twist.vx, 0.0, 1e-9);
  EXPECT_NEAR(twist.vy, 0.0, 1e-9);
  EXPECT_NEAR(twist.omega, omega, 1e-9);
}

TEST(SwerveKinematicsTest, approachSteerWrapsAndRateLimits) {
  // Wrap: shortest arc across +-pi.
  const double wrapped = swerve::approachSteer(3.0, -3.0, 10.0);
  EXPECT_NEAR(wrapped, -3.0, 1e-9);
  // Rate limit: at most max_delta toward the target.
  const double limited = swerve::approachSteer(0.0, 1.0, 0.2);
  EXPECT_NEAR(limited, 0.2, 1e-12);
  EXPECT_NEAR(swerve::approachSteer(0.0, -1.0, 0.2), -0.2, 1e-12);
}

TEST(SwerveKinematicsTest, clampControlEnforcesFeasibleSet) {
  swerve::ControlLimits limits{4.0, 1.6, 30.0};
  const Control4D clamped =
      swerve::clampControl(Control4D{5.0, -5.0, -1.0, 100.0}, limits);
  EXPECT_NEAR(clamped.fl_steer, 1.6, 1e-12);
  EXPECT_NEAR(clamped.rr_steer, -1.6, 1e-12);
  EXPECT_NEAR(clamped.fl_vel, 0.0, 1e-12);   // negative wheel speed clamped
  EXPECT_NEAR(clamped.rr_vel, 30.0, 1e-12);
}

TEST(SwerveKinematicsTest, rolloutStepRateLimitsSteer) {
  const auto geometry = squareGeometry();
  const auto limits = wideLimits();
  swerve::RolloutState state;
  state.yaw = 0.0;
  swerve::BodyTwist twist;
  const double dt = 0.05;
  // Steer target 1.0 rad away from the current 0: the FIRST step moves the
  // steer exactly max_rate*dt (the motor cannot jump).
  const swerve::RolloutState next = swerve::rolloutStep(
      state, Control4D{1.0, 1.0, 10.0, 10.0}, geometry, limits, dt,
      std::numeric_limits<double>::quiet_NaN(), twist);
  EXPECT_NEAR(next.fl_steer, limits.max_steer_rate * dt, 1e-12);
  EXPECT_NEAR(next.rr_steer, limits.max_steer_rate * dt, 1e-12);
  // The executed twist reflects the POST-approach steer (drives at the
  // reduced angle, not the sampled target).
  EXPECT_NEAR(twist.vx, 10.0 * std::cos(0.5), 1e-9);
}

TEST(SwerveKinematicsTest, twistToWheels8DMatchesDiagPair) {
  const auto geometry = squareGeometry();
  const swerve::BodyTwist twist{1.0, 0.5, 0.8};
  const auto wheels = swerve::twistToWheels8D(twist, geometry);
  // fl wheel (px=l_f, py=d_l): its velocity vector must match the
  // rigid-body point velocity.
  const double fl_vx = twist.vx - geometry.d_l * twist.omega;
  const double fl_vy = twist.vy + geometry.l_f * twist.omega;
  EXPECT_NEAR(wheels.steer[0], std::atan2(fl_vy, fl_vx), 1e-9);
  EXPECT_NEAR(wheels.rotor[0] * geometry.tire_radius,
              std::hypot(fl_vx, fl_vy), 1e-9);
}

// ---------- mppi core ----------

TEST(MppiCoreTest, samplerDeterministicUnderSeed) {
  mppi::Sampler a(42);
  mppi::Sampler b(42);
  std::vector<double> noise_a;
  std::vector<double> noise_b;
  mppi::NoiseSigma sigma{0.3, 4.0};
  a.fillNoise(noise_a, 8, 5, sigma);
  b.fillNoise(noise_b, 8, 5, sigma);
  ASSERT_EQ(noise_a.size(), noise_b.size());
  for (std::size_t i = 0; i < noise_a.size(); ++i) {
    EXPECT_EQ(noise_a[i], noise_b[i]);
  }
}

TEST(MppiCoreTest, weightedUpdateDominatesMinCost) {
  ControlSequence base(3, Control4D{0.0, 0.0, 0.0, 0.0});
  ControlSequence fast(3, Control4D{0.1, 0.1, 20.0, 20.0});
  ControlSequence slow(3, Control4D{0.0, 0.0, 2.0, 2.0});
  std::vector<ControlSequence> samples{fast, slow};
  std::vector<double> costs{0.0, 100.0};  // fast far better
  ControlSequence out;
  ASSERT_TRUE(mppi::weightedUpdate(base, samples, costs, 0.05, out));
  for (std::size_t t = 0; t < out.size(); ++t) {
    EXPECT_NEAR(out[t].fl_vel, fast[t].fl_vel, 1e-6);
  }
}

TEST(MppiCoreTest, weightedUpdateAveragesEqualCosts) {
  ControlSequence base(2, Control4D{0.0, 0.0, 0.0, 0.0});
  ControlSequence a(2, Control4D{0.0, 0.0, 10.0, 10.0});
  ControlSequence b(2, Control4D{0.0, 0.0, 20.0, 20.0});
  std::vector<ControlSequence> samples{a, b};
  std::vector<double> costs{1.0, 1.0};
  ControlSequence out;
  ASSERT_TRUE(mppi::weightedUpdate(base, samples, costs, 0.3, out));
  EXPECT_NEAR(out[0].fl_vel, 15.0, 1e-9);
}

TEST(MppiCoreTest, weightedUpdateIgnoresNonFiniteCosts) {
  ControlSequence base(1, Control4D{0.0, 0.0, 0.0, 0.0});
  ControlSequence good(1, Control4D{0.0, 0.0, 12.0, 12.0});
  ControlSequence bad(1, Control4D{0.0, 0.0, 30.0, 30.0});
  std::vector<ControlSequence> samples{good, bad};
  std::vector<double> costs{1.0, std::numeric_limits<double>::infinity()};
  ControlSequence out;
  ASSERT_TRUE(mppi::weightedUpdate(base, samples, costs, 0.3, out));
  EXPECT_NEAR(out[0].fl_vel, 12.0, 1e-9);
}

TEST(MppiCoreTest, shiftSequenceReplicatesTail) {
  ControlSequence sequence;
  sequence.push_back(Control4D{1.0, 1.0, 1.0, 1.0});
  sequence.push_back(Control4D{2.0, 2.0, 2.0, 2.0});
  sequence.push_back(Control4D{3.0, 3.0, 3.0, 3.0});
  mppi::shiftSequence(sequence);
  EXPECT_NEAR(sequence[0].fl_steer, 2.0, 1e-12);
  EXPECT_NEAR(sequence[1].fl_steer, 3.0, 1e-12);
  EXPECT_NEAR(sequence[2].fl_steer, 3.0, 1e-12);
}

TEST(MppiCoreTest, rolloutStraightLineAndFixedSpin) {
  const auto geometry = squareGeometry();
  const auto limits = wideLimits();
  // Pure translation along +x: both wheels steer 0 at equal speed.
  ControlSequence sequence(4, Control4D{0.0, 0.0, 1.0, 1.0});  // 1 m/s
  State start{0.0, 0.0, 0.0};
  Trajectory trajectory;
  mppi::rolloutTrajectory(start, 0.0, 0.0, sequence, geometry, limits, 0.1,
                          std::numeric_limits<double>::quiet_NaN(),
                          trajectory);
  ASSERT_EQ(trajectory.states.size(), 5u);
  ASSERT_EQ(trajectory.twists.size(), 4u);
  for (std::size_t t = 0; t < trajectory.states.size(); ++t) {
    EXPECT_NEAR(trajectory.states[t].x, 0.1 * static_cast<double>(t), 1e-9);
    EXPECT_NEAR(trajectory.states[t].y, 0.0, 1e-9);
  }
  // Fixed spin: omega override rotates the body while translation
  // continues -- the rollout model matches what the chassis executes.
  Trajectory spinning;
  mppi::rolloutTrajectory(start, 0.0, 0.0, sequence, geometry, limits, 0.1,
                          1.0, spinning);
  EXPECT_NEAR(spinning.states.back().yaw, 0.4, 1e-9);
  EXPECT_GT(spinning.states.back().x, 0.0);
}

// ---------- critics ----------

Trajectory makeTrajectoryAlongX(double y_offset = 0.0, int steps = 5,
                                double step = 0.1, double omega = 0.0) {
  Trajectory trajectory;
  for (int t = 0; t <= steps; ++t) {
    trajectory.states.push_back(
        State{step * t, y_offset, 0.0});
  }
  for (int t = 0; t < steps; ++t) {
    trajectory.twists.push_back(swerve::BodyTwist{1.0, 0.0, omega});
  }
  return trajectory;
}

Path makeXPath(int points = 11) {
  Path path;
  path.header.frame_id = "map";
  for (int i = 0; i < points; ++i) {
    PoseStamped pose = PoseStamped::fromIsometry(
        "map", Eigen::Isometry3d::Identity());
    pose.pose.position = Eigen::Vector3d(0.1 * i, 0.0, 0.0);
    path.poses.push_back(pose);
  }
  return path;
}

TEST(MppiCriticsTest, pathFollowPrefersOnPathAndProgress) {
  std::vector<Eigen::Vector2d> path;
  for (int i = 0; i <= 10; ++i) {
    path.emplace_back(0.1 * i, 0.0);
  }
  mppi::CriticData data;
  data.path = &path;
  data.w_path_follow = 1.0;
  const double on_path = critics::pathFollow(makeTrajectoryAlongX(0.0), data);
  const double off_path =
      critics::pathFollow(makeTrajectoryAlongX(0.5), data);
  EXPECT_NEAR(on_path, 0.0, 1e-9);
  EXPECT_GT(off_path, on_path);
}

TEST(MppiCriticsTest, goalAndGoalAngle) {
  mppi::CriticData data;
  data.goal_xy = Eigen::Vector2d(1.0, 0.0);
  data.goal_yaw = 0.0;
  data.w_goal = 1.0;
  data.w_goal_angle = 1.0;
  data.angular_free = true;
  data.goal_angle_radius = 0.5;
  Trajectory near = makeTrajectoryAlongX(0.0);  // ends at 0.5 from goal
  Trajectory far = makeTrajectoryAlongX(0.0, 5, 0.02);  // ends at 0.9
  EXPECT_LT(critics::goal(near, data), critics::goal(far, data));
  // Heading error counted only inside the radius.
  Trajectory wrong_heading = makeTrajectoryAlongX(0.0);
  wrong_heading.states.back().yaw = 1.0;
  EXPECT_GT(critics::goalAngle(wrong_heading, data), 0.0);
  data.angular_free = false;
  EXPECT_NEAR(critics::goalAngle(wrong_heading, data), 0.0, 1e-12);
  EXPECT_NEAR(critics::twirling(wrong_heading, data), 0.0, 1e-12);
}

TEST(MppiCriticsTest, constraintAndDeadband) {
  mppi::CriticData data;
  data.max_vel_x = 2.0;
  data.max_vel_y = 2.0;
  data.max_vel_theta = 3.0;
  data.w_constraint = 1.0;
  data.deadband_xy = 0.05;
  data.deadband_theta = 0.05;
  data.w_velocity_deadband = 1.0;
  Trajectory inside = makeTrajectoryAlongX();
  EXPECT_NEAR(critics::constraint(inside, data), 0.0, 1e-12);
  Trajectory over = makeTrajectoryAlongX();
  for (auto& twist : over.twists) {
    twist.vx = 3.0;
  }
  EXPECT_GT(critics::constraint(over, data), 0.0);
  // Deadband: zero and above-threshold are free, the middle is penalized.
  EXPECT_NEAR(critics::velocityDeadband(inside, data), 0.0, 1e-12);
  Trajectory micro = makeTrajectoryAlongX();
  for (auto& twist : micro.twists) {
    twist.vx = 0.02;
  }
  EXPECT_GT(critics::velocityDeadband(micro, data), 0.0);
}

TEST(MppiCriticsTest, obstaclesRejectionScale) {
  mppi::CriticData data;
  data.w_obstacles = 1.0;
  data.cost_at = [](double x, double y) {
    (void)y;
    return x > 0.25 ? 254 : 0;  // lethal wall just ahead
  };
  EXPECT_EQ(critics::obstacles(makeTrajectoryAlongX(), data),
            1e6);
  data.cost_at = [](double, double) { return 0; };
  EXPECT_NEAR(critics::obstacles(makeTrajectoryAlongX(), data), 0.0, 1e-12);
  // Total collapses on a lethal hit.
  data.cost_at = [](double, double) { return 254; };
  EXPECT_EQ(critics::totalCost(makeTrajectoryAlongX(), data), 1e6);
}

// ---------- plugin ----------

std::shared_ptr<costmap_2d::CostmapWrapper> makeWrapper(
    unsigned int size, double res,
    const std::function<void(costmap_2d::Costmap2D&)>& obstacle_fn = {}) {
  auto lm = std::make_shared<costmap_2d::LayeredCostmap>(
      "map", /*rolling_window=*/false, /*track_unknown=*/false);
  lm->resizeMap(size, size, res, 0.0, 0.0);
  costmap_2d::Costmap2D layer_map(size, size, res, 0.0, 0.0,
                                  costmap_2d::FREE_SPACE);
  if (obstacle_fn) {
    obstacle_fn(layer_map);
  }
  auto static_layer = std::make_shared<costmap_2d_layers::StaticLayer>();
  static_layer->setMap(layer_map);
  lm->addLayer(static_layer);
  lm->updateMap(0.0, 0.0, 0.0);
  return std::make_shared<costmap_2d::CostmapWrapper>(lm, "map");
}

std::shared_ptr<ayanami::modules::CostmapMppiController> makeController(
    const std::shared_ptr<costmap_2d::CostmapWrapper>& wrapper,
    ayanami::Config cfg = ayanami::Config("")) {
  auto controller =
      std::make_shared<ayanami::modules::CostmapMppiController>();
  auto ctx = std::make_shared<CostmapMapContext>();
  ctx->setGlobalCostmap(wrapper);
  // Deterministic solves; small map so the tests run fast.
  cfg.overrideParam<int>("test_mppi", "random_seed", 42);
  cfg.overrideParam<int>("test_mppi", "batch", 200);
  controller->configure(ayanami::ModuleContext{"test_mppi", std::move(cfg)},
                        ctx->getSharedConstPtr());
  return controller;
}

PoseStamped makePose(double x, double y, double yaw = 0.0) {
  auto p = PoseStamped::fromIsometry("map", Eigen::Isometry3d::Identity());
  p.pose.position = Eigen::Vector3d(x, y, 0.0);
  p.pose.orientation =
      Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
  return p;
}

Path makePathPoints(const std::vector<Eigen::Vector2d>& points) {
  Path path;
  path.header.frame_id = "map";
  for (const auto& p : points) {
    auto pose =
        PoseStamped::fromIsometry("map", Eigen::Isometry3d::Identity());
    pose.pose.position = Eigen::Vector3d(p.x(), p.y(), 0.0);
    path.poses.push_back(pose);
  }
  return path;
}

TEST(CostmapMppiControllerTest, setPlanValidation) {
  auto controller = makeController(makeWrapper(40, 0.5));
  EXPECT_FALSE(controller->setPlan(makePathPoints({{1.0, 1.0}})));
  EXPECT_TRUE(
      controller->setPlan(makePathPoints({{1.0, 1.0}, {5.0, 1.0}})));
}

TEST(CostmapMppiControllerTest, noPlanReturnsNoValidCommand) {
  auto controller = makeController(makeWrapper(40, 0.5));
  TwistStamped cmd;
  const ResultT<void> result =
      controller->computeVelocityCommand(makePose(1.0, 1.0), TwistStamped{},
                                         cmd);
  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.error, ErrorCode::NO_VALID_COMMAND);
}

TEST(CostmapMppiControllerTest, cancelReturnsCanceled) {
  auto controller = makeController(makeWrapper(40, 0.5));
  ASSERT_TRUE(controller->setPlan(
      makePathPoints({{1.0, 1.0}, {5.0, 1.0}})));
  EXPECT_TRUE(controller->cancel());
  TwistStamped cmd;
  const ResultT<void> result =
      controller->computeVelocityCommand(makePose(1.0, 1.0), TwistStamped{},
                                         cmd);
  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.error, ErrorCode::CANCELED);
}

TEST(CostmapMppiControllerTest, firstCommandDrivesTowardGoal) {
  auto controller = makeController(makeWrapper(40, 0.5));
  ASSERT_TRUE(controller->setPlan(
      makePathPoints({{1.0, 1.0}, {9.0, 1.0}})));
  TwistStamped cmd;
  const ResultT<void> result =
      controller->computeVelocityCommand(makePose(1.0, 1.0, 0.0),
                                         TwistStamped{}, cmd);
  ASSERT_TRUE(result.ok) << result.message;
  // Zero the dirty non-planar components.
  EXPECT_EQ(cmd.twist.linear.z(), 0.0);
  EXPECT_EQ(cmd.twist.angular.x(), 0.0);
  EXPECT_EQ(cmd.twist.angular.y(), 0.0);
  // Drives +x (the path direction) at a useful speed.
  EXPECT_GT(cmd.twist.linear.x(), 0.3);
  EXPECT_LT(std::abs(cmd.twist.linear.y()), 0.5);
  // Within the twist clamps.
  EXPECT_LE(cmd.twist.linear.x(), 2.0 + 1e-9);
}

TEST(CostmapMppiControllerTest, multiCyclesTrackWithoutNaN) {
  auto controller = makeController(makeWrapper(40, 0.5));
  ASSERT_TRUE(controller->setPlan(
      makePathPoints({{1.0, 1.0}, {9.0, 1.0}})));
  double x = 1.0;
  const double dt = 0.05;
  for (int cycle = 0; cycle < 40; ++cycle) {
    TwistStamped cmd;
    const ResultT<void> result =
        controller->computeVelocityCommand(makePose(x, 1.0, 0.0),
                                           TwistStamped{}, cmd);
    ASSERT_TRUE(result.ok) << result.message << " cycle " << cycle;
    ASSERT_TRUE(std::isfinite(cmd.twist.linear.x()));
    ASSERT_TRUE(std::isfinite(cmd.twist.linear.y()));
    ASSERT_TRUE(std::isfinite(cmd.twist.angular.z()));
    x += cmd.twist.linear.x() * dt;
  }
  EXPECT_GT(x, 1.5);  // made progress along the path
}

TEST(CostmapMppiControllerTest, arrivalZeroesTranslation) {
  auto controller = makeController(makeWrapper(40, 0.5));
  ASSERT_TRUE(controller->setPlan(
      makePathPoints({{1.0, 1.0}, {2.0, 1.0}})));
  TwistStamped cmd;
  const ResultT<void> result =
      controller->computeVelocityCommand(makePose(1.98, 1.0, 0.0),
                                         TwistStamped{}, cmd);
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_NEAR(cmd.twist.linear.x(), 0.0, 1e-12);
  EXPECT_NEAR(cmd.twist.linear.y(), 0.0, 1e-12);
}

TEST(CostmapMppiControllerTest, obstacleAheadDeviatesCommand) {
  // Wall across the direct line (column x=8..9 cells at 0.5 res = 4.0..5.0m,
  // y in [4.0, 6.0]) with gaps above and below; the controller must brake
  // or steer toward a gap instead of driving straight into it.
  auto wrapper = makeWrapper(40, 0.5, [](costmap_2d::Costmap2D& m) {
    for (unsigned int y = 8; y <= 12; ++y) {
      m.setCost(8, y, costmap_2d::LETHAL_OBSTACLE);
      m.setCost(9, y, costmap_2d::LETHAL_OBSTACLE);
    }
  });
  auto controller = makeController(wrapper);
  ASSERT_TRUE(controller->setPlan(
      makePathPoints({{1.0, 5.25}, {9.0, 5.25}})));
  TwistStamped cmd;
  const ResultT<void> result =
      controller->computeVelocityCommand(makePose(3.0, 5.25, 0.0),
                                         TwistStamped{}, cmd);
  ASSERT_TRUE(result.ok) << result.message;
  const double heading = std::atan2(cmd.twist.linear.y(),
                                    cmd.twist.linear.x());
  const bool deviated = std::abs(heading) > 0.15;
  // Braking = clearly below half the cruise speed (the full closed-loop
  // avoidance is covered by closedLoopReachesGoalAroundWall; this single
  // cycle asserts the wall is already FELT).
  const bool braked = std::hypot(cmd.twist.linear.x(), cmd.twist.linear.y()) <
                      1.0;
  EXPECT_TRUE(deviated || braked)
      << "cmd=(" << cmd.twist.linear.x() << ", " << cmd.twist.linear.y()
      << ") should deviate or brake in front of the wall";
}

TEST(CostmapMppiControllerTest, angularModes) {
  // Fixed (small gyro): omega pinned to the configured constant.
  ayanami::Config fixed_cfg("");
  fixed_cfg.overrideParam<std::string>("test_mppi", "angular_mode", "fixed");
  fixed_cfg.overrideParam<double>("test_mppi", "fixed_angular_z", 1.0);
  auto fixed_controller = makeController(makeWrapper(40, 0.5), fixed_cfg);
  ASSERT_TRUE(fixed_controller->setPlan(
      makePathPoints({{1.0, 1.0}, {9.0, 1.0}})));
  TwistStamped cmd;
  ASSERT_TRUE(fixed_controller
                  ->computeVelocityCommand(makePose(1.0, 1.0, 0.0),
                                           TwistStamped{}, cmd)
                  .ok);
  EXPECT_NEAR(cmd.twist.angular.z(), 1.0, 1e-9);

  // Zero: omega always zero.
  ayanami::Config zero_cfg("");
  zero_cfg.overrideParam<std::string>("test_mppi", "angular_mode", "zero");
  auto zero_controller = makeController(makeWrapper(40, 0.5), zero_cfg);
  ASSERT_TRUE(zero_controller->setPlan(
      makePathPoints({{1.0, 1.0}, {9.0, 1.0}})));
  ASSERT_TRUE(zero_controller
                  ->computeVelocityCommand(makePose(1.0, 1.0, 0.0),
                                           TwistStamped{}, cmd)
                  .ok);
  EXPECT_EQ(cmd.twist.angular.z(), 0.0);
}

TEST(CostmapMppiControllerTest, goalToleranceBoundsStopRadius) {
  auto controller = makeController(makeWrapper(40, 0.5));
  // Config stop radius 0.3; the checker contract 0.1 tightens it.
  ayanami::Config cfg("");
  cfg.overrideParam<double>("test_mppi", "goal_stop_radius", 0.3);
  auto bounded = makeController(makeWrapper(40, 0.5), cfg);
  bounded->setGoalTolerances(ayanami::PoseTolerance{0.1, 0.2});
  ASSERT_TRUE(bounded->setPlan(
      makePathPoints({{1.0, 1.0}, {2.0, 1.0}})));
  TwistStamped cmd;
  // 0.2 from the goal: inside the config radius, outside the tightened
  // one -> translation continues.
  ASSERT_TRUE(bounded
                  ->computeVelocityCommand(makePose(1.8, 1.0, 0.0),
                                           TwistStamped{}, cmd)
                  .ok);
  EXPECT_GT(cmd.twist.linear.x(), 0.0);
  // 0.05 from the goal: stopped.
  ASSERT_TRUE(bounded
                  ->computeVelocityCommand(makePose(1.95, 1.0, 0.0),
                                           TwistStamped{}, cmd)
                  .ok);
  EXPECT_NEAR(cmd.twist.linear.x(), 0.0, 1e-12);
}

TEST(CostmapMppiControllerTest, closedLoopReachesGoalAroundWall) {
  // Kinematic closed loop: the commanded twist integrates the pose, the
  // steer-lag model included via the controller's own estimate. The path
  // detours around a wall (as the planner would produce); the run must
  // reach the goal without ever touching a lethal cell.
  auto wrapper = makeWrapper(200, 0.05, [](costmap_2d::Costmap2D& m) {
    // Wall at x=5.0m, 3 cells thick, spanning y in [3.0, 7.25].
    for (unsigned int y = 60; y <= 145; ++y) {
      m.setCost(99, y, costmap_2d::LETHAL_OBSTACLE);
      m.setCost(100, y, costmap_2d::LETHAL_OBSTACLE);
      m.setCost(101, y, costmap_2d::LETHAL_OBSTACLE);
    }
  });
  ayanami::Config cfg("");
  cfg.overrideParam<int>("test_mppi", "batch", 600);
  auto controller = makeController(wrapper, std::move(cfg));
  // Rounded corners (as a planner/smoother would emit) so the tracking
  // problem stays inside the chassis curvature.
  std::vector<Eigen::Vector2d> waypoints;
  const auto rounded = [&](Eigen::Vector2d corner, Eigen::Vector2d before,
                           Eigen::Vector2d after, double radius) {
    const Eigen::Vector2d in =
        corner - radius * (corner - before).normalized();
    const Eigen::Vector2d out =
        corner + radius * (after - corner).normalized();
    for (int s = 0; s <= 4; ++s) {
      const double angle =
          std::atan2((corner - in).y(), (corner - in).x()) +
          (std::atan2((out - corner).y(), (out - corner).x()) -
           std::atan2((corner - in).y(), (corner - in).x())) *
              s / 4.0;
      waypoints.emplace_back(in.x() + radius * std::cos(angle) -
                                 radius * std::cos(std::atan2(
                                     (corner - in).y(), (corner - in).x())),
                             in.y() + radius * std::sin(angle));
    }
    (void)out;
  };
  (void)rounded;
  waypoints = {{1.0, 5.0}, {4.0, 5.0}, {4.1, 5.4}, {4.3, 5.8}, {4.4, 6.4},
               {4.4, 7.4}, {4.5, 7.8}, {4.9, 8.0}, {5.3, 7.9}, {5.6, 7.6},
               {5.7, 7.2}, {5.7, 6.4}, {5.6, 5.8}, {5.5, 5.4}, {5.6, 5.2},
               {6.0, 5.0}, {9.0, 5.0}};
  ASSERT_TRUE(controller->setPlan(makePathPoints(waypoints)));

  Eigen::Vector2d pose(1.0, 5.0);
  double yaw = 0.0;
  const double dt = 0.05;
  const Eigen::Vector2d goal(9.0, 5.0);
  for (int cycle = 0; cycle < 800; ++cycle) {
    TwistStamped cmd;
    const ResultT<void> result = controller->computeVelocityCommand(
        makePose(pose.x(), pose.y(), yaw), TwistStamped{}, cmd);
    ASSERT_TRUE(result.ok) << result.message << " cycle " << cycle;
    // Integrate the body twist (kinematic chassis with steer lag already
    // modeled inside the controller).
    const double vx = cmd.twist.linear.x();
    const double vy = cmd.twist.linear.y();
    const double wz = cmd.twist.angular.z();
    pose.x() += (std::cos(yaw) * vx - std::sin(yaw) * vy) * dt;
    pose.y() += (std::sin(yaw) * vx + std::cos(yaw) * vy) * dt;
    yaw += wz * dt;
    // No lethal collision along the run.
    unsigned int mx = 0;
    unsigned int my = 0;
    const auto* grid = wrapper->get().getCostmap();
    if (grid->worldToMap(pose.x(), pose.y(), mx, my)) {
      ASSERT_LT(grid->getCost(mx, my),
                costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
          << "collided at cycle " << cycle << " pose=(" << pose.x() << ", "
          << pose.y() << ")";
    }
    if ((pose - goal).norm() < 0.2) {
      break;
    }
  }
  EXPECT_LT((pose - goal).norm(), 0.2)
      << "final pose=(" << pose.x() << ", " << pose.y() << ")";
}

TEST(CostmapMppiControllerTest, hModeGateSwitchesAndTracks) {
  // With h_mode forced on: good tracking in free space -> 3D; a large
  // tracking error -> back to 4D. The gate is observable through
  // lastSolveMode().
  auto wrapper = makeWrapper(40, 0.5);
  ayanami::Config cfg("");
  cfg.overrideParam<bool>("test_mppi", "h_mode", true);
  auto controller = makeController(wrapper, std::move(cfg));
  ASSERT_TRUE(controller->setPlan(
      makePathPoints({{1.0, 1.0}, {9.0, 1.0}})));
  TwistStamped cmd;
  for (int cycle = 0; cycle < 8; ++cycle) {
    ASSERT_TRUE(controller
                    ->computeVelocityCommand(makePose(2.0, 1.0, 0.0),
                                             TwistStamped{}, cmd)
                    .ok);
  }
  // Robot exactly on path, heading along it, no obstacle -> 3D.
  EXPECT_EQ(controller->lastSolveMode(),
            ayanami::modules::mppi::SolveMode::Twist3D);
  // Teleported 2 m off path -> tracking error breaks the gate -> 4D.
  ASSERT_TRUE(controller
                  ->computeVelocityCommand(makePose(2.0, 3.0, 0.0),
                                           TwistStamped{}, cmd)
                  .ok);
  EXPECT_EQ(controller->lastSolveMode(),
            ayanami::modules::mppi::SolveMode::Swerve4D);
  // Back on path -> 3D again.
  ASSERT_TRUE(controller
                  ->computeVelocityCommand(makePose(3.0, 1.0, 0.0),
                                           TwistStamped{}, cmd)
                  .ok);
  EXPECT_EQ(controller->lastSolveMode(),
            ayanami::modules::mppi::SolveMode::Twist3D);
}

TEST(CostmapMppiControllerTest, swerveTelemetryMatchesCommandTwist) {
  auto wrapper = makeWrapper(40, 0.5);
  auto controller = makeController(wrapper);
  ASSERT_TRUE(controller->setPlan(
      makePathPoints({{1.0, 1.0}, {9.0, 1.0}})));
  ayanami::messages::SwerveCommandsStamped received;
  bool received_any = false;
  ayanami::CallbackSubscription sub =
      ayanami::interfaces::NavigationCallbacks::on_swerve_command.subscribe(
          [&](const ayanami::messages::SwerveCommandsStamped& stamped) {
            received = stamped;
            received_any = true;
          });
  TwistStamped cmd;
  ASSERT_TRUE(controller
                  ->computeVelocityCommand(makePose(1.0, 1.0, 0.0),
                                           TwistStamped{}, cmd)
                  .ok);
  ASSERT_TRUE(received_any);
  EXPECT_EQ(received.header.frame_id, "map");
  // The four wheel commands must reproduce the published twist through
  // the diagonal pair (the same inverse the chassis controller runs).
  const auto geometry = ayanami::modules::swerve::SwerveGeometry{
      0.2, 0.2, 0.2, 0.2, 0.076};
  const double a = received.wheels[0].rotor * geometry.tire_radius *
                   std::cos(received.wheels[0].steer);
  const double b = received.wheels[0].rotor * geometry.tire_radius *
                   std::sin(received.wheels[0].steer);
  const double c = received.wheels[3].rotor * geometry.tire_radius *
                   std::cos(received.wheels[3].steer);
  const double d = received.wheels[3].rotor * geometry.tire_radius *
                   std::sin(received.wheels[3].steer);
  EXPECT_NEAR(0.5 * (a + c), cmd.twist.linear.x(), 1e-6);
  EXPECT_NEAR(0.5 * (b + d), cmd.twist.linear.y(), 1e-6);
}

TEST(CostmapMppiControllerTest, moduleFactoryCreatesController) {
  ASSERT_FALSE(g_module_so.empty());
  auto controller =
      std::dynamic_pointer_cast<ayanami::modules::CostmapMppiController>(
          ayanami::ExtensionModule::loadModule(g_module_so));
  ASSERT_NE(controller, nullptr);
}

TEST(CostmapMppiControllerTest, solvePerformanceUnderBudget) {
  // Full-size map + production batch: the solve must fit the 100 Hz
  // budget (p95 < 8 ms).
  auto wrapper = makeWrapper(200, 0.05);
  // Production batch (the makeController default trims to 200 for the
  // behavioral tests; the benchmark restores the full 600).
  ayanami::Config cfg("");
  cfg.overrideParam<int>("test_mppi", "batch", 600);
  auto bench = makeController(wrapper, std::move(cfg));
  ASSERT_TRUE(bench->setPlan(makePathPoints({{1.0, 5.0}, {9.0, 5.0}})));
  double x = 1.0;
  std::vector<double> durations;
  for (int cycle = 0; cycle < 50; ++cycle) {
    TwistStamped cmd;
    const auto t0 = std::chrono::steady_clock::now();
    const ResultT<void> result =
        bench->computeVelocityCommand(makePose(x, 5.0, 0.0), TwistStamped{},
                                      cmd);
    const auto t1 = std::chrono::steady_clock::now();
    ASSERT_TRUE(result.ok) << result.message;
    durations.push_back(
        std::chrono::duration<double, std::milli>(t1 - t0).count());
    x += cmd.twist.linear.x() * 0.05;
  }
  std::sort(durations.begin(), durations.end());
  const double p50 = durations[durations.size() / 2];
  const double p95 = durations[static_cast<std::size_t>(0.95 *
                                                        (durations.size() -
                                                         1))];
  std::cout << "[perf] solve p50=" << p50 << "ms p95=" << p95 << "ms"
            << std::endl;
  EXPECT_LT(p95, 8.0) << "p50=" << p50 << "ms p95=" << p95 << "ms";
}

}  // namespace

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  if (argc > 1) {
    g_module_so = argv[1];
  }
  return RUN_ALL_TESTS();
}
