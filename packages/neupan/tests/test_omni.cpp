// Omni-kinematics tests for the vendored neupan port (stage 2 of the
// ayanami migration plan, logs/neupan-controller-migration-plan.md).
//
// 1. Formula-level: A/B/C of linear_omni_model vs hand-derived expectations
//    (Python robot.py L304-313 is the reference).
// 2. Propagate: theta carries over; position integrates (v, phi).
// 3. End-to-end behavior with the diff_default.bin checkpoint (same
//    rectangle G/h, so the DUNE weights are geometry-compatible):
//    - straight run: x advances, theta stays, |vy| ~ 0
//    - obstacle ahead: lateral deviation happens -- this is the regression
//      for the QP sparsity fix (the B(r,1) velocity-heading column MUST
//      reach the constraint matrix; without it omni cannot sidestep).
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <neupan/neupan_planner.hpp>
#include <neupan/robot.hpp>

namespace {

constexpr double kDt = 0.1;
constexpr int kReceding = 10;
// Matches models/diff_default.bin (replay_planner.yaml robot block).
constexpr double kLength = 1.6;
constexpr double kWidth = 2.0;

neupan::Vec3 propagate(const neupan::Vec3& s, const neupan::Vec2& action,
                       double dt) {
  return s + dt * neupan::Vec3(action(0), action(1), 0.0);
}

neupan::NeuPANPlanner makeOmniPlanner(double max_phi) {
  neupan::NeuPANPlanner::Config c;
  c.kinematics = neupan::Kinematics::Omni;
  c.receding = kReceding;
  c.step_time = kDt;
  c.ref_speed = 1.0;
  c.collision_threshold = 0.3;
  c.max_speed = neupan::Vec2(2.0, max_phi);
  c.max_acce = neupan::Vec2(4.0, 4.0);
  c.length = kLength;
  c.width = kWidth;
  c.ipath.interval = 0.1;
  c.ipath.arrive_threshold = 0.15;
  c.pan.iter_num = 2;
  c.pan.dune_max_num = 100;
  c.adjust.max_num = 10;
  c.adjust.q_s = neupan::Vec3(0.1, 0.1, 0.1);
  c.adjust.p_u = 2.0;
  c.adjust.eta = 15.0;
  c.adjust.d_min = 0.1;
  c.adjust.d_max = 1.0;
  c.adjust.ro_obs = 50.0;
  c.adjust.bk = 0.0;
  c.dune_checkpoint = std::string(NEUPAN_MODEL_DIR) + "/diff_default.bin";
  return neupan::NeuPANPlanner(c);
}

}  // namespace

TEST(OmniKinematics, LinearizeMatchesPythonFormula) {
  const neupan::Robot robot = neupan::Robot::omniRectangle(
      kReceding, kDt, neupan::Vec2(2.0, 1.0), neupan::Vec2(4.0, 4.0), kLength,
      kWidth);
  ASSERT_EQ(robot.kinematics, neupan::Kinematics::Omni);
  ASSERT_EQ(robot.G.rows(), 4);  // rectangle: DUNE checkpoint compatible

  const neupan::Vec3 nom_s(1.0, 2.0, 0.7);
  const neupan::Vec2 nom_u(2.0, 0.5);  // v = 2, phi = 0.5
  neupan::Mat33 A;
  neupan::Mat32 B;
  neupan::Vec3 C;
  robot.linearize(nom_s, nom_u, A, B, C);

  const double v = nom_u(0);
  const double psi = nom_u(1);
  const double c = std::cos(psi);
  const double s = std::sin(psi);

  EXPECT_TRUE(A.isApprox(neupan::Mat33::Identity()));

  neupan::Mat32 Bexp;
  Bexp << c * kDt, -v * s * kDt,  //
      s * kDt, v * c * kDt,       //
      0.0, 0.0;
  EXPECT_TRUE(B.isApprox(Bexp)) << "B actual:\n" << B;

  neupan::Vec3 Cexp(psi * v * s * kDt, -psi * v * c * kDt, 0.0);
  EXPECT_TRUE(C.isApprox(Cexp, 1e-12)) << "C actual: " << C.transpose();

  // The linearization must be exact at the operating point:
  // f(nom) == A*nom_s + B*nom_u + C (consistency of the A/B/C triple).
  const neupan::Vec3 f = robot.propagate(nom_s, nom_u);
  EXPECT_TRUE(
      f.isApprox(A * nom_s + B * nom_u + C, 1e-12))
      << "f: " << f.transpose()
      << " vs A*s+B*u+C: " << (A * nom_s + B * nom_u + C).transpose();
}

TEST(OmniKinematics, PropagateLeavesThetaUntouched) {
  const neupan::Robot robot = neupan::Robot::omniRectangle(
      kReceding, kDt, neupan::Vec2(2.0, 1.0), neupan::Vec2(4.0, 4.0), kLength,
      kWidth);
  const neupan::Vec3 s0(0.0, 0.0, 1.234);
  const neupan::Vec2 u(1.0, M_PI / 2.0);  // drive along +y
  const neupan::Vec3 s1 = robot.propagate(s0, u);
  EXPECT_NEAR(s1(0), 0.0, 1e-12);
  EXPECT_NEAR(s1(1), kDt, 1e-12);
  EXPECT_DOUBLE_EQ(s1(2), 1.234);
}

TEST(OmniPlanner, StraightRunTracksReferenceWithoutHeadingDrift) {
  neupan::NeuPANPlanner planner = makeOmniPlanner(1.57);
  planner.setWaypoints({neupan::Vec3(0.0, 0.0, 0.0),
                        neupan::Vec3(4.0, 0.0, 0.0)});

  neupan::Vec3 state(0.0, 0.0, 0.0);
  double min_abs_vy = std::numeric_limits<double>::max();
  bool solved_any = false;
  for (int frame = 0; frame < 80; ++frame) {
    neupan::NeuPANPlanner::Info info;
    const neupan::Mat2X no_points;
    const neupan::Vec2 action = planner.forward(state, no_points, info);
    if (info.arrive) break;
    ASSERT_TRUE(info.solved) << "frame " << frame;
    solved_any = true;
    ASSERT_LE(std::abs(action(0)), 2.0 + 1e-9);  // |vx| within speed box
    min_abs_vy = std::min(min_abs_vy, std::abs(action(1)));
    state = propagate(state, action, kDt);
    if (state(0) > 3.5) break;
  }
  ASSERT_TRUE(solved_any);
  EXPECT_GT(state(0), 2.0) << "robot should advance along +x, got "
                           << state.transpose();
  EXPECT_NEAR(state(2), 0.0, 1e-9) << "theta must stay unactuated";
  EXPECT_LT(min_abs_vy, 0.05) << "straight run should not sidestep";
}

TEST(OmniPlanner, DiagonalReferenceDrivesLaterally) {
  // The phi channel's functional proof: without the B(r,1) column reaching
  // the dynamics rows, omni could not produce ANY lateral velocity. A
  // diagonal reference therefore must show |vy| well above zero while theta
  // stays untouched.
  neupan::NeuPANPlanner planner = makeOmniPlanner(1.57);
  planner.setWaypoints(
      {neupan::Vec3(0.0, 0.0, 0.0), neupan::Vec3(2.0, 1.5, 0.0)});

  neupan::Vec3 state(0.0, 0.0, 0.0);
  double max_abs_vy = 0.0;
  for (int frame = 0; frame < 60; ++frame) {
    neupan::NeuPANPlanner::Info info;
    const neupan::Mat2X no_points;
    const neupan::Vec2 action = planner.forward(state, no_points, info);
    if (info.arrive) break;
    ASSERT_TRUE(info.solved) << "frame " << frame;
    max_abs_vy = std::max(max_abs_vy, std::abs(action(1)));
    state = propagate(state, action, kDt);
  }
  EXPECT_GT(max_abs_vy, 0.3)
      << "tracking a diagonal needs lateral velocity; max |vy| was "
      << max_abs_vy;
  EXPECT_NEAR(state(2), 0.0, 1e-9) << "theta must stay unactuated";
  EXPECT_GT(state(1), 0.8) << "robot should climb in y, at "
                           << state.transpose();
}

TEST(OmniPlanner, ObstacleAheadDeceleratesAndHoldsClearance) {
  // Behavior smoke for avoidance: a point on the path 1.5 m ahead. The QP
  // must keep the solve feasible, slow down before the point and never
  // penetrate the footprint. (Which escape direction the (v, phi)
  // parameterization picks -- slow down, back off, sidestep -- is an
  // upstream tuning property, not asserted here.)
  neupan::NeuPANPlanner planner = makeOmniPlanner(1.57);
  planner.setWaypoints({neupan::Vec3(0.0, 0.0, 0.0),
                        neupan::Vec3(5.0, 0.0, 0.0)});

  neupan::Vec3 state(0.0, 0.0, 0.0);
  double final_vx = 0.0;
  double worst_distance = std::numeric_limits<double>::max();
  for (int frame = 0; frame < 120; ++frame) {
    neupan::NeuPANPlanner::Info info;
    neupan::Mat2X points(2, 1);
    points << 1.5, 0.0;
    const neupan::Vec2 action = planner.forward(state, points, info);
    if (info.arrive) break;
    ASSERT_TRUE(info.solved) << "frame " << frame;
    ASSERT_FALSE(info.stop) << "frame " << frame
                            << " (stop means the footprint already grazed)";
    if (info.min_distance < std::numeric_limits<double>::infinity())
      worst_distance = std::min(worst_distance, info.min_distance);
    final_vx = action(0);
    state = propagate(state, action, kDt);
    if (frame > 20)
      ASSERT_LE(state(0), 1.45)
          << "the footprint front edge (x+0.8) must not pass the point";
  }
  EXPECT_LT(final_vx, 0.5)
      << "the robot should have slowed before the point, vx was " << final_vx;
}
