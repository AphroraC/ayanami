#pragma once

#include <cmath>
#include <vector>

namespace ayanami {
namespace modules {
namespace swerve {

/// @brief 4-wheel independent drive/steer (4WIDS) geometry: wheel group
/// positions relative to the body center -- fl = (+l_f, +d_l),
/// fr = (+l_f, -d_r), rl = (-l_r, +d_l), rr = (-l_r, -d_r). A square
/// chassis fills all four with the same half-size (mppi_swerve_drive_ros
/// target_system parameters; kept as four values so non-square frames need
/// no code change).
struct SwerveGeometry {
  double d_l{0.2};
  double d_r{0.2};
  double l_f{0.2};
  double l_r{0.2};
  double tire_radius{0.076};
};

/// @brief Control limits applied when clamping sampled controls.
struct ControlLimits {
  /// @brief Steering rate (rad/s): rollout approaches sampled steer targets
  /// at this rate -- the model of the steer motor being rate-limited.
  double max_steer_rate{4.0};
  /// @brief Steering range (rad, symmetric); a swerve wheel drives only
  /// forward (wheel speed >= 0), reverse = steer around.
  double max_steer_angle{1.6};
  /// @brief Wheel LINEAR speed ceiling (m/s); the IROS2024 4D conversion treats the wheel velocity as linear (its 8D inverse divides by the tire radius).
  double max_wheel_speed{30.0};
};

/// @brief The diagonal control pair (front-left, rear-right): 4 scalars
/// that span the full 3-DOF twist of a rigid square chassis (the IROS2024
/// MPPI-4D parameterization). FR/RL are determined by the rigid-body
/// inverse and never sampled.
struct Control4D {
  double fl_steer{0.0};
  double rr_steer{0.0};
  double fl_vel{0.0};
  double rr_vel{0.0};
};

/// @brief Body-frame planar twist (ayanami Twist convention).
struct BodyTwist {
  double vx{0.0};
  double vy{0.0};
  double omega{0.0};
};

/// @brief 4D control -> body twist (mppi_swerve_drive_ros
/// convertControlSpace4DToVxVyOmega, verbatim formulas). The two wheel
/// vectors average into the translation; the lateral offsets of the two
/// wheels turn the x-component difference into the body rate.
BodyTwist control4dToTwist(const Control4D& control,
                           const SwerveGeometry& geometry);

/// @brief All four wheel commands from the rigid-body inverse: steer_i =
/// atan2(vy + px_i*omega, vx - py_i*omega), rotor_i =
/// hypot(...)/tire_radius (fl, fr, rl, rr order). NOT used by the rollout
/// -- the diag pair drives everything; exposed for debugging/telemetry and
/// future 8D pre-command output.
struct Wheels8D {
  double steer[4]{0.0, 0.0, 0.0, 0.0};
  double rotor[4]{0.0, 0.0, 0.0, 0.0};
};
Wheels8D twistToWheels8D(const BodyTwist& twist,
                         const SwerveGeometry& geometry);

/// @brief Moves `current` toward `target` along the wrapped shortest arc,
/// at most max_delta radians (the per-step steer-rate limit).
double approachSteer(double current, double target, double max_delta);

/// @brief Clamps a sampled control into the feasible set: steer within
/// +-max_steer_angle, wheel speed within [0, max_wheel_speed].
Control4D clampControl(const Control4D& control, const ControlLimits& limits);

/// @brief Executed twist of a rollout step (the post-approach 4D converted
/// to a body twist; omega already reflects omega_override).
struct RolloutState {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double fl_steer{0.0};
  double rr_steer{0.0};
};
RolloutState rolloutStep(const RolloutState& state, const Control4D& target,
                         const SwerveGeometry& geometry,
                         const ControlLimits& limits, double dt,
                         double omega_override, BodyTwist& twist_out);

}  // namespace swerve
}  // namespace modules
}  // namespace ayanami
