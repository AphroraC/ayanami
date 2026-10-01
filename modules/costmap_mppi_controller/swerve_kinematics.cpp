#include "swerve_kinematics.hpp"

#include <algorithm>

namespace ayanami {
namespace modules {
namespace swerve {

namespace {
constexpr double k_two_pi = 6.283185307179586476925286766559;
}

BodyTwist control4dToTwist(const Control4D& control,
                           const SwerveGeometry& geometry) {
  const double a = control.fl_vel * std::cos(control.fl_steer);
  const double b = control.fl_vel * std::sin(control.fl_steer);
  const double c = control.rr_vel * std::cos(control.rr_steer);
  const double d = control.rr_vel * std::sin(control.rr_steer);
  BodyTwist twist;
  twist.vx = 0.5 * (a + c);
  twist.vy = 0.5 * (b + d);
  // Wheel angular positions about the body z axis: fl at lateral +d_l,
  // rr at lateral -d_r (paper formulas; a square chassis has both > 0).
  const double w_l = (a - twist.vx) / (-geometry.d_l);
  const double w_r = (c - twist.vx) / (+geometry.d_r);
  twist.omega = 0.5 * (w_l + w_r);
  return twist;
}

Wheels8D twistToWheels8D(const BodyTwist& twist,
                         const SwerveGeometry& geometry) {
  Wheels8D wheels;
  const double px[4] = {geometry.l_f, geometry.l_f, -geometry.l_r,
                        -geometry.l_r};
  const double py[4] = {geometry.d_l, -geometry.d_r, geometry.d_l,
                        -geometry.d_r};
  for (int i = 0; i < 4; ++i) {
    const double vx_wheel = twist.vx - py[i] * twist.omega;
    const double vy_wheel = twist.vy + px[i] * twist.omega;
    wheels.steer[i] = std::atan2(vy_wheel, vx_wheel);
    wheels.rotor[i] = std::hypot(vx_wheel, vy_wheel) / geometry.tire_radius;
  }
  return wheels;
}

double approachSteer(double current, double target, double max_delta) {
  double diff = std::remainder(target - current, k_two_pi);
  diff = std::clamp(diff, -max_delta, max_delta);
  return std::remainder(current + diff, k_two_pi);
}

Control4D clampControl(const Control4D& control, const ControlLimits& limits) {
  Control4D clamped;
  clamped.fl_steer = std::clamp(control.fl_steer, -limits.max_steer_angle,
                                limits.max_steer_angle);
  clamped.rr_steer = std::clamp(control.rr_steer, -limits.max_steer_angle,
                                limits.max_steer_angle);
  clamped.fl_vel = std::clamp(control.fl_vel, 0.0, limits.max_wheel_speed);
  clamped.rr_vel = std::clamp(control.rr_vel, 0.0, limits.max_wheel_speed);
  return clamped;
}

RolloutState rolloutStep(const RolloutState& state, const Control4D& target,
                         const SwerveGeometry& geometry,
                         const ControlLimits& limits, double dt,
                         double omega_override, BodyTwist& twist_out) {
  RolloutState next;
  next.fl_steer =
      approachSteer(state.fl_steer, target.fl_steer, limits.max_steer_rate * dt);
  next.rr_steer =
      approachSteer(state.rr_steer, target.rr_steer, limits.max_steer_rate * dt);
  const Control4D executed{next.fl_steer, next.rr_steer, target.fl_vel,
                           target.rr_vel};
  twist_out = control4dToTwist(executed, geometry);
  if (!std::isnan(omega_override)) {
    twist_out.omega = omega_override;
  }
  const double cos_yaw = std::cos(state.yaw);
  const double sin_yaw = std::sin(state.yaw);
  next.x = state.x + (cos_yaw * twist_out.vx - sin_yaw * twist_out.vy) * dt;
  next.y = state.y + (sin_yaw * twist_out.vx + cos_yaw * twist_out.vy) * dt;
  next.yaw = std::remainder(state.yaw + twist_out.omega * dt, k_two_pi);
  return next;
}

}  // namespace swerve
}  // namespace modules
}  // namespace ayanami
