#pragma once

#include <Eigen/Dense>

#include <functional>
#include <vector>

#include "mppi_core.hpp"

namespace ayanami {
namespace modules {
namespace mppi {

/// @brief Per-solve critic context: everything the critics read besides the
/// trajectory itself. The obstacle query is a COST lookup on the fused
/// view (cost >= INSCRIBED_INFLATED_OBSTACLE blocks) -- the caller prepared
/// one short-lock snapshot pair per solve, so no live-map lock is ever
/// held inside the batch loop.
struct CriticData {
  /// @brief Dense reference path (world frame), tail is the goal.
  const std::vector<Eigen::Vector2d>* path{nullptr};
  /// @brief Robot progress on the path at solve start (index of the
  /// nearest point found by the controller this cycle).
  std::size_t path_cursor{0};
  Eigen::Vector2d goal_xy{0.0, 0.0};
  double goal_yaw{0.0};
  /// @brief Graded cost query (0..255); lethal/inscribed >= 253.
  std::function<unsigned char(double, double)> cost_at;
  /// @brief Chassis footprint in the BODY frame (empty = center-point
  /// checking). Populated with the bounding-square corners (+ center) from
  /// the swerve geometry when footprint checking is enabled; the obstacle
  /// critic transforms the offsets by each state's yaw. Consecutive
  /// footprints overlap at rollout step lengths, which is why the critic
  /// only interpolates the CENTER midpoint (thin-obstacle anti-tunneling)
  /// and skips footprint midpoints.
  std::vector<Eigen::Vector2d> footprint_offsets;
  /// @brief Twist-space limits mirrored from the navigation config (the
  /// constraint critic keeps sampled twists inside what the execution
  /// layer would clamp anyway).
  double max_vel_x{2.0};
  double max_vel_y{2.0};
  double max_vel_theta{3.0};
  /// @brief Angular-mode gates: free mode shapes heading via the goal-angle
  /// critic and penalizes useless spin via twirling; fixed (small-gyro) and
  /// zero modes disable both (their omega is not the MPPI's decision).
  bool angular_free{true};
  /// @brief Within this radius of the goal the goal-angle critic drives
  /// the final heading (meters).
  double goal_angle_radius{0.5};
  /// @brief Command deadbands (m/s, rad/s): twists inside are worthless
  /// steering jitter for a swerve chassis (steer motors would chatter).
  double deadband_xy{0.05};
  double deadband_theta{0.05};

  // Weights (tuned following nav2_mppi_controller defaults where they map).
  double w_path_follow{32.0};
  double w_goal{24.0};
  double w_goal_angle{16.0};
  double w_twirling{2.0};
  double w_constraint{4.0};
  double w_velocity_deadband{2.0};
  double w_obstacles{50.0};
};

/// @brief Score = weight * raw cost of one critic over one trajectory.
/// Implementations are pure and stateless (the per-trajectory path cursor
/// lives on the local stack).
namespace critics {

/// @brief Sum of distances from each state to the reference path with a
/// per-trajectory monotonic cursor (searches a +-window around the cursor;
/// advancing the cursor is what makes progress profitable).
double pathFollow(const Trajectory& trajectory, const CriticData& data);

/// @brief Distance from the final state to the goal -- the time-optimal
/// bias (faster trajectories end closer).
double goal(const Trajectory& trajectory, const CriticData& data);

/// @brief Final heading error, active only inside goal_angle_radius of the
/// goal (free angular mode only).
double goalAngle(const Trajectory& trajectory, const CriticData& data);

/// @brief Sum of squared omega over the executed twists (free mode only):
/// an omnidirectional chassis should translate, not dance.
double twirling(const Trajectory& trajectory, const CriticData& data);

/// @brief Quadratic penalty on twist components beyond the twist-space
/// limits (the 4D clamp bounds wheel speed; this bounds the body twist the
/// execution layer would clamp).
double constraint(const Trajectory& trajectory, const CriticData& data);

/// @brief Penalty for twists inside the command deadband: pushes the
/// solution off the micro-command zone that makes steer motors chatter.
double velocityDeadband(const Trajectory& trajectory, const CriticData& data);

/// @brief Graded obstacle cost summed over the states (center point, or
/// the MAX over the footprint points when a footprint is set); a single
/// lethal / inscribed hit adds a rejection-scale penalty (the sample is
/// effectively discarded). No cost_at query -> no-op.
double obstacles(const Trajectory& trajectory, const CriticData& data);

/// @brief Total weighted score of all enabled critics.
double totalCost(const Trajectory& trajectory, const CriticData& data);

}  // namespace critics
}  // namespace mppi
}  // namespace modules
}  // namespace ayanami
