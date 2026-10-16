#pragma once

#include <atomic>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <random>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <ayanami/costmap/components/costmap_map_context.hpp>
#include <ayanami/costmap/core/costmap_controller.hpp>
#include <ayanami/costmap/core/costmap_snapshot.hpp>
#include <ayanami/costmap/core/fused_map_view.hpp>
#include <ayanami/interfaces/callbacks.hpp>

#include "mppi_core.hpp"
#include "mppi_critics.hpp"
#include "swerve_kinematics.hpp"

namespace ayanami {
namespace modules {

/**
 * @brief MPPI local controller in the 4D swerve control space
 * (CostmapController family).
 *
 * Reference implementations: nav2_mppi_controller (optimizer structure,
 * critic list, weighted update) and mppi_swerve_drive_ros (IROS2024 MPPI-4D
 * control parameterization and MPPI-H mode switching for a 4-wheel
 * independent drive/steer chassis).
 *
 * Control space: the diagonal wheel pair (fl_steer, rr_steer, fl_vel,
 * rr_vel) -- 4 scalars spanning the full 3-DOF twist of a rigid chassis.
 * The rollout rate-limits the steer state toward the sampled targets
 * (modeling the steer motors), so every scored trajectory is one the
 * chassis can actually execute; the FR/RL wheel commands are implied by
 * the rigid-body inverse and the OUTPUT is the body twist (vx, vy, omega)
 * -- the four-wheel inverse lives in the chassis controller, and the
 * ayanami Twist contract is unchanged.
 *
 * MPPI-H (h_mode): when the tracking error AND the heading error are both
 * inside their thresholds AND no obstacle is near, the solve samples the
 * twist space directly (3D mode -- efficient precise tracking while the
 * steer lag is negligible); every violation falls back to the 4D steer
 * space (feasible near obstacles / during reorientation). Each mode keeps
 * its own warm start; switching resets the entering mode's sequence.
 *
 * Steer-state estimate: ayanami has no wheel-joint feedback, so the
 * controller integrates its own steer state from the previous cycle's
 * command (4D: the executed steers; 3D: the steers implied by the
 * commanded twist) at the (conservative) configured steer rate; keep
 * max_steer_rate aligned with (or below) the chassis firmware.
 *
 * Map access discipline: the global grid is a held copy refreshed lazily
 * by generation; each computeVelocityCommand takes ONE short-lock local
 * snapshot and the whole MPPI batch runs on the immutable FusedMapView
 * pair -- the map writer is never delayed by a solve.
 *
 * Arrival is decided by the execution layer's goal checker. Inside
 * goal_stop_radius of the path tail the controller zeroes the translation
 * and lets the goal-angle critic rotate the body onto the goal heading
 * (fixed-spin mode never zeroes omega).
 */
class CostmapMppiController final : public ayanami::costmap::CostmapController {
 public:
  CostmapMppiController();
  ~CostmapMppiController() override;

  /// @brief Parameter-loading seam (ExtensionModule): called by the loader
  /// right after a successful load, and by configure() for callers that
  /// build the object directly.
  void configureParameters(const ayanami::ModuleContext& context) override;

  void configure(const ModuleContext& context,
                 ayanami::costmap::CostmapMapContext::ConstPtr map) override;

  ResultT<void> computeVelocityCommand(
      const geometry_msgs::PoseStamped& pose,
      const geometry_msgs::TwistStamped& velocity,
      geometry_msgs::TwistStamped& cmd) override;

  bool setPlan(const nav_msgs::Path& plan) override;

  bool cancel() override;

  /// @brief Arrival contract pushed by the execution layer from the goal
  /// checker: bounds the translation stop radius around the path tail.
  void setGoalTolerances(const ayanami::PoseTolerance& tolerance) override;

  /// @brief Cycle-period contract (AbstractController): the execution pushes
  /// 1/controller_frequency before each run. Used for the steer-rate limit
  /// (max_steer_rate * period); the rollout dt stays a pure algorithm
  /// parameter (settings_.dt) -- it sizes the prediction horizon, not the
  /// execution cadence.
  void setCyclePeriod(double seconds) override;

  /// @brief The control space the LAST solve sampled (MPPI-H telemetry).
  mppi::SolveMode lastSolveMode() const { return last_mode_; }

 private:
  struct CostmapMppiControllerParam {
    // Chassis geometry (square chassis: all four equal).
    double d_l = 0.2;
    double d_r = 0.2;
    double l_f = 0.2;
    double l_r = 0.2;
    double tire_radius = 0.076;
    // Swerve limits (keep aligned with the chassis firmware, biased low).
    double max_steer_rate = 4.0;
    double max_steer_angle = 1.6;
    double max_wheel_speed = 2.5;  // wheel linear speed (m/s)
    // MPPI solve.
    int batch = 600;
    int time_steps = 30;
    double dt = 0.05;
    double temperature = 0.3;  // RELATIVE softmax sharpness (spread-scaled)
    double sigma_steer = 0.25;
    double sigma_wheel = 0.8;
    // Noise covariance adaptation: the per-dim std of the previous
    // optimal sequence becomes the next sampling sigma, clamped to
    // [0.2, 2.0] x the base sigma.
    bool adaptive_sigma = true;
    // Twist-space limits (constraint critic + output clamp; the wheel
    // ceiling max_wheel_speed bounds the translation too).
    double max_vel_x = 2.0;
    double max_vel_y = 2.0;
    double max_vel_theta = 3.0;
    // MPPI-H mode switching (3D twist space when tracking is good and no
    // obstacle is near). DEFAULT OFF: the tight-obstacle e2e exposed
    // convergence transients in the 3D->4D handoff near walls (the cruise
    // leaves a warm-start/steer-state context the 4D solve chases); the
    // gate machinery itself is unit-tested -- enable after field tuning.
    bool h_mode = false;
    double h_dist_error = 0.3;      // m: tracking error inside -> 3D
    double h_yaw_error = 0.3;       // rad: heading error inside -> 3D
    double h_obstacle_radius = 1.0;  // m: lethal within -> always 4D
    double sigma_vx = 0.3;          // 3D sampling sigmas
    double sigma_vy = 0.3;
    double sigma_omega = 0.6;
    // Angular mode: "free" (omega from MPPI) | "fixed" (small-gyro constant
    // spin) | "zero".
    std::string angular_mode = "free";
    double fixed_angular_z = 0.0;
    // Critic weights + shaping.
    double w_path_follow = 32.0;
    double w_goal = 24.0;
    double w_goal_angle = 16.0;
    double w_twirling = 2.0;
    double w_constraint = 4.0;
    double w_velocity_deadband = 2.0;
    double w_obstacles = 50.0;
    double goal_angle_radius = 0.5;
    double deadband_xy = 0.05;
    double deadband_theta = 0.05;
    // Footprint collision checking: bounding square of the swerve geometry
    // (corners + center) instead of the bare center point.
    bool use_footprint = true;
    // Path handling: dense reference spacing + progress cursor window.
    double path_spacing = 0.1;
    double cursor_jump_distance = 1.0;
    // Arrival translation stop radius (m; bounded by the goal checker's
    // contract via setGoalTolerances).
    double goal_stop_radius = 0.15;
    // Period between cycles (s): advances the steer-state estimate and the
    // warm start consistently with the rollout model.
    double control_period = 0.05;
    int random_seed = 0;  // 0 (or negative) = entropy seed; tests pin it
  };

  /// @brief One MPPI solve from the current state; fills cmd_twist (body
  /// frame, unclamped) and advances the warm start of the active mode.
  bool solve(const geometry_msgs::PoseStamped& pose, double yaw,
             const std::function<unsigned char(double, double)>& cost_at,
             bool obstacle_nearby, swerve::BodyTwist& cmd_twist);

  /// @brief MPPI-H gate: tracking + heading errors against the path
  /// tangent, plus the lethal-proximity veto.
  mppi::SolveMode selectMode(double x, double y, double yaw,
                             const std::function<unsigned char(double, double)>&
                                 cost_at);

  /// @brief Any lethal cell within h_obstacle_radius of (x, y) (8-point
  /// ring + the pose itself).
  bool obstacleNearby(
      double x, double y,
      const std::function<unsigned char(double, double)>& cost_at) const;

  void densifyPath(const nav_msgs::Path& plan);
  std::size_t advanceCursor(double x, double y);
  void fillCriticData(mppi::CriticData& data) const;

  /// @brief Noise covariance adaptation: the per-dim std of the previous
  /// optimal sequence becomes the next sampling sigma (clamped to
  /// [0.2, 2.0] x the base sigma). Steer dims pair into one sigma (max of
  /// the two stds), wheel dims likewise.
  void adaptSigmas(const mppi::ControlSequence& sequence);
  void adaptSigmas3D(const std::vector<mppi::Control3D>& sequence);

  double sequenceStd(const std::vector<double>& values) const;

  CostmapMppiControllerParam param;

  ayanami::costmap::CostmapMapContext::ConstPtr map_context;

  // Path state (setPlan-owned; dense world-frame polyline, tail = goal).
  std::vector<Eigen::Vector2d> dense_path_;
  Eigen::Vector2d goal_xy_{0.0, 0.0};
  double goal_yaw_{0.0};
  std::size_t path_cursor_{0};
  Eigen::Vector2d path_tangent_{1.0, 0.0};

  // Swerve + MPPI state.
  /// @brief Effective per-cycle period (s) for steer-rate limiting: seeded
  /// from param.control_period, overwritten by the execution's
  /// setCyclePeriod push (1 / controller_frequency).
  double control_cycle{0.05};
  double steer_fl_{0.0};
  double steer_rr_{0.0};
  mppi::ControlSequence warm_start_4d_;
  std::vector<mppi::Control3D> warm_start_3d_;
  mppi::NoiseSigma active_sigma_{0.25, 0.8};
  mppi::NoiseSigma3D active_sigma_3d_{0.3, 0.3, 0.6};
  mppi::Sampler sampler_{0u};
  mppi::Settings settings_;
  swerve::SwerveGeometry geometry_;
  swerve::ControlLimits limits_;
  double omega_override_{std::numeric_limits<double>::quiet_NaN()};
  double goal_stop_radius_{0.15};
  mppi::SolveMode last_mode_{mppi::SolveMode::Swerve4D};

  std::optional<ayanami::PoseTolerance> goal_tolerances_;

  // Map read state (held global copy + per-cycle local snapshot).
  ayanami::costmap::CostmapSnapshot global_copy_;
  std::uint64_t global_copy_generation_{0};
  bool have_global_copy_{false};
  ayanami::costmap::CostmapSnapshot local_snapshot_;

  std::atomic<bool> canceled_{false};

  /// @brief True while the installed plan is degenerate (start == goal
  /// position): no reference is installed; compute holds zero translation
  /// and P-rotates toward the goal heading (the goal checker owns the
  /// arrival verdict).
  bool degenerate_path_{false};
  double degenerate_goal_yaw_{0.0};
};

}  // namespace modules
}  // namespace ayanami
