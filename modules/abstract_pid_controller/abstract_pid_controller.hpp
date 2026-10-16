#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <vector>
#include <optional>
#include <string>

#include <ayanami/abstract/core/abstract_controller.hpp>
#include <ayanami/messages/geometry_msgs.hpp>
#include <ayanami/messages/nav_msgs.hpp>

namespace ayanami {
namespace modules {

/**
 * @brief 3-axis PID local controller module (abstract family: pure path
 * tracker, no map dependency by construction).
 *
 * Derived directly from abstract::AbstractController -- it only reads the
 * robot pose + path. Ported from the costmap_pid_controller module (same
 * parameter set, same five historical control fixes; the costmap family
 * shell and the inert map-context handle are removed).
 *
 * Wrap-up of the five historical control fixes:
 * - computeVelocityCommand forcibly zeroes cmd_vel at entry (dirty memory bug)
 * - diff mode: yaw control aims straight at the target (no sign flip); omni mode picks the nearest body-frame direction angle
 * - repeated setPlan with the same goal does not reset PID state (plan_signature comparison)
 * - linear is rate-limited separately by max_acc_x/max_decel_x; angular is limited by max_acc_theta
 * - distanceDecay decelerates near the goal; once the position enters the param.goal_tolerance XY tolerance,
 *   linear velocity is zeroed and only the angular PID aligns to the goal heading; arrival is fully decided by the execution
 *   layer goal_checker (the controller no longer declares arrival itself and takes no goal_checker parameter).
 */
class PidController final : public ayanami::abstract::AbstractController {
 public:
  using Ptr = std::shared_ptr<PidController>;

  PidController();

  virtual ~PidController() = default;

  /// @brief Parameter-loading seam (ExtensionModule): called by the loader
  /// right after a successful load; the abstract family has no further
  /// injection (this module reads no map).
  void configureParameters(const ayanami::ModuleContext& context) override;

  ResultT<void> computeVelocityCommand(
      const geometry_msgs::PoseStamped& pose,
      const geometry_msgs::TwistStamped& velocity,
      geometry_msgs::TwistStamped& cmd) override;

  bool setPlan(const nav_msgs::Path& plan) override;

  bool cancel() override;

  /// @brief Arrival contract pushed by the execution layer from the goal
  /// checker. Treated as an upper bound on the "stop translating and align
  /// heading" distance (see goalToleranceXy).
  void setGoalTolerances(const ayanami::PoseTolerance& tolerance) override;

  /// @brief Cycle-period contract (AbstractController): the execution pushes
  /// 1/controller_frequency before each run; the PID's integral/derivative
  /// and acceleration limiting run on this dt. Falls back to
  /// param.default_dt when not injected (standalone/test use).
  void setCyclePeriod(double seconds) override;

 private:
  struct PidControllerParam {
    double max_vel_x = 2.0;
    double max_vel_theta = 1.0;
    double max_acc_x = 1.0;
    double max_decel_x = 1.0;
    double max_acc_theta = 1.0;
    double kp_v = 1.0;
    double ki_v = 0.0;
    double kd_v = 0.0;
    double kp_w = 1.0;
    double ki_w = 0.0;
    double kd_w = 0.0;
    double goal_tolerance = 0.1;
    double slide_window = 1.0;
    // Fallback dt for standalone/test use (no execution layer attached); in
    // navigation the injected cycle period (setCyclePeriod) wins.
    double default_dt = 0.05;
    double heading_deadband = 0.08;
    double rotate_in_place_angle = 0.45;
    std::string mode = "diff";  // "omni" | "diff"
  };

  // Single-axis PID: holds integral/last_error (reset is triggered uniformly by resetTrackingState).
  struct PidAxis {
    double kp = 1.0;
    double ki = 0.0;
    double kd = 0.0;
    double i_max = 1.0;
    double integral = 0.0;
    double last_error = 0.0;
    bool has_last_error = false;

    void reset();
    double compute(double error, double dt);
  };

  void resetTrackingState();
  /// @brief Effective XY bound for stopping linear velocity: the tighter of
  /// param.goal_tolerance and the injected checker tolerance.
  double goalToleranceXy() const;
  double limitAcceleration(double last_vel, double cmd_vel, double max_acc,
                           double max_decel, double dt) const;
  double distanceDecay(double distance, double target_velocity) const;

  PidControllerParam param;
  /// @brief Arrival contract injected per run by the execution layer; unset
  /// when the controller runs standalone (params alone decide the bound).
  std::optional<ayanami::PoseTolerance> goal_tolerances;
  nav_msgs::Path plan;
  geometry_msgs::PoseStamped goal;
  geometry_msgs::PoseStamped last_pose;
  PidAxis pid_x;
  PidAxis pid_y;
  PidAxis pid_w;
  double last_linear_vel = 0.0;
  double last_angular_vel = 0.0;
  size_t last_track_idx = 1;
  /// @brief Cumulative arc length of `plan` (cum_[0] = 0; rebuilt on every
  /// setPlan) and the total -- the interpolated lookahead runs in this
  /// domain so the tracking target moves CONTINUOUSLY with the robot
  /// instead of jumping between discrete path points (the jump-then-P
  /// pattern was the dominant tracking-quality limit: every index step was
  /// an output step).
  std::vector<double> plan_cum_;
  double plan_total_{0.0};
  /// @brief Arc-length interpolated lookahead point: the point at
  /// s_nearest + slide_window along the path (clamped to the goal).
  Eigen::Vector2d lookaheadPoint(const Eigen::Vector2d& robot,
                                 size_t nearest) const;
  uint64_t plan_signature = 0;
  /// @brief Injected cycle period (s); seeded from param.default_dt at
  /// configure time, overwritten by the execution's setCyclePeriod push.
  double cycle_period = 0.05;

  std::atomic<bool> canceled{false};
};

}  // namespace modules
}  // namespace ayanami
