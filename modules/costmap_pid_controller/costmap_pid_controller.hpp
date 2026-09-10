#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include <ayanami/costmap/core/costmap_controller.hpp>
#include <ayanami/costmap/components/costmap_map_context.hpp>
#include <ayanami/messages/geometry_msgs.hpp>
#include <ayanami/messages/nav_msgs.hpp>

namespace ayanami {
namespace modules {

/**
 * @brief 3-axis PID local controller module (controller costmap family).
 *
 * Pure path tracker: it only reads the robot pose + path; the algorithm itself does not read the map. Still, per the strict
 * costmap family constraint it derives from costmap::CostmapController and implements configure(context, map) +
 * computeVelocityCommand (the costmap host only accepts Costmap* types). Rewritten with reference to the
 * _legacy pid_controller.
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
class CostmapPidController final : public ayanami::costmap::CostmapController {
 public:
  using Ptr = std::shared_ptr<CostmapPidController>;

  CostmapPidController();

  virtual ~CostmapPidController() = default;

  /// @brief costmap family injection contract. The algorithm does not read the map and only holds the map module pointer;
  /// parameters are read from the ModuleContext-injected own configuration.
  /// @brief Parameter-loading seam (ExtensionModule): called by the loader
  /// right after a successful load, and by configure() for callers that build
  /// the object directly.
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
  /// checker. Treated as an upper bound on the "stop translating and align
  /// heading" distance (see goalToleranceXy).
  void setGoalTolerances(const ayanami::PoseTolerance& tolerance) override;

 private:
  struct CostmapPidControllerParam {
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
    double dist_decay = 2.0;
    double goal_tolerance = 0.1;
    double slide_window = 1.0;
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

  CostmapPidControllerParam param;
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
  uint64_t plan_signature = 0;
  ayanami::costmap::CostmapMapContext::ConstPtr map_context;

  std::atomic<bool> canceled{false};
};

}  // namespace modules
}  // namespace ayanami
