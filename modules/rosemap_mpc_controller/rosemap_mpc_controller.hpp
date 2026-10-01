#pragma once

#include <atomic>
#include <cstdint>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include <ayanami/rosemap/core/rosemap_controller.hpp>
#include <ayanami/types.hpp>

namespace ayanami {
namespace modules {

/**
 * @brief Omni-base LMPC trajectory-tracking controller (rosemap family).
 *
 * Pure C++ core ported from rose_navigation control/lmpc.cpp (MIT), with
 * the RM-specific extras stripped (turtle mode / constant spin / tunnel
 * logic) and the trajectory source adapted to the ayanami path contract.
 *
 * Design points (unchanged from the reference):
 * - Zero map queries: obstacle safety lives upstream (planner/smoother +
 *   the BT's IsPathValid gate); the controller only tracks.
 * - QP: single-integrator omni model (states = positions, inputs = world
 *   frame planar velocities), condensed-by-constraint form with initial
 *   state equality + dynamics equality + speed box + adjacent-control
 *   (acceleration) box constraints. The A matrix sparsity is fixed for a
 *   constant horizon, so only Hessian/gradient/bounds update per cycle
 *   (OsqpEigen warm start across cycles).
 * - Reference: the stamped path (relative stamps, the family
 *   time-parameterization contract) is projected onto the robot's nearest
 *   sample each cycle; the horizon samples positions with velocity
 *   feedforward, skips blind_radius duplicates and freezes zero velocity
   * past the tail (approach-to-goal deceleration; ARRIVAL is decided by
   * the execution layer's goal checker, never by the controller).
 * - Deviation gate: when the robot strays beyond reference_deviations from
 *   the path, the controller returns NO_VALID_COMMAND (the execution
 *   patience turns that into BT recovery instead of blind tracking).
   * - Output: first QP control (world frame) rotated into the BODY frame
   *   (ayanami Twist convention). Angular velocity comes from angular_mode:
   *   "heading" (P-controller on the velocity-direction error, default;
   *   near the tail with collapsed translation it rotates onto the goal
   *   heading), "fixed" (constant spin, sentry small-gyro style), or "zero".
 */
class RosemapMpcController final : public ayanami::rosemap::RosemapController {
 public:
  /// @brief Ctor and dtor are defined out of line: the QpState unique_ptr
  /// member requires the complete type at both construction-failure and
  /// destruction (pimpl discipline).
  RosemapMpcController();
  ~RosemapMpcController() override;

  /// @brief Parameter-loading seam (ExtensionModule): called by the loader
  /// right after a successful load, and by configure() for callers that
  /// build the object directly.
  void configureParameters(const ayanami::ModuleContext& context) override;

  void configure(const ModuleContext& context,
                 ayanami::rosemap::RosemapMapContext::ConstPtr map) override;

  bool setPlan(const nav_msgs::Path& path) override;

  ResultT<void> computeVelocityCommand(
      const geometry_msgs::PoseStamped& pose,
      const geometry_msgs::TwistStamped& velocity,
      geometry_msgs::TwistStamped& cmd) override;

 private:
  struct RosemapMpcControllerParam {
    /// @brief Prediction horizon.
    double predict_dt = 0.05;
    int predict_steps = 20;
    /// @brief Inner re-solve iterations within one control cycle.
    int max_iter = 3;
    /// @brief Speed / acceleration limits (world frame planar).
    double max_speed = 2.5;
    double max_accel = 3.0;
    /// @brief Extra latency between planning and control (seconds).
    double delay_time = 0.0;
    /// @brief Reference points closer than this to the robot are skipped
    /// (avoids an all-duplicate horizon).
    double blind_radius = 0.05;
    /// @brief Projection gate: beyond this deviation the reference is
    /// unreliable and the controller refuses (meters).
    double reference_deviation = 0.5;
    /// @brief Tracking weights: q_position (x, y), r_control, rd_delta.
    double q_position = 150.0;
    double r_control = 1.0;
    double rd_delta = 50.0;
    /// @brief Angular mode: "heading" | "fixed" | "zero".
    std::string angular_mode = "heading";
    double angular_gain = 2.0;
    double fixed_angular_z = 0.0;
    /// @brief Endgame heading alignment ("heading" mode only): once the
    /// linear command collapses within this distance of the path tail, the
    /// body rotates onto the goal heading (meters).
    double goal_heading_radius = 0.5;
  };

  /// @brief One stamped path sample (finite-difference velocity included).
  struct Sample {
    Eigen::Vector2d position{0.0, 0.0};
    Eigen::Vector2d velocity{0.0, 0.0};
    double stamp{0.0};
  };

  RosemapMpcControllerParam param;

  ayanami::rosemap::RosemapMapContext::ConstPtr map_context;
  std::atomic<bool> canceled{false};

  /// @brief Stored reference (setPlan): stamped samples in the path frame.
  std::vector<Sample> reference;
  std::string reference_frame;
  /// @brief Goal heading from the plan tail pose (setPlan): endgame
  /// alignment target for the "heading" angular mode.
  double goal_yaw{0.0};
  /// @brief Plan identity: bumped on every setPlan so the MPC's cached
  /// solution resets across plans (navi trajectory_id gating).
  std::uint64_t plan_id{0};
  std::uint64_t applied_plan_id{0};

  // ---- OsqpEigen QP state (created lazily, warm-started across cycles) ----
  struct QpState;
  std::unique_ptr<QpState> qp;
};

}  // namespace modules
}  // namespace ayanami
