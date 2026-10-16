#pragma once

#include <atomic>
#include <memory>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "ayanami/abstract/core/abstract_controller.hpp"
#include "ayanami/messages/geometry_msgs.hpp"
#include "ayanami/messages/nav_msgs.hpp"
#include "ayanami/types.hpp"

namespace ayanami {
namespace modules {

/**
 * @brief Iterative linear MPC local controller (abstract family, map-free),
 * a faithful port of rose_navigation's LMPC (control/lmpc.cpp).
 *
 * Algorithm (rose parity, adapted to the ayanami Path contract):
 * - Reference: rose samples a TIME-parameterized trajectory; the ayanami
 *   nav_msgs::Path carries poses only, so the plan is re-parameterized by
 *   ARC LENGTH (per-segment reference speed = stamp delta when the poses
 *   carry valid stamps, else cruise_speed). Reference points advance along
 *   the arc at the reference speed, blind-radius deduplicated (rose
 *   get_re), and past the path end the last point repeats with zero
 *   reference velocity (natural deceleration to the goal).
 * - QP (rose solve_mpc_omni verbatim): variables [x (2T) | v (2T)] over the
 *   prediction horizon; initial-state equality, discrete double-integrator
 *   dynamics equalities, speed box and dv (acceleration) box constraints;
 *   cost = position tracking (q_pos, the x0 state is fixed by the equality
 *   and unpenalized) + control effort (r_vel, dref feedforward term) +
 *   adjacent-control smoothness (rd_smooth). Solved with OSQP
 *   (OsqpEigen), warm-started across cycles with the sparse structure
 *   cached (only H/g/bounds update).
 * - Iteration: re-solve until the control sequence converges (du < 1e-4,
 *   rose's early exit) or max_iter; the framework-level deviation guard
 *   (ControllerExecution) is complemented by the module's own
 *   deviation_threshold rejection (rose rejects references > 0.5 m away
 *   and hands the task back to the planner).
 * - Output: the first control (world frame) rotated into the body frame;
 *   angular.z = fixed_angular_z when nonzero (gyro mode) else a P
 *   alignment to the path tangent.
 */
class AbstractMpcController final : public abstract::AbstractController {
 public:
  using Ptr = std::shared_ptr<AbstractMpcController>;

  AbstractMpcController();
  ~AbstractMpcController() override;

  void configureParameters(const ayanami::ModuleContext& context) override;

  ResultT<void> computeVelocityCommand(
      const geometry_msgs::PoseStamped& pose,
      const geometry_msgs::TwistStamped& velocity,
      geometry_msgs::TwistStamped& cmd) override;

  bool setPlan(const nav_msgs::Path& plan) override;

  /// @brief Arrival contract: accepted (unused -- a pure tracker; arrival
  /// stays with the goal checker).
  void setGoalTolerances(const ayanami::PoseTolerance&) override {}

  bool cancel() override;

 private:
  struct AbstractMpcControllerParam {
    double predict_dt = 0.1;        ///< MPC discretization step (s)
    int predict_steps = 30;         ///< horizon length
    int max_iter = 20;              ///< iterative re-solve cap (rose 100 +
                                    ///< wall budget; du-early-exit applies)
    double max_speed = 2.0;         ///< speed box (m/s)
    double max_accel = 1.0;         ///< dv box = max_accel * dt
    double delay_time = 0.0;        ///< reference arc advance (s)
    double blind_radius = 0.5;      ///< skip references closer than this
    double deviation_threshold = 0.5;  ///< module-level rejection (rose)
    double cruise_speed = 1.5;      ///< reference speed without stamps
    double q_pos = 15.0;            ///< position tracking weight (rose Q[0:2])
    double r_vel = 0.1;             ///< control effort weight (rose R)
    double rd_smooth = 1.0;         ///< adjacent-control smoothness (rose Rd[0])
    double angular_kp = 3.0;        ///< tangent alignment P gain
    double fixed_angular_z = 0.0;   ///< nonzero = gyro mode override
  };

  struct Impl;
  std::unique_ptr<Impl> impl;
  AbstractMpcControllerParam param;
  std::atomic<bool> canceled{false};
};

}  // namespace modules
}  // namespace ayanami
