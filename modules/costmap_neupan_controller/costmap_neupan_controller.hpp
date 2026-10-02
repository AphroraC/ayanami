#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <neupan/neupan_planner.hpp>

#include <ayanami/costmap/components/costmap_map_context.hpp>
#include <ayanami/costmap/core/costmap_controller.hpp>
#include <ayanami/costmap/core/costmap_snapshot.hpp>
#include <ayanami/messages/geometry_msgs.hpp>
#include <ayanami/messages/nav_msgs.hpp>
#include <ayanami/types.hpp>

namespace ayanami {
namespace modules {

/**
 * @brief NeuPAN local controller module (costmap family).
 *
 * Receding-horizon end-to-end planner-controller (DUNE obstacle encoder +
 * NRMP QP, vendored under packages/neupan, GPL-3.0 with ayanami). Runs
 * inline on the control thread at controller_frequency; per cycle it feeds
 * the LOCAL obstacle PRESENCE mask (raw marks before inflation, read as a
 * const snapshot from the map context) as obstacle points plus the current
 * map-frame pose into NeuPANPlanner::forward and publishes the resulting
 * twist.
 *
 * Heading is NOT actuated by NeuPAN's omni model (the B-matrix theta row is
 * zero by construction): this module owns the body heading through a small
 * outer loop that aligns theta with the lookahead reference point's line
 * of sight.
 *
 * Arrival is owned by the execution layer's goal checker as everywhere
 * else: NeuPAN's internal arrive/stop flags only zero the twist (and are
 * logged); param.arrive_threshold must not exceed the checker's xy
 * tolerance or the robot zeros out short of acceptance (warn-only
 * contract, same as the DWB precedent).
 */
class CostmapNeupanController final : public ayanami::costmap::CostmapController {
 public:
  using Ptr = std::shared_ptr<CostmapNeupanController>;

  CostmapNeupanController();
  ~CostmapNeupanController() override = default;

  /// @brief Parameter-loading seam (ExtensionModule): reads the module json
  /// section and builds the NeuPANPlanner (the planner depends on no map).
  void configureParameters(const ayanami::ModuleContext& context) override;

  /// @brief costmap family injection contract: keeps the map context for
  /// the per-cycle presence reads. The algorithm never writes the map.
  void configure(const ModuleContext& context,
                 ayanami::costmap::CostmapMapContext::ConstPtr map) override;

  ayanami::ResultT<void> computeVelocityCommand(
      const geometry_msgs::PoseStamped& pose,
      const geometry_msgs::TwistStamped& velocity,
      geometry_msgs::TwistStamped& cmd) override;

  bool setPlan(const nav_msgs::Path& plan) override;

  bool cancel() override;

  /// @brief Arrival contract (warn-only clamp advisory, see class comment).
  void setGoalTolerances(const ayanami::PoseTolerance& tolerance) override;

  /// @brief Cycle-period consistency check (warn-only): step_time is the
  /// horizon discretization, the control period is the call cadence; a
  /// large mismatch means the nominal warm start is stale each cycle.
  void setCyclePeriod(double seconds) override;

 private:
  struct CostmapNeupanControllerParam {
    // ---- planner core (NeuPANPlanner::Config fields) ----
    int receding = 10;
    double step_time = 0.1;
    double ref_speed = 2.0;
    double collision_threshold = 0.3;
    double length = 0.5;
    double width = 0.5;
    /// u = (v, phi): linear speed bound and velocity-heading bound.
    double max_speed_v = 2.0;
    double max_speed_phi = 1.57;
    double max_acce_v = 4.0;
    double max_acce_phi = 4.0;
    std::string kinematics = "omni";
    std::string dune_checkpoint = "packages/neupan/models/diff_sentry.bin";
    // ---- initial path ----
    double ipath_interval = 0.1;
    double arrive_threshold = 0.1;
    int ind_range = 10;
    // ---- PAN / NRMP ----
    int iter_num = 2;
    int dune_max_num = 200;
    int nrmp_max_num = 10;
    double iter_threshold = 0.01;
    double q_s = 0.1;
    double p_u = 2.0;
    double eta = 15.0;
    double d_min = 0.1;
    double d_max = 1.0;
    double ro_obs = 50.0;
    double bk = 0.0;
    // ---- OSQP ----
    double osqp_eps_abs = 1e-4;
    double osqp_eps_rel = 1e-4;
    int osqp_max_iter = 1000;
    double osqp_time_limit = 0.02;
    bool osqp_polish = true;
    bool osqp_verbose = false;
    // ---- obstacle feed ----
    /// Cap on the obstacle points handed to DUNE per cycle (the mask's
    /// nonzero cells, stride-downsampled to this count).
    int feed_max_points = 200;
    // ---- heading outer loop (theta is ours, not NeuPAN's) ----
    double heading_kp = 2.5;
    double heading_deadband = 0.05;
    double max_angular_z = 3.0;
    /// Lookahead into ref_s for the line-of-sight heading reference
    /// (columns of step_time apart; 3 = 0.3 s ahead at the default grid).
    int heading_lookahead = 3;
  };

  /// @brief Build the planner from the loaded params (also used by the
  /// tolerance-driven rebuild in setGoalTolerances).
  void buildPlanner();

  /// @brief Collect map-frame obstacle points from the local presence mask
  /// (nonzero cell centers, TF to the pose frame, stride downsample).
  void collectObstaclePoints(const std::string& target_frame);

  /// @brief Outer-loop yaw rate toward the lookahead reference point.
  double headingRate(const neupan::Mat3X& ref_s, double x, double y,
                     double yaw) const;

  CostmapNeupanControllerParam param;
  std::unique_ptr<neupan::NeuPANPlanner> planner;
  ayanami::costmap::CostmapMapContext::ConstPtr map_context;

  /// @brief Per-cycle scratch (reused, no per-call allocation): the
  /// presence snapshot carrier and the (2, N) point matrix.
  ayanami::costmap::CostmapSnapshot presence_snapshot;
  neupan::Mat2X obstacle_points;

  /// @brief Last pose (map frame) -- replaceInitialPath's re-projection
  /// state between setPlan calls.
  neupan::Vec3 last_state{0.0, 0.0, 0.0};
  bool has_state = false;

  /// @brief Effective arrive threshold applied to the built planner
  /// (min of the param and the injected checker tolerance).
  double effective_arrive_threshold = 0.0;
  std::optional<double> injected_arrive_tolerance;

  /// @brief One-shot flags (avoid log spam).
  bool tolerance_warned = false;
  bool period_warned = false;

  /// @brief Telemetry throttle timestamps.
  std::chrono::steady_clock::time_point last_telemetry_at{};

  std::atomic<bool> canceled{false};
};

}  // namespace modules
}  // namespace ayanami
