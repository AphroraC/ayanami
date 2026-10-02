#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <esdfmap/esdf_map.hpp>

#include <ayanami/esdfmap/core/esdfmap_smoother.hpp>
#include <ayanami/types.hpp>

namespace ayanami {
namespace modules {

/**
 * @brief MINCO minimum-snap trajectory smoother (esdfmap family).
 *
 * The esdfmap-family port of rosemap_minco_smoother (same vendor skeleton:
 * MINCO_S3NU + LBFGS, MIT-licensed Zhepei Wang headers under vendor/;
 * hardening follows the navi_minco_bit minco_optimizer semantics):
 * - Fixed segment times: the input path's stamps (relative seconds, the
 *   planner's speed-aware allocation) are kept -- the smoother refines the
 *   GEOMETRY, never the schedule.
 * - Cost = minco jerk energy + ESDF position safety (smoothed-L1 hinge on
 *   safe_dist - distance, evaluated at the control points through the
 *   package interpolation contract; a failed/out-of-field query counts as
 *   violating, never as free).
 * - Freeze mask: control points inside freeze_window seconds of the start
 *   keep their input position (reference continuity for the running
 *   controller).
 * - Wall-clock budget: max_duration doubles as the LBFGS budget via the
 *   progress callback (which also honors cancel()).
 * - Hard validation before publishing: the optimized trajectory is sampled
 *   at validation_dt; non-finite or severely overspeed states, or a
 *   collision (distance < 0), ROLL BACK to the input path and still report
 *   SUCCESS -- smoothing is an enhancement, never a prerequisite (the FSM's
 *   DegradeToRawPath fallback covers the rest).
 */
class EsdfmapMincoSmoother final : public ayanami::esdfmap::EsdfmapSmoother {
 public:
  EsdfmapMincoSmoother() = default;
  ~EsdfmapMincoSmoother() override = default;

  /// @brief Parameter-loading seam (ExtensionModule): called by the loader
  /// right after a successful load, and by configure() for callers that
  /// build the object directly.
  void configureParameters(const ayanami::ModuleContext& context) override;

  void configure(const ModuleContext& context,
                 ::esdfmap::EsdfMap::Ptr map) override;

  ResultT<void> smoothPath(nav_msgs::Path& path,
                           double max_duration) override;

  bool cancel() override;

 private:
  struct EsdfmapMincoSmootherParam {
    /// @brief Clearance target: penalty active where distance < safe_dist.
    double safe_dist = 0.35;
    /// @brief ESDF position-safety penalty weight.
    double weight_pos = 1.0e4;
    /// @brief Minco jerk-energy weights (x, y).
    double weight_smooth = 1.0e2;
    /// @brief Smoothed-L1 hinge width for the position penalty.
    double mu_l1 = 0.4;
    /// @brief Control points with stamp < freeze_window keep their input
    /// position (seconds from path start).
    double freeze_window = 0.0;
    /// @brief Output resample period (seconds) -- the dense stamped path
    /// handed to the controller.
    double output_dt = 0.05;
    /// @brief LBFGS knobs (navi minco_optimizer semantics).
    int max_iterations = 256;
    double opt_accuracy = 1e-4;
    int lbfgs_mem_size = 64;
    /// @brief Validation: speed cap reference and the severe scale
    /// (1.5x semantics from navi validateTrajectory). The cap reference is
    /// the input path's own implied peak speed.
    double severe_scale = 1.5;
    double validation_dt = 0.05;
  };

  EsdfmapMincoSmootherParam param;

  ::esdfmap::EsdfMap::Ptr esdf_map;
  std::atomic<bool> canceled{false};
};

}  // namespace modules
}  // namespace ayanami
