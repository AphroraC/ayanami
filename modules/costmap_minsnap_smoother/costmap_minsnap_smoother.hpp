#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <ayanami/costmap/core/costmap_snapshot.hpp>
#include <ayanami/costmap/core/costmap_smoother.hpp>
#include <ayanami/costmap/core/fused_map_view.hpp>

#include "minsnap_solver.hpp"
#include "square_corridor.hpp"

namespace ayanami {
namespace modules {

/**
 * @brief Minimum-snap path smoother over a square safety-corridor
 * (CostmapSmoother family; tdt-nav-kit MinimumSnap semantics).
 *
 * The input waypoint polyline is time-allocated (trapezoidal), each
 * waypoint becomes a per-axis polynomial segment, and the minimum-snap
 * quadratic program is solved with the CLOSED-FORM equality backend by
 * default (zero external dependencies; every waypoint pinned). When the
 * module is compiled against an external OSQP/OsqpEigen install
 * (AYANAMI_MINSNAP_HAS_OSQP), backend = "osqp_corridor" constrains each
 * waypoint into its free square corridor instead (tdt OSQPCorridor).
 *
 * Both backends share the tdt collision-repair iteration: a round that
 * produces colliding segments either inserts the midpoint of every
 * colliding segment (and splits its time) or shrinks the corridor /
 * compresses the time of the colliding segment, bounded by
 * collision_iterations and the smoothPath WHOLE-CALL time budget
 * (max_duration; the initial corridor build is inside the budget, and a new
 * repair round never starts past the deadline).
 *
 * Map access discipline (family read contract): the global grid is a held
 * copy refreshed lazily by generation; the local overlay is one short-lock
 * snapshot per call; all corridor/collision queries run on this immutable
 * pair through FusedMapView -- the map writer is never delayed by
 * smoothing.
 *
 * Graceful degradation: solver failure,
 * exhausted iterations, or residual collisions roll the path back to the
 * original and still report SUCCESS -- smoothing is an enhancement, and a
 * pure smoothing artifact must not enter the BT recovery ladder.
 */
class CostmapMinSnapSmoother final : public ayanami::costmap::CostmapSmoother {
 public:
  CostmapMinSnapSmoother() = default;
  virtual ~CostmapMinSnapSmoother() = default;

  /// @brief Parameter-loading seam (ExtensionModule): called by the loader
  /// right after a successful load, and by configure() for callers that
  /// build the object directly.
  void configureParameters(const ayanami::ModuleContext& context) override;

  void configure(const ModuleContext& context,
                 ayanami::costmap::CostmapMapContext::ConstPtr map) override;

  /// @brief In-place smoothing; restores the original path on failure.
  ResultT<void> smoothPath(ayanami::nav_msgs::Path& path,
                           double max_duration) override;

  bool cancel() override;

 private:
  struct CostmapMinSnapSmootherParam {
    /// @brief Polynomial coefficient count per segment (tdt setOrder).
    int polynomial_order = 6;
    /// @brief Highest constrained derivative; 3 = minimum jerk.
    int derivative_order = 3;
    /// @brief Trapezoidal time-allocation bounds (m/s, m/s^2).
    double max_speed = 3.0;
    double max_acc = 1.0;
    /// @brief Square-corridor expansion range (m, OSQP backend).
    double max_corridor_range = 2.5;
    /// @brief Corridor shrink (m, OSQP backend).
    double corridor_shrink = 0.0;
    /// @brief Collision-repair rounds (tdt setCollisionCheckIter).
    int collision_iterations = 6;
    /// @brief Per-segment normalized time (numerical conditioning).
    bool normalized_time = true;
    /// @brief "closed_form" (default) or "osqp_corridor".
    std::string backend = "closed_form";
    /// @brief Waypoint cap: longer inputs are uniformly subsampled so the
    /// dense-cell output of the astar planner stays tractable (the SDF
    /// planner's simplified output typically needs no subsampling).
    int max_waypoints = 40;
  };

  /// @brief One collision-repair iteration pipeline shared by both
  /// backends. Returns true on a collision-free result.
  bool solveIterated(minsnap::SolveProblem& problem,
                     std::vector<corridor::Box>& corridor, bool use_osqp,
                     const std::function<bool(double, double)>& world_blocked,
                     double cell_size,
                     const std::optional<std::chrono::steady_clock::time_point>& deadline,
                     std::vector<Eigen::Vector2d>& samples_out);

  CostmapMinSnapSmootherParam param;

  ayanami::costmap::CostmapMapContext::ConstPtr map_context;

  /// @brief Held copy of the (static) global master grid (generation-gated).
  ayanami::costmap::CostmapSnapshot global_copy;
  std::uint64_t global_copy_generation{0};
  bool have_global_copy{false};
  /// @brief Reused local-snapshot buffer (one short-lock copy per call).
  ayanami::costmap::CostmapSnapshot local_snapshot;

  std::atomic<bool> canceled{false};
};

}  // namespace modules
}  // namespace ayanami
