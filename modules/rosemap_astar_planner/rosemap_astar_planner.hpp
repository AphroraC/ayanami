#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <ayanami/rosemap/core/rosemap_planner.hpp>
#include <ayanami/rosemap/core/rosemap_snapshot.hpp>
#include <ayanami/types.hpp>

namespace ayanami {
namespace modules {

/**
 * @brief ESDF-native A* global planner (rosemap family; rose_navigation
 * path_search semantics with navi rog-map post-processing).
 *
 * Pure C++ (zero ROS). Per planning call:
 * 1. Hold the family's current RosemapSnapshot (immutable, no locks) and
 *    run the whole search on it.
 * 2. A* on the signed distance grid: a cell is blocked when its distance
 *    < safe_radius (ESDF-native occupancy test); the step cost carries a
 *    clearance penalty weight / (d + 0.1) that pushes paths away from
 *    obstacles; 8-connected without corner cutting.
 * 3. Start/goal repair: a blocked or out-of-field start fails
 *    (START_INVALID); a blocked goal falls back to the nearest safe cell
 *    within the configured tolerance (approximate path); an out-of-field
 *    goal is projected onto the field boundary along the start->goal
 *    direction (the real goal is appended after projection).
 * 4. Simplification: Douglas-Peucker with line-of-sight splitting (a kept
 *    segment is verified clear by sampling the distance field), then an
 *    optional greedy string-pull pass.
 * 5. Time parameterization (navi rog-map PTAllocation semantics): per-vertex
 *    turn-speed limiting (curvature decay), forward/backward velocity
 *    propagation (acceleration feasibility + goal stop), per-segment
 *    trapezoid-compatible time. Segment times land on the output path's
 *    pose stamps as RELATIVE seconds from path start (the family's path
 *    time-parameterization contract consumed by the smoother/MPC).
 */
class RosemapAstarPlanner final : public ayanami::rosemap::RosemapPlanner {
 public:
  RosemapAstarPlanner() = default;
  ~RosemapAstarPlanner() override = default;

  /// @brief Parameter-loading seam (ExtensionModule): called by the loader
  /// right after a successful load, and by configure() for callers that
  /// build the object directly.
  void configureParameters(const ayanami::ModuleContext& context) override;

  void configure(const ModuleContext& context,
                 ayanami::rosemap::RosemapMapContext::ConstPtr map) override;

  ResultT<void> makePlan(const geometry_msgs::PoseStamped& start,
                         const geometry_msgs::PoseStamped& goal,
                         double tolerance,
                         nav_msgs::Path& plan) override;

  bool cancel() override;

 private:
  struct RosemapAstarPlannerParam {
    /// @brief A cell with distance < safe_radius is blocked (meters).
    double safe_radius = 0.3;
    /// @brief Step-cost clearance penalty: weight / (d + 0.1).
    double obstacle_penalty_weight = 2.0;
    /// @brief Heuristic weight on the euclidean term (>1 de-admissifies).
    double heuristic_weight = 1.0;
    bool use_diagonal = true;
    /// @brief Douglas-Peucker deviation threshold (meters).
    double simplify_threshold = 0.15;
    /// @brief Greedy string-pull pass after DP simplification.
    bool string_pull = true;
    /// @brief Turn-speed limiting (navi LimitLocalVel semantics).
    double expected_speed = 2.0;
    double max_accel = 3.0;
    double turn_deadzone = 0.174;
    double turn_saturation = 1.57;
    double turn_min_vel = 0.3;
    double turn_decay_power = 1.6;
    /// @brief Config-owned goal tolerance (meters, nearest-safe-cell
    /// radius); the interface `tolerance` parameter is accepted for the
    /// contract and ignored, mirroring the costmap planner convention.
    double tolerance = 0.0;
    /// @brief Search budget (wall clock + iteration cap).
    double max_search_time = 0.5;
    int max_iterations = 2000000;
  };

  RosemapAstarPlannerParam param;

  ayanami::rosemap::RosemapMapContext::ConstPtr map_context;
  std::atomic<bool> canceled{false};
};

}  // namespace modules
}  // namespace ayanami
