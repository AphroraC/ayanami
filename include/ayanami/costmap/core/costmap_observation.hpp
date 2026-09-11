#pragma once

#include <string>
#include <vector>

#include <Eigen/Geometry>

namespace ayanami {
namespace costmap {

/**
 * @brief One observation source's runtime configuration (nav2
 * observation_sources semantics). The config lives with the CONSUMING
 * obstacle layer (the layer json's per-source child blocks), not with the
 * server: marking/clearing policy, hit windows and sensor liveness are
 * layer-owned filtering parameters.
 */
struct ObservationSourceConfig {
  enum class Kind { PointCloud, LaserScan };

  std::string name;
  Kind kind{Kind::LaserScan};
  /// @brief false = the source never inserts obstacle marks (clear-only).
  bool marking{true};
  /// @brief false = the source never raytraces / clears (mark-only).
  bool clearing{true};
  /// @brief Hit z window: points outside [min, max] are ignored.
  double min_obstacle_height{0.0};
  double max_obstacle_height{1.0};
  /// @brief Marking range window: hits closer than min / farther than max
  /// (from the sensor origin) are ignored.
  double obstacle_min_range{0.0};
  double obstacle_max_range{3.0};
  /// @brief Raytrace range window: clearing rays shorter than min are
  /// skipped, endpoints beyond max are clamped to max.
  double raytrace_min_range{0.0};
  double raytrace_max_range{3.5};
  /// @brief Sensor liveness: when no observation arrives for this many
  /// seconds the map is marked not-current (0 = check disabled). Consumed
  /// by the map-writer loop and surfaced through CostmapWrapper::isCurrent
  /// -> execution safetyCheck -> BT recovery.
  double expected_update_rate{0.0};
};

/**
 * @brief One buffered observation handed to a consuming obstacle layer by
 * the sole map writer. The writer resolved the stamp-time sensor transform
 * into the TARGET map's frame; the layer owns the filtering (height /
 * range windows), the mark/raytrace application and the previous-window
 * clearing against that data.
 */
struct MapObservation {
  /// @brief The layer's own source config the server matched by kind.
  const ObservationSourceConfig* source{nullptr};
  /// @brief Sensor origin pose in the target map's frame.
  Eigen::Isometry3d t_frame_sensor{Eigen::Isometry3d::Identity()};
  /// @brief Sensor-frame hit points (z KEPT for the map-frame height
  /// filter); never null.
  const std::vector<Eigen::Vector3d>* points{nullptr};
  /// @brief Sensor-frame ray endpoints (clamped to raytrace_max_range);
  /// never null.
  const std::vector<Eigen::Vector3d>* clearing_points{nullptr};
  /// @brief Robot pose (x, y, yaw) in the target map's frame (footprint
  /// clearing); has_robot_pose=false skips that pass.
  Eigen::Vector3d robot_pose_xy_yaw{Eigen::Vector3d::Zero()};
  bool has_robot_pose{false};
};

}  // namespace costmap
}  // namespace ayanami
