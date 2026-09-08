#pragma once

#include <memory>
#include <shared_mutex>
#include <string>
#include <utility>

#include <costmap_2d/costmap_2d.hpp>

#include "ayanami/messages/geometry_msgs.hpp"

namespace ayanami {
namespace costmap {

/**
 * @brief Locked read view: the map input for a single algorithm call
 * (planner/smoother/controller/recovery).
 *
 * Replaces the old CostmapSnapshot (full deep copy / COW epoch) -- the view
 * holds the map's shared lock via RAII and the grid pointer points straight
 * at the master grid, zero copy. A single planning call is <10ms; the map
 * writer (~10Hz) waits for readers to release before writing.
 *
 * - grid(): master grid pointer, valid only within this view's lifetime
 *   (the lock is released when the view is destroyed).
 * - Red line: the snapshot IS the lock, never a copy. COW or any deep-copy
 *   snapshot (CostmapSnapshot-style) is forbidden; planner/smoother
 *   guarantee <10ms per call, so holding the shared lock for the whole
 *   call is the only sanctioned map access.
 * - footprint: this map's robot footprint (configure-time cache, immutable,
 *   no lock needed).
 * - robot_pose: the robot pose resolved at view construction (TransformStore,
 *   lock-free query).
 * - frame / current: metadata.
 */
class CostmapView {
 public:
  CostmapView() = default;
  CostmapView(CostmapView&&) = default;
  CostmapView& operator=(CostmapView&&) = default;
  CostmapView(const CostmapView&) = delete;
  CostmapView& operator=(const CostmapView&) = delete;

  CostmapView(std::shared_lock<std::shared_mutex> lock,
              const costmap_2d::Costmap2D* grid, std::string frame,
              const costmap_2d::Footprint* footprint,
              ayanami::geometry_msgs::PoseStamped robot_pose,
              bool has_robot_pose, bool current)
      : lock(std::move(lock)),
        costmap_grid(grid),
        view_frame(std::move(frame)),
        cached_footprint(footprint),
        view_robot_pose(std::move(robot_pose)),
        robot_pose_valid(has_robot_pose),
        is_current(current) {}

  /// @brief The master grid (read-only; valid only within this view's lifetime, may be null).
  const costmap_2d::Costmap2D* grid() const { return costmap_grid; }

  /// @brief The view's frame name.
  const std::string& frame() const { return view_frame; }

  /// @brief The robot footprint (configure-time cache, immutable; may be null).
  const costmap_2d::Footprint* footprint() const { return cached_footprint; }

  /// @brief The robot pose in the frame.
  const ayanami::geometry_msgs::PoseStamped& robot_pose() const {
    return view_robot_pose;
  }
  bool has_robot_pose() const { return robot_pose_valid; }

  /// @brief Map freshness at construction time.
  bool current() const { return is_current; }

  bool valid() const { return costmap_grid != nullptr; }

 private:
  std::shared_lock<std::shared_mutex> lock;
  const costmap_2d::Costmap2D* costmap_grid{nullptr};
  std::string view_frame;
  const costmap_2d::Footprint* cached_footprint{nullptr};
  ayanami::geometry_msgs::PoseStamped view_robot_pose;
  bool robot_pose_valid{false};
  bool is_current{true};
};

}  // namespace costmap
}  // namespace ayanami
