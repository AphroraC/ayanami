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
 * @brief Callback-scoped read access to one costmap: the map input for a single
 * algorithm call (planner/smoother/controller/recovery).
 *
 * Replaces the old CostmapSnapshot (full deep copy / COW epoch): the reader
 * holds the map's shared lock via RAII and exposes the master grid directly,
 * zero copy.
 *
 * Scope is the whole point. A reader is only ever handed to a callback by
 * CostmapMapContext::readGlobal()/readLocal(), so its lifetime is bounded by
 * that call and the lock cannot outlive the algorithm step that needed it. It is
 * move-only and non-copyable: the previous design returned this object by value
 * from lockGlobal()/lockLocal(), which made "store it in a member / capture it
 * in a lambda" perfectly legal C++ -- and silently held the map writer off until
 * the reader was destroyed (recorded as case B in the troubleshooting notes).
 * That class of bug is now unrepresentable: there is no value to keep.
 *
 * - grid(): master grid pointer; valid only while the callback runs. Do NOT copy
 *   grid data out, and do not pass the pointer beyond the callback -- if an
 *   algorithm needs the map for longer than a bounded pass, the planner's
 *   composePlanningOverlay() is the sanctioned materialization (it holds the
 *   locks only for the compose window).
 * - footprint(): the robot footprint (configure-time cache, immutable, no lock).
 * - robot_pose(): the robot pose in this map's frame, resolved BEFORE the lock
 *   was taken (never a transform lookup under a map lock), so it may be absent
 *   when TF was unavailable -- check hasRobotPose().
 * - frame() / current(): metadata (the frame the grid is expressed in, and
 *   freshness at construction).
 */
class CostmapReader {
 public:
  CostmapReader() = default;
  CostmapReader(CostmapReader&&) = default;
  CostmapReader& operator=(CostmapReader&&) = default;
  CostmapReader(const CostmapReader&) = delete;
  CostmapReader& operator=(const CostmapReader&) = delete;

  CostmapReader(std::shared_lock<std::shared_mutex> lock,
                const costmap_2d::Costmap2D* grid, std::string frame,
                const costmap_2d::Footprint* footprint,
                ayanami::geometry_msgs::PoseStamped robot_pose,
                bool has_robot_pose, bool current)
      : lock(std::move(lock)),
        costmap_grid(grid),
        map_frame(std::move(frame)),
        cached_footprint(footprint),
        resolved_robot_pose(std::move(robot_pose)),
        robot_pose_valid(has_robot_pose),
        is_current(current) {}

  /// @brief The master grid (read-only; valid only for the callback's duration).
  const costmap_2d::Costmap2D* grid() const { return costmap_grid; }

  /// @brief The frame this grid is expressed in.
  const std::string& frame() const { return map_frame; }

  /// @brief The robot footprint (configure-time cache, immutable; may be null).
  const costmap_2d::Footprint* footprint() const { return cached_footprint; }

  /// @brief The robot pose in this map's frame (resolved before the lock).
  const ayanami::geometry_msgs::PoseStamped& robot_pose() const {
    return resolved_robot_pose;
  }
  bool hasRobotPose() const { return robot_pose_valid; }

  /// @brief Map freshness at the moment the read started.
  bool current() const { return is_current; }

  bool valid() const { return costmap_grid != nullptr; }

 private:
  std::shared_lock<std::shared_mutex> lock;
  const costmap_2d::Costmap2D* costmap_grid{nullptr};
  std::string map_frame;
  const costmap_2d::Footprint* cached_footprint{nullptr};
  ayanami::geometry_msgs::PoseStamped resolved_robot_pose;
  bool robot_pose_valid{false};
  bool is_current{true};
};

}  // namespace costmap
}  // namespace ayanami
