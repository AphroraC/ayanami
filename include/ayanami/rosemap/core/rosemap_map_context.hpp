#pragma once

#include <atomic>
#include <memory>
#include <string>

#include <Eigen/Geometry>

#include "ayanami/abstract/components/robot_state_controller.hpp"
#include "ayanami/map/abstract_map_context.hpp"
#include "ayanami/messages/geometry_msgs.hpp"
#include "ayanami/rosemap/core/rosemap_command.hpp"
#include "ayanami/rosemap/core/rosemap_query.hpp"
#include "ayanami/rosemap/core/rosemap_snapshot.hpp"

namespace ayanami {
namespace rosemap {

/// @brief The rosemap family "map module": a rolling 2D signed-distance
/// field published as immutable snapshots (rose_navigation RoseMap concept,
/// rebuilt on the ayanami map-family abstraction).
///
/// Ownership and holding discipline:
/// - Ownership: the host (RosemapNavigation / RosemapNavigationServer) holds
///   RosemapMapContext::Ptr; family modules (planner/smoother/controller/
///   recovery) hold a ConstPtr via configure(context, map) and never modify
///   the field.
/// - Map reads: `currentSnapshot()` hands out the immutable snapshot
///   shared_ptr (lock-free atomic load; hold it for a whole search or
///   optimization run -- the writer builds the NEXT snapshot in parallel and
///   never mutates a published one). `queryRosemap(wx, wy)` is the one-shot
///   interpolated query (distance + analytic gradient, family interpolation
///   contract). There is deliberately NO reader lock and no CostmapReader-
///   style callback view: the snapshot swap already provides strictly
///   consistent reads.
/// - Freshness: `current()` reports snapshot existence + optional age
///   gating (field.stale_timeout; 0 = no age gate, the execution-layer
///   safetyCheck consumes this).
/// - Path validity: `isPathValid` is a sparse probe (from the point nearest
///   the robot, mirroring the costmap family) that fails a point only when
///   it lies INSIDE an obstacle (distance <= path_margin); points outside
///   the rolling window pass -- the window is a local product, and far path
///   segments are simply unobserved, not blocked.
/// - All TF queries go through the RobotStateController global singleton
///   (the framework's one transform surface).
class RosemapMapContext : public map::AbstractMapContext,
                          public std::enable_shared_from_this<RosemapMapContext> {
 public:
  using Ptr = std::shared_ptr<RosemapMapContext>;
  using ConstPtr = std::shared_ptr<const RosemapMapContext>;

  RosemapMapContext() = default;

  std::string mapFamily() const override { return "rosemap"; }
  map::MapCommandPort::Ptr commandPort() const override { return command_port; }

  // ---- Writer interface (RosemapNavigationServer only) ----

  /// @brief Publish a freshly built snapshot (atomic shared_ptr swap; the
  /// first publication also flips current() to true).
  void publishSnapshot(std::shared_ptr<const RosemapSnapshot> snapshot) {
    std::atomic_store_explicit(&current_snapshot_, std::move(snapshot),
                               std::memory_order_release);
  }

  void setFrames(std::string field_frame, std::string robot_frame) {
    this->field_frame = std::move(field_frame);
    this->robot_frame = std::move(robot_frame);
  }

  void setCommandPort(map::MapCommandPort::Ptr port) {
    command_port = std::move(port);
  }

  /// @brief Age gate for the execution-layer safetyCheck: 0 disables age
  /// gating (current once any snapshot exists).
  void setStaleTimeout(double seconds) { stale_timeout = seconds; }

  // ---- frames / freshness ----
  const std::string& fieldFrame() const { return field_frame; }
  const std::string& robotFrame() const { return robot_frame; }

  /// @brief True when a snapshot exists and (with a positive stale timeout)
  /// it is fresher than the timeout.
  bool current() const {
    const auto snapshot = currentSnapshot();
    if (!snapshot) {
      return false;
    }
    if (stale_timeout <= 0.0) {
      return true;
    }
    const double age = std::chrono::duration<double>(
                           std::chrono::steady_clock::now() -
                           snapshot->received)
                           .count();
    return age <= stale_timeout;
  }

  /// @brief Age of the current snapshot in seconds (+infinity when absent).
  double fieldAgeSeconds() const {
    const auto snapshot = currentSnapshot();
    if (!snapshot) {
      return std::numeric_limits<double>::infinity();
    }
    return std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                         snapshot->received)
        .count();
  }

  // ---- Reader interface ----

  /// @brief The immutable current snapshot (lock-free). Hold it for the
  /// duration of a search/optimization: the writer never mutates a
  /// published snapshot, so a held snapshot stays byte-stable.
  std::shared_ptr<const RosemapSnapshot> currentSnapshot() const {
    return std::atomic_load_explicit(&current_snapshot_,
                                     std::memory_order_acquire);
  }

  /// @brief One-shot interpolated query at a world point (field frame):
  /// distance + analytic gradient via the family interpolation contract.
  RosemapQueryResult queryRosemap(double wx, double wy) const {
    const auto snapshot = currentSnapshot();
    if (!snapshot || !snapshot->valid()) {
      RosemapQueryResult unavailable;
      unavailable.status = RosemapQueryStatus::kOutOfField;
      return unavailable;
    }
    return sampleDistance(*snapshot, wx, wy);
  }

  /// @brief Resolve the robot pose in the field frame via the
  /// RobotStateController singleton (latest-time semantics).
  bool robotPose(ayanami::geometry_msgs::PoseStamped& pose) const {
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    if (!ayanami::components::RobotStateController::instance().lookupTransform(
            field_frame, robot_frame, ayanami::Timestamp::latest(), transform,
            0.0)) {
      return false;
    }
    pose = geometry_msgs::PoseStamped::fromIsometry(field_frame, transform);
    return true;
  }

  /// @brief field <- pose-frame transform for consumers that receive paths
  /// in another frame (latest-time; false when unavailable).
  bool fieldFromFrame(const std::string& frame_id,
                      Eigen::Isometry3d& field_from_frame) const {
    if (frame_id.empty() || frame_id == field_frame) {
      field_from_frame = Eigen::Isometry3d::Identity();
      return true;
    }
    return ayanami::components::RobotStateController::instance()
        .lookupTransform(field_frame, frame_id, ayanami::Timestamp::latest(),
                         field_from_frame, 0.0);
  }

  /// @brief Path validity: a sparse probe starting from the point nearest
  /// the robot (traversed segments may sit inside fresh obstacles and are a
  /// false-negative source, mirroring the costmap family). A point fails
  /// only when it lies inside an obstacle (distance <= path_margin); points
  /// outside the rolling window pass (unobserved, not blocked). Degrades to
  /// a non-empty check when no snapshot exists yet.
  bool isPathValid(const ayanami::nav_msgs::Path& path) const {
    if (path.poses.empty()) {
      return false;
    }
    const auto snapshot = currentSnapshot();
    if (!snapshot || !snapshot->valid()) {
      return true;
    }
    // Paths arrive in the BT/global frame; the field may live in another
    // frame (field_frame). Transform once, before probing.
    Eigen::Isometry3d field_from_path = Eigen::Isometry3d::Identity();
    const std::string& path_frame = path.header.frame_id;
    if (!path_frame.empty() && path_frame != field_frame) {
      if (!ayanami::components::RobotStateController::instance()
               .lookupTransform(field_frame, path_frame,
                                ayanami::Timestamp::latest(),
                                field_from_path, 0.0)) {
        // No transform: cannot verify -- conservatively valid (the planner
        // that produced the path already gated on the field).
        return true;
      }
    }
    ayanami::geometry_msgs::PoseStamped robot;
    const bool have_robot = robotPose(robot);
    std::size_t start_index = 0;
    if (have_robot) {
      double best = std::numeric_limits<double>::max();
      for (std::size_t i = 0; i < path.poses.size(); ++i) {
        const Eigen::Vector3d point =
            field_from_path * path.poses[i].pose.position;
        const double d = (point - robot.pose.position).norm();
        if (d < best) {
          best = d;
          start_index = i;
        }
      }
    }
    for (std::size_t i = start_index; i < path.poses.size(); ++i) {
      const Eigen::Vector3d point =
          field_from_path * path.poses[i].pose.position;
      const RosemapQueryResult query =
          sampleDistance(*snapshot, point.x(), point.y());
      if (query.status != RosemapQueryStatus::kOk) {
        continue;  // outside the window: unobserved, passes
      }
      if (query.distance <= path_margin) {
        return false;
      }
    }
    return true;
  }

  /// @brief Obstacle-clearance margin for isPathValid (meters; a probed
  /// point with distance <= margin reads as blocked). Config-owned
  /// (field.path_margin), default 0 = only interior points fail.
  void setPathMargin(double margin) { path_margin = margin; }

  /// @brief Const owner for family modules (configure injection).
  ConstPtr getSharedConstPtr() const { return shared_from_this(); }

 private:
  std::shared_ptr<const RosemapSnapshot> current_snapshot_;
  std::string field_frame{"map"};
  std::string robot_frame{"base_link"};
  map::MapCommandPort::Ptr command_port;
  double stale_timeout{0.0};
  double path_margin{0.0};
};

}  // namespace rosemap
}  // namespace ayanami
