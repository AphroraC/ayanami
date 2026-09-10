#pragma once

#include "ayanami/interfaces/transform_store.hpp"
#include "ayanami/messages/geometry_msgs.hpp"

#include <chrono>
#include <boost/circular_buffer.hpp>
#include <Eigen/Geometry>
#include <mutex>
#include <optional>
#include <shared_mutex>
#include <string>

namespace ayanami {
namespace components {

/**
 * @brief Global singleton robot state controller (extension).
 *
 * Maintains the latest velocity (written directly by the ayanami_ros
 * odomCallback) + queries the latest pose from the fast_tf TransformStore
 * (global_frame <- robot_frame). Frame names and velocity_timeout are read
 * from Config only once in initialize() and cached; the hot
 * getRobotState() never reads Config.
 *
 * Semantics: the pose always takes the latest TF; the velocity takes the
 * latest odom, and if the last write is older than velocity_timeout it is
 * considered stale -> return zero velocity while the pose stays valid.
 *
 * Singleton discipline (same class): instance and all method definitions
 * live only in libayanami's src/components/robot_state_controller.cpp (not
 * header-inlined), so the ayanami_ros writer and the core controller
 * execution readers resolve to the same instance across DSOs.
 *
 * Thread safety: all public methods are serialized by a mutex; inside the
 * lock getRobotState only does a TF query + cached copies, never
 * triggering IO/Config.
 */
class RobotStateController {
 public:
  static RobotStateController& instance();

  /// @brief One-time initialization: cache frame names + velocity timeout
  /// (the hot path no longer reads Config).
  /// @param local_frame Frame in which the local map/local view resolves
  /// the robot pose (semantically usually the odom frame; aligned with the
  /// costmap.local_frame single source, not an independent ROS TF odom
  /// frame configuration).
  void initialize(const std::string& global_frame,
                  const std::string& local_frame,
                  const std::string& robot_frame, double velocity_timeout);

  /// @brief Update the latest velocity (direct write entry from the ayanami_ros odomCallback). Thread-safe.
  void setVelocity(const geometry_msgs::TwistStamped& velocity);

  void setOdometry(const geometry_msgs::TwistStamped& odometry);
  bool getOdometry(const ayanami::Timestamp& stamp,
                   geometry_msgs::TwistStamped& odometry) const;
  void setTransform(const std::string& parent, const std::string& child,
                    ayanami::Timestamp stamp,
                    const Eigen::Isometry3d& transform,
                    bool is_static = false);
  bool lookupTransform(const std::string& target, const std::string& source,
                       ayanami::Timestamp stamp, Eigen::Isometry3d& transform,
                       double timeout = 0.0) const;

  /// @brief Read the latest robot state: pose = TransformStore's latest
  /// global<-robot; velocity = latest velocity, or zero velocity when older
  /// than velocity_timeout (or absent).
  /// @return false means no pose is available (the caller treats it as
  /// poseless; the velocity is invalid).
  bool getRobotState(geometry_msgs::PoseStamped& pose,
                     geometry_msgs::TwistStamped& velocity) const;

  /// @brief Convenience accessors (testing/observation); semantics match the corresponding getRobotState fields.
  geometry_msgs::PoseStamped getRobotPose() const;
  geometry_msgs::TwistStamped getRobotVelocity() const;

  /// @brief Whether the velocity is fresh (present and within the timeout window).
  bool hasFreshVelocity() const;

  /// @brief Clear the cached velocity (for tests, to simulate a timeout).
  void resetVelocity();

  std::string globalFrame() const;
  std::string localFrame() const;
  std::string robotFrame() const;
  double getVelocityTimeout() const;

 private:
  RobotStateController() = default;
  ~RobotStateController() = default;
  RobotStateController(const RobotStateController&) = delete;
  RobotStateController& operator=(const RobotStateController&) = delete;

  /// @brief Call with the lock held: query the latest pose from TransformStore (global <- robot).
  bool poseFromTransformStore(geometry_msgs::PoseStamped& pose) const;
  /// @brief Call with the lock held: latest velocity (zero velocity when stale).
  geometry_msgs::TwistStamped velocityLocked() const;

  /// @brief Read-mostly guard: readers take shared_lock, writers
  /// exclusive_lock (see the class Thread safety note).
  mutable std::shared_mutex mutex;
  std::string global_frame{"map"};
  std::string local_frame{"odom"};
  std::string robot_frame{"base_link"};
  double velocity_timeout_s{0.25};
  std::optional<geometry_msgs::TwistStamped> last_velocity;
  std::chrono::steady_clock::time_point last_velocity_at{};
  boost::circular_buffer<geometry_msgs::TwistStamped> odometry_buffer{128};
  double odometry_time_tolerance_s{0.25};
};

}  // namespace components
}  // namespace ayanami
