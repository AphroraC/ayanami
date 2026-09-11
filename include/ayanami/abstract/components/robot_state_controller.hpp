#pragma once

#include "ayanami/messages/geometry_msgs.hpp"
#include "ayanami/messages/timestamp.hpp"

#include <chrono>
#include <functional>
#include <mutex>
#include <optional>
#include <shared_mutex>
#include <string>

#include <boost/circular_buffer.hpp>
#include <Eigen/Geometry>

namespace ayanami {
namespace components {

/**
 * @brief Minimal robot-state bridge (global singleton, extension).
 *
 * TF: the core never stores transforms. All TF queries delegate to an
 * injected source (TransformLookupFn) whose signatures mirror tf2's
 * BufferCore (target/source frames + stamp + timeout): ayanami_ros
 * installs a tf2_listener-backed implementation once at startup, so the
 * core gets the full tf2 capability set (interpolation, static edges,
 * stamp-based queries) while staying middleware-free. Tests install
 * stubs.
 *
 * Odometry: the only internal state besides the injected callback is a
 * boost::circular_buffer of the latest odometry messages (written by
 * ayanami_ros; read by the controller execution for the current
 * velocity).
 *
 * Hot-path discipline: frame names + velocity timeout are cached once in
 * initialize(); the hot queries never read Config.
 *
 * Singleton discipline: instance() and all method definitions live only
 * in libayanami's src/components/robot_state_controller.cpp, so the
 * ayanami_ros writer and the core readers resolve to the same instance
 * across DSOs.
 *
 * Locking: the injected lookup fn is called OUTSIDE the internal mutex
 * (it manages its own synchronization -- tf2 buffers are thread-safe);
 * the internal mutex guards only the odometry buffer and frame cache.
 */
class RobotStateController {
 public:
  static RobotStateController& instance();

  /// @brief Injected TF query source. The implementation must be safe to
  /// call from any core thread (tf2 buffers are internally synchronized).
  /// @param timeout seconds; <= 0 = fail immediately if not available
  ///        (tf2 latest-query semantics).
  using TransformLookupFn = std::function<bool(
      const std::string& target, const std::string& source,
      const Timestamp& stamp, Eigen::Isometry3d& transform, double timeout)>;

  /// @brief Install the TF source (ayanami_ros startup; tests: stubs).
  /// Calling lookupTransform/canTransform/lookupPose before installation
  /// fails gracefully (returns false).
  void setTransformSource(TransformLookupFn fn);

  /// @brief One-time initialization: cache frame names + velocity timeout
  /// (the hot path no longer reads Config).
  void initialize(const std::string& global_frame,
                  const std::string& local_frame,
                  const std::string& robot_frame, double velocity_timeout);

  // ---- tf2-aligned query surface (the core's only TF entry) ----

  /// @brief True when target<-source resolves at `stamp` (within timeout).
  bool canTransform(const std::string& target, const std::string& source,
                    const Timestamp& stamp, double timeout = 0.0) const;

  /// @brief The transform of `source` in `target` at `stamp`
  /// (tf2 lookupTransform semantics; the source resolves stamp-time ->
  /// interpolation, {0,0} = latest).
  bool lookupTransform(const std::string& target, const std::string& source,
                       const Timestamp& stamp, Eigen::Isometry3d& transform,
                       double timeout = 0.0) const;

  /// @brief Convenience wrapper: the pose of `base_frame` expressed in
  /// `target_frame` at `stamp`.
  bool lookupPose(const std::string& target_frame,
                  const std::string& base_frame, const Timestamp& stamp,
                  geometry_msgs::PoseStamped& pose,
                  double timeout = 0.0) const;

  // ---- Odometry (the only cached state) ----

  /// @brief Push one odometry sample (ayanami_ros odomCallback).
  void setOdometry(const geometry_msgs::TwistStamped& odometry);

  /// @brief Nearest odometry sample to `stamp` (latest when stamp is
  /// latest); false when the buffer is empty or the nearest sample is
  /// older than the tolerance.
  bool getOdometry(const Timestamp& stamp,
                   geometry_msgs::TwistStamped& odometry) const;

  /// @brief Latest robot state: pose = lookupPose(global<-robot, latest);
  /// velocity = latest odometry, or zero velocity when older than
  /// velocity_timeout.
  /// @return false means no pose is available (the caller treats it as
  /// poseless; the velocity is invalid).
  bool getRobotState(geometry_msgs::PoseStamped& pose,
                     geometry_msgs::TwistStamped& velocity) const;

  /// @brief Convenience accessors (testing/observation).
  geometry_msgs::PoseStamped getRobotPose() const;
  geometry_msgs::TwistStamped getRobotVelocity() const;

  /// @brief Whether the velocity is fresh (present and within the timeout window).
  bool hasFreshVelocity() const;

  /// @brief Clear the cached velocity (for tests, to simulate a timeout).
  void resetVelocity();

  /// @brief Clear the odometry buffer (tests: isolate between cases).
  void resetOdometry();

  std::string globalFrame() const;
  std::string localFrame() const;
  std::string robotFrame() const;
  double getVelocityTimeout() const;

 private:
  RobotStateController() = default;
  ~RobotStateController() = default;
  RobotStateController(const RobotStateController&) = delete;
  RobotStateController& operator=(const RobotStateController&) = delete;

  /// @brief Call with the lock held: latest velocity (zero velocity when stale).
  geometry_msgs::TwistStamped velocityLocked() const;

  mutable std::shared_mutex mutex;
  /// @brief The injected TF source; swapped only during startup (guarded
  /// by this mutex). Called OUTSIDE the lock by the query methods.
  TransformLookupFn lookup_fn;
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
