// RobotStateController implementation (extension).
//
// The singleton/method definitions live only in this translation unit (one
// copy inside libayanami): no header-inline, otherwise each .so / translation
// unit would hold its own static local instance of instance -- silently
// diverging across DSOs
// (the ayanami_ros odomCallback writer vs the core costmap controller
// execution readers). Defining it here guarantees the ROS writer and the
// core readers resolve to the same instance inside libayanami.
//
// TF bridge discipline: the injected lookup fn is invoked OUTSIDE the
// internal mutex (it carries its own synchronization); the mutex guards
// only the odometry buffer, the frame cache and the fn slot itself.

#include "ayanami/abstract/components/robot_state_controller.hpp"

#include <cstdlib>
#include <mutex>
#include <shared_mutex>

namespace ayanami {
namespace components {

RobotStateController& RobotStateController::instance() {
  static RobotStateController controller;
  return controller;
}

void RobotStateController::setTransformSource(TransformLookupFn fn) {
  std::unique_lock<std::shared_mutex> lock(mutex);
  lookup_fn = std::move(fn);
}

void RobotStateController::initialize(const std::string& global,
                                     const std::string& local,
                                     const std::string& robot,
                                     double velocity_timeout) {
  std::unique_lock<std::shared_mutex> lock(mutex);
  global_frame = global;
  local_frame = local;
  robot_frame = robot;
  velocity_timeout_s = velocity_timeout > 0.0 ? velocity_timeout : 0.25;
  last_velocity.reset();
}

bool RobotStateController::canTransform(const std::string& target,
                                        const std::string& source,
                                        const Timestamp& stamp,
                                        double timeout) const {
  TransformLookupFn fn;
  {
    std::shared_lock<std::shared_mutex> lock(mutex);
    fn = lookup_fn;
  }
  if (!fn) {
    return false;
  }
  Eigen::Isometry3d unused = Eigen::Isometry3d::Identity();
  return fn(target, source, stamp, unused, timeout);
}

bool RobotStateController::lookupTransform(const std::string& target,
                                           const std::string& source,
                                           const Timestamp& stamp,
                                           Eigen::Isometry3d& transform,
                                           double timeout) const {
  TransformLookupFn fn;
  {
    std::shared_lock<std::shared_mutex> lock(mutex);
    fn = lookup_fn;
  }
  if (!fn) {
    return false;
  }
  return fn(target, source, stamp, transform, timeout);
}

bool RobotStateController::lookupPose(const std::string& target_frame,
                                      const std::string& base_frame,
                                      const Timestamp& stamp,
                                      geometry_msgs::PoseStamped& pose,
                                      double timeout) const {
  Eigen::Isometry3d transform;
  if (!lookupTransform(target_frame, base_frame, stamp, transform, timeout)) {
    return false;
  }
  pose = geometry_msgs::PoseStamped::fromIsometry(target_frame, transform);
  return true;
}

void RobotStateController::setOdometry(
    const geometry_msgs::TwistStamped& odometry) {
  std::unique_lock<std::shared_mutex> lock(mutex);
  odometry_buffer.push_back(odometry);
  last_velocity = odometry;
  last_velocity_at = std::chrono::steady_clock::now();
}

bool RobotStateController::getOdometry(
    const ayanami::Timestamp& stamp,
    geometry_msgs::TwistStamped& odometry) const {
  std::shared_lock<std::shared_mutex> lock(mutex);
  if (odometry_buffer.empty()) {
    return false;
  }
  if (stamp.isLatest()) {
    odometry = odometry_buffer.back();
    return true;
  }
  const auto target = stamp.count();
  const auto* best = &odometry_buffer.front();
  auto best_distance = std::llabs(best->header.stamp.count() - target);
  for (const auto& candidate : odometry_buffer) {
    const auto distance = std::llabs(candidate.header.stamp.count() - target);
    if (distance < best_distance) {
      best = &candidate;
      best_distance = distance;
    }
  }
  if (best_distance > static_cast<long long>(odometry_time_tolerance_s * 1e9)) {
    return false;
  }
  odometry = *best;
  return true;
}

geometry_msgs::TwistStamped RobotStateController::velocityLocked() const {
  if (!last_velocity) {
    return {};
  }
  const auto age = std::chrono::steady_clock::now() - last_velocity_at;
  if (age > std::chrono::duration<double>(velocity_timeout_s)) {
    return {};
  }
  return *last_velocity;
}

bool RobotStateController::getRobotState(
    geometry_msgs::PoseStamped& pose,
    geometry_msgs::TwistStamped& velocity) const {
  if (!lookupPose(globalFrame(), robotFrame(), Timestamp::latest(), pose)) {
    return false;
  }
  std::shared_lock<std::shared_mutex> lock(mutex);
  velocity = velocityLocked();
  return true;
}

geometry_msgs::PoseStamped RobotStateController::getRobotPose() const {
  geometry_msgs::PoseStamped pose;
  lookupPose(globalFrame(), robotFrame(), Timestamp::latest(), pose);
  return pose;
}

geometry_msgs::TwistStamped RobotStateController::getRobotVelocity() const {
  std::shared_lock<std::shared_mutex> lock(mutex);
  return velocityLocked();
}

bool RobotStateController::hasFreshVelocity() const {
  std::shared_lock<std::shared_mutex> lock(mutex);
  if (!last_velocity) {
    return false;
  }
  return (std::chrono::steady_clock::now() - last_velocity_at) <=
         std::chrono::duration<double>(velocity_timeout_s);
}

void RobotStateController::resetVelocity() {
  std::unique_lock<std::shared_mutex> lock(mutex);
  last_velocity.reset();
}

void RobotStateController::resetOdometry() {
  std::unique_lock<std::shared_mutex> lock(mutex);
  odometry_buffer.clear();
  last_velocity.reset();
}

std::string RobotStateController::globalFrame() const {
  std::shared_lock<std::shared_mutex> lock(mutex);
  return global_frame;
}

std::string RobotStateController::localFrame() const {
  std::shared_lock<std::shared_mutex> lock(mutex);
  return local_frame;
}

std::string RobotStateController::robotFrame() const {
  std::shared_lock<std::shared_mutex> lock(mutex);
  return robot_frame;
}

double RobotStateController::getVelocityTimeout() const {
  std::shared_lock<std::shared_mutex> lock(mutex);
  return velocity_timeout_s;
}

}  // namespace components
}  // namespace ayanami
