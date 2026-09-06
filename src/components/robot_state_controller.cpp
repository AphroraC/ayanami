// RobotStateController 实现 (扩展)。
//
// 单例/方法定义只在本翻译单元 (libayanami 内一份): 不用 header-inline,
// 否则每个 .so / 编译单元各持一份 instance 的静态局部实例 —— 跨 DSO
// (ayanami_ros odomCallback 写者 vs 核心 costmap controller execution 读者)
// 静默发散。定义在此保证 ROS 写者与核心读者解析到 libayanami 内的同一实例。

#include "ayanami/abstract/components/robot_state_controller.hpp"

#include <cstdlib>

namespace ayanami {
namespace components {

RobotStateController& RobotStateController::instance() {
  static RobotStateController controller;
  return controller;
}

void RobotStateController::initialize(const std::string& global,
                                     const std::string& local,
                                     const std::string& robot,
                                     double velocity_timeout) {
  std::lock_guard<std::mutex> lock(mutex);
  global_frame = global;
  local_frame = local;
  robot_frame = robot;
  velocity_timeout_s = velocity_timeout > 0.0 ? velocity_timeout : 0.25;
  last_velocity.reset();
}

void RobotStateController::setVelocity(
    const geometry_msgs::TwistStamped& velocity) {
  std::lock_guard<std::mutex> lock(mutex);
  last_velocity = velocity;
  last_velocity_at = std::chrono::steady_clock::now();
}

void RobotStateController::setOdometry(
    const geometry_msgs::TwistStamped& odometry) {
  std::lock_guard<std::mutex> lock(mutex);
  odometry_buffer.push_back(odometry);
  last_velocity = odometry;
  last_velocity_at = std::chrono::steady_clock::now();
}

bool RobotStateController::getOdometry(
    const ayanami::Timestamp& stamp,
    geometry_msgs::TwistStamped& odometry) const {
  std::lock_guard<std::mutex> lock(mutex);
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

void RobotStateController::setTransform(
    const std::string& parent, const std::string& child,
    ayanami::Timestamp stamp, const Eigen::Isometry3d& transform,
    bool is_static) {
  std::lock_guard<std::mutex> lock(mutex);
  ayanami::interfaces::TransformStore::instance().setTransform(
      parent, child, stamp, transform, is_static);
}

bool RobotStateController::lookupTransform(
    const std::string& target, const std::string& source,
    ayanami::Timestamp stamp, Eigen::Isometry3d& transform,
    double timeout) const {
  std::lock_guard<std::mutex> lock(mutex);
  return ayanami::interfaces::TransformStore::instance().lookupTransform(
      target, source, stamp, transform, timeout);
}

bool RobotStateController::poseFromTransformStore(
    geometry_msgs::PoseStamped& pose) const {
  Eigen::Isometry3d T;
  if (!ayanami::interfaces::TransformStore::instance().lookupTransform(
          global_frame, robot_frame, ayanami::Timestamp::latest(), T)) {
    return false;
  }
  pose = geometry_msgs::PoseStamped::fromIsometry(global_frame, T);
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
  std::lock_guard<std::mutex> lock(mutex);
  if (!poseFromTransformStore(pose)) {
    return false;
  }
  velocity = velocityLocked();
  return true;
}

geometry_msgs::PoseStamped RobotStateController::getRobotPose() const {
  std::lock_guard<std::mutex> lock(mutex);
  geometry_msgs::PoseStamped pose;
  poseFromTransformStore(pose);
  return pose;
}

geometry_msgs::TwistStamped RobotStateController::getRobotVelocity() const {
  std::lock_guard<std::mutex> lock(mutex);
  return velocityLocked();
}

bool RobotStateController::hasFreshVelocity() const {
  std::lock_guard<std::mutex> lock(mutex);
  if (!last_velocity) {
    return false;
  }
  return (std::chrono::steady_clock::now() - last_velocity_at) <=
         std::chrono::duration<double>(velocity_timeout_s);
}

void RobotStateController::resetVelocity() {
  std::lock_guard<std::mutex> lock(mutex);
  last_velocity.reset();
}

std::string RobotStateController::getGlobalFrame() const {
  std::lock_guard<std::mutex> lock(mutex);
  return global_frame;
}

std::string RobotStateController::getLocalFrame() const {
  std::lock_guard<std::mutex> lock(mutex);
  return local_frame;
}

std::string RobotStateController::getRobotFrame() const {
  std::lock_guard<std::mutex> lock(mutex);
  return robot_frame;
}

double RobotStateController::getVelocityTimeout() const {
  std::lock_guard<std::mutex> lock(mutex);
  return velocity_timeout_s;
}

}  // namespace components
}  // namespace ayanami
