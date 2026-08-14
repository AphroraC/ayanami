#pragma once

#include "ayanami/messages/geometry_msgs.hpp"

#include <mutex>
#include <string>

namespace ayanami {
namespace components {

/**
 * @brief 机器人位姿/速度缓存 + 坐标系名 (对标 MBF RobotStateProvider, spec §5.3)。
 *
 * 供 controller/action 层轮询读取当前机器人状态 (Pose/Twist), 帧名由
 * setFrames 注入 (全局帧 / odom / robot)。所有读写用 std::mutex 保护。
 */
class RobotStateProvider {
public:
  /// @brief 注入帧名 (全局帧、里程计帧、机器人本体帧)。
  void setFrames(const std::string& global, const std::string& odom,
                 const std::string& robot);

  /// @brief 更新机器人位姿与速度缓存 (线程安全)。
  void setRobotState(const geometry_msgs::PoseStamped& pose,
                     const geometry_msgs::TwistStamped& velocity);

  /// @brief 读取缓存的位姿; 未 setRobotState 时返回 identity (默认构造)。
  geometry_msgs::PoseStamped getRobotPose() const;

  /// @brief 读取缓存的速度; 未 setRobotState 时返回零速度 (默认构造)。
  geometry_msgs::TwistStamped getRobotVelocity() const;

  const std::string& getGlobalFrame() const;
  const std::string& getOdomFrame() const;
  const std::string& getRobotFrame() const;

private:
  mutable std::mutex mutex_;
  geometry_msgs::PoseStamped pose_;
  geometry_msgs::TwistStamped velocity_;
  std::string global_frame_, odom_frame_, robot_frame_;
};

inline void RobotStateProvider::setFrames(const std::string& global,
                                          const std::string& odom,
                                          const std::string& robot) {
  std::lock_guard<std::mutex> lock(mutex_);
  global_frame_ = global;
  odom_frame_ = odom;
  robot_frame_ = robot;
}

inline void RobotStateProvider::setRobotState(
    const geometry_msgs::PoseStamped& pose,
    const geometry_msgs::TwistStamped& velocity) {
  std::lock_guard<std::mutex> lock(mutex_);
  pose_ = pose;
  velocity_ = velocity;
}

inline geometry_msgs::PoseStamped RobotStateProvider::getRobotPose() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return pose_;
}

inline geometry_msgs::TwistStamped RobotStateProvider::getRobotVelocity() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return velocity_;
}

inline const std::string& RobotStateProvider::getGlobalFrame() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return global_frame_;
}

inline const std::string& RobotStateProvider::getOdomFrame() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return odom_frame_;
}

inline const std::string& RobotStateProvider::getRobotFrame() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return robot_frame_;
}

} // namespace components
} // namespace ayanami
