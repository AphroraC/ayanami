// conversions.hpp — nav_2d_utils/conversions.hpp 的 ayanami 化 (Eigen 类型
// ↔ 2D 类型)。tf2::getYaw / orientationAroundZAxis 换数学实现, 语义一致。
#pragma once

#include <Eigen/Geometry>

#include "ayanami/messages/geometry_msgs.hpp"

#include "dwb_types.hpp"

namespace dwb_core {
namespace conversions {

/// @brief 等价 tf2::getYaw (四元数 → 偏航)。
inline double getYaw(const Eigen::Quaterniond& q) {
  // tf2::getYaw 语义: atan2(2(wz + xy), 1 - 2(y^2 + z^2))
  return std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                    1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}

/// @brief 等价 nav2_util::geometry_utils::orientationAroundZAxis。
inline Eigen::Quaterniond orientationAroundZAxis(double angle) {
  return Eigen::Quaterniond(Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()));
}

/// @brief 等价 nav_2d_utils::twist2Dto3D。
inline ayanami::geometry_msgs::Twist twist2Dto3D(
    const nav_2d_msgs::Twist2D& cmd_vel_2d) {
  ayanami::geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x() = cmd_vel_2d.x;
  cmd_vel.linear.y() = cmd_vel_2d.y;
  cmd_vel.angular.z() = cmd_vel_2d.theta;
  return cmd_vel;
}

/// @brief 等价 nav_2d_utils::twist3Dto2D。
inline nav_2d_msgs::Twist2D twist3Dto2D(
    const ayanami::geometry_msgs::Twist& cmd_vel) {
  nav_2d_msgs::Twist2D cmd_vel_2d;
  cmd_vel_2d.x = cmd_vel.linear.x();
  cmd_vel_2d.y = cmd_vel.linear.y();
  cmd_vel_2d.theta = cmd_vel.angular.z();
  return cmd_vel_2d;
}

/// @brief 等价 nav_2d_utils::poseStampedToPose2D。
inline nav_2d_msgs::Pose2DStamped poseStampedToPose2D(
    const ayanami::geometry_msgs::PoseStamped& pose) {
  nav_2d_msgs::Pose2DStamped pose2d;
  pose2d.frame_id = pose.header.frame_id;
  pose2d.stamp = pose.header.stamp;
  pose2d.pose.x = pose.pose.translation().x();
  pose2d.pose.y = pose.pose.translation().y();
  const Eigen::Quaterniond q(pose.pose.linear());
  pose2d.pose.theta = getYaw(q);
  return pose2d;
}

/// @brief 等价 nav_2d_utils::poseToPose2D (Isometry3d 版)。
inline geometry_msgs::Pose2D poseToPose2D(
    const Eigen::Isometry3d& pose) {
  geometry_msgs::Pose2D pose2d;
  pose2d.x = pose.translation().x();
  pose2d.y = pose.translation().y();
  const Eigen::Quaterniond q(pose.linear());
  pose2d.theta = getYaw(q);
  return pose2d;
}

/// @brief 等价 nav_2d_utils::pose2DToPose (返回 Isometry3d)。
inline Eigen::Isometry3d pose2DToPose(const geometry_msgs::Pose2D& pose2d) {
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(pose2d.x, pose2d.y, 0.0);
  pose.linear() = orientationAroundZAxis(pose2d.theta).toRotationMatrix();
  return pose;
}

/// @brief 等价 nav_2d_utils::pose2DToPoseStamped。
inline ayanami::geometry_msgs::PoseStamped pose2DToPoseStamped(
    const nav_2d_msgs::Pose2DStamped& pose2d) {
  ayanami::geometry_msgs::PoseStamped pose;
  pose.header.frame_id = pose2d.frame_id;
  pose.header.stamp = pose2d.stamp;
  pose.pose = pose2DToPose(pose2d.pose);
  return pose;
}

/// @brief 等价 nav_2d_utils::pose2DToPoseStamped (显式 frame/stamp 版)。
inline ayanami::geometry_msgs::PoseStamped pose2DToPoseStamped(
    const geometry_msgs::Pose2D& pose2d, const std::string& frame,
    const ayanami::Timestamp& stamp) {
  ayanami::geometry_msgs::PoseStamped pose;
  pose.header.frame_id = frame;
  pose.header.stamp = stamp;
  pose.pose = pose2DToPose(pose2d);
  return pose;
}

/// @brief 等价 nav_2d_utils::pathToPath2D。
inline nav_2d_msgs::Path2D pathToPath2D(
    const ayanami::nav_msgs::Path& path) {
  nav_2d_msgs::Path2D path2d;
  path2d.frame_id = path.header.frame_id;
  path2d.stamp = path.header.stamp;
  for (const auto& pose : path.poses) {
    path2d.poses.push_back(poseToPose2D(pose.pose));
  }
  return path2d;
}

/// @brief 等价 nav_2d_utils::pathToPath。
inline ayanami::nav_msgs::Path pathToPath(const nav_2d_msgs::Path2D& path2d) {
  ayanami::nav_msgs::Path path;
  path.header.frame_id = path2d.frame_id;
  path.header.stamp = path2d.stamp;
  path.poses.resize(path2d.poses.size());
  for (unsigned int i = 0; i < path2d.poses.size(); i++) {
    path.poses[i] = pose2DToPoseStamped(path2d.poses[i], path2d.frame_id,
                                        path2d.stamp);
  }
  return path;
}

}  // namespace conversions
}  // namespace dwb_core
