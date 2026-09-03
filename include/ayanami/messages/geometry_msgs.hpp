#pragma once

#include "ayanami/messages/timestamp.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <cstdint>
#include <memory>
#include <string>

namespace ayanami {
namespace geometry_msgs {

/**
 * @brief 自研 std_msgs/Header 等价物, 布局对齐 ROS2:
 *        {builtin_interfaces/Time stamp, string frame_id} (ROS2 无 seq)。
 *
 * 帧名唯一来源为配置里的 robot_frame (spec Global Constraints)，核心内部不做 TF。
 * stamp 为中间件无关 Timestamp (默认 latest 哨兵, 与旧行为 "无时间信息 → latest"
 * 对齐)。
 */
struct Header {
  ayanami::Timestamp stamp;
  std::string frame_id;
  using Ptr = std::shared_ptr<Header>;
  using ConstPtr = std::shared_ptr<const Header>;
};

// Eigen 内联别名：保持 Eigen 原生语义，不做 ROS 风格 Polar 化。
using Quaternion = Eigen::Quaterniond;
using Vector3 = Eigen::Vector3d;
using Point = Eigen::Vector3d;

/**
 * @brief geometry_msgs/Pose 等价物, 布局对齐 ROS2:
 *        {Point position; Quaternion orientation}, 用 Eigen 类型存储。
 *
 * 与 Eigen::Isometry3d 互相转换：fromIsometry() 构造，toIsometry() 提取。
 */
struct Pose {
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};

  Eigen::Isometry3d toIsometry() const {
    Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
    iso.translation() = position;
    iso.linear() = orientation.toRotationMatrix();
    return iso;
  }

  static Pose fromIsometry(const Eigen::Isometry3d& iso) {
    Pose p;
    p.position = iso.translation();
    p.orientation = Eigen::Quaterniond(iso.linear());
    return p;
  }

  using Ptr = std::shared_ptr<Pose>;
  using ConstPtr = std::shared_ptr<const Pose>;
};

/**
 * @brief 速度 (体坐标系，linear 前向 / angular 偏航为主)。
 */
struct Twist {
  Eigen::Vector3d linear = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular = Eigen::Vector3d::Zero();
  using Ptr = std::shared_ptr<Twist>;
  using ConstPtr = std::shared_ptr<const Twist>;
};

/**
 * @brief 带 frame 的位姿, pose 布局对齐 ROS2 geometry_msgs/Pose。
 *
 * 与 Eigen::Isometry3d 互相转换：fromIsometry() 构造，toIsometry() 提取。
 */
struct PoseStamped {
  Header header;
  Pose pose;

  static PoseStamped fromIsometry(const std::string& frame,
                                  const Eigen::Isometry3d& iso) {
    PoseStamped p;
    p.header.frame_id = frame;
    p.pose = Pose::fromIsometry(iso);
    return p;
  }
  Eigen::Isometry3d toIsometry() const { return pose.toIsometry(); }

  using Ptr = std::shared_ptr<PoseStamped>;
  using ConstPtr = std::shared_ptr<const PoseStamped>;
};

/**
 * @brief 带 frame 的点。
 */
struct PointStamped {
  Header header;
  Eigen::Vector3d point = Eigen::Vector3d::Zero();
  using Ptr = std::shared_ptr<PointStamped>;
  using ConstPtr = std::shared_ptr<const PointStamped>;
};

/**
 * @brief 带 frame 的速度 (等价 geometry_msgs/TwistStamped)。
 */
struct TwistStamped {
  Header header;
  Twist twist;
  using Ptr = std::shared_ptr<TwistStamped>;
  using ConstPtr = std::shared_ptr<const TwistStamped>;
};

} // namespace geometry_msgs
} // namespace ayanami
