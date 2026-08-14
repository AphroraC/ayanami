#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <cstdint>
#include <memory>
#include <string>

namespace ayanami {
namespace geometry_msgs {

/**
 * @brief 自研 std_msgs::Header 等价物。
 *
 * 帧名唯一来源为配置里的 robot_frame (spec Global Constraints)，核心内部不做 TF。
 */
struct Header {
  uint32_t seq = 0;
  std::string frame_id;
  using Ptr = std::shared_ptr<Header>;
  using ConstPtr = std::shared_ptr<const Header>;
};

// Eigen 内联别名：保持 Eigen 原生语义，不做 ROS 风格 Polar 化。
using Quaternion = Eigen::Quaterniond;
using Vector3 = Eigen::Vector3d;
using Point = Eigen::Vector3d;
using Pose = Eigen::Isometry3d;

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
 * @brief 带 frame 的位姿，pose 直接是 Eigen::Isometry3d。
 *
 * 与 Eigen::Isometry3d 互相转换：fromIsometry() 构造，toIsometry() 提取。
 */
struct PoseStamped {
  Header header;
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();

  static PoseStamped fromIsometry(const std::string& frame,
                                  const Eigen::Isometry3d& iso) {
    PoseStamped p;
    p.header.frame_id = frame;
    p.pose = iso;
    return p;
  }
  Eigen::Isometry3d toIsometry() const { return pose; }

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
