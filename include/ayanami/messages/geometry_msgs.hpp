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
 * @brief In-house std_msgs/Header equivalent, layout aligned with ROS2:
 *        {builtin_interfaces/Time stamp, string frame_id} (ROS2 has no seq).
 *
 * The frame name's single source of truth is the configured robot_frame
 * (spec Global Constraints); the core does no TF internally.
 * stamp is a middleware-agnostic Timestamp (default latest sentinel,
 * aligned with the old "no time info -> latest" behavior).
 */
struct Header {
  ayanami::Timestamp stamp;
  std::string frame_id;
  using Ptr = std::shared_ptr<Header>;
  using ConstPtr = std::shared_ptr<const Header>;
};

// Inline Eigen aliases: keep Eigen's native semantics; no ROS-style Polar-ization.
using Quaternion = Eigen::Quaterniond;
using Vector3 = Eigen::Vector3d;
using Point = Eigen::Vector3d;

/**
 * @brief geometry_msgs/Pose equivalent, layout aligned with ROS2:
 *        {Point position; Quaternion orientation}, stored with Eigen types.
 *
 * Converts to/from Eigen::Isometry3d: fromIsometry() constructs,
 * toIsometry() extracts.
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

/// @brief Absolute-tolerance pose comparison. Eigen's isApprox is a RELATIVE
/// comparison and silently fails for near-zero vectors -- goals/poses at or
/// near the frame origin would never compare equal at any noise level. All
/// "same pose?" decisions go through here: position in meters, orientation
/// as quaternion-coefficient distance.
inline bool poseClose(const Pose& lhs, const Pose& rhs,
                      double position_tol = 1e-6,
                      double orientation_tol = 1e-6) {
  if ((lhs.position - rhs.position).cwiseAbs().maxCoeff() > position_tol) {
    return false;
  }
  return (lhs.orientation.coeffs() - rhs.orientation.coeffs())
             .cwiseAbs()
             .maxCoeff() <= orientation_tol;
}


/**
 * @brief Velocity (body frame; linear forward / angular yaw dominant).
 */
struct Twist {
  Eigen::Vector3d linear = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular = Eigen::Vector3d::Zero();
  using Ptr = std::shared_ptr<Twist>;
  using ConstPtr = std::shared_ptr<const Twist>;
};

/**
 * @brief Framed pose; the pose layout aligns with ROS2 geometry_msgs/Pose.
 *
 * Converts to/from Eigen::Isometry3d: fromIsometry() constructs, toIsometry() extracts.
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
 * @brief Framed point.
 */
struct PointStamped {
  Header header;
  Eigen::Vector3d point = Eigen::Vector3d::Zero();
  using Ptr = std::shared_ptr<PointStamped>;
  using ConstPtr = std::shared_ptr<const PointStamped>;
};

/**
 * @brief Framed velocity (equivalent to geometry_msgs/TwistStamped).
 */
struct TwistStamped {
  Header header;
  Twist twist;
  using Ptr = std::shared_ptr<TwistStamped>;
  using ConstPtr = std::shared_ptr<const TwistStamped>;
};

} // namespace geometry_msgs
} // namespace ayanami
