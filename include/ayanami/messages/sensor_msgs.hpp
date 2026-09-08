#pragma once

#include "ayanami/messages/geometry_msgs.hpp"

#include <memory>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace ayanami {
namespace sensor_msgs {

/**
 * @brief Point cloud: frame header + PCL XYZI cloud.
 *
 * The framework's single point-cloud representation. There is no
 * PointField/blob plumbing anywhere in ayanami: producers fill the PCL
 * cloud directly (the ROS adapter converts sensor_msgs/PointCloud2 once at
 * the boundary via pcl::fromROSMsg).
 */
struct PointCloud2 {
  ayanami::geometry_msgs::Header header;
  pcl::PointCloud<pcl::PointXYZI> cloud;
  using Ptr = std::shared_ptr<PointCloud2>;
  using ConstPtr = std::shared_ptr<const PointCloud2>;
};

/**
 * @brief 2D laser scan (equivalent to sensor_msgs/LaserScan).
 */
struct LaserScan {
  ayanami::geometry_msgs::Header header;
  float angle_min = 0.0f;
  float angle_max = 0.0f;
  float angle_increment = 0.0f;
  float time_increment = 0.0f;
  float scan_time = 0.0f;
  float range_min = 0.0f;
  float range_max = 0.0f;
  std::vector<float> ranges;
  std::vector<float> intensities;
  using Ptr = std::shared_ptr<LaserScan>;
  using ConstPtr = std::shared_ptr<const LaserScan>;
};

} // namespace sensor_msgs
} // namespace ayanami
