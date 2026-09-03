#pragma once

#include "ayanami/messages/geometry_msgs.hpp"

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace ayanami {
namespace nav_msgs {

/**
 * @brief 地图元数据 (等价 nav_msgs/MapMetaData, 布局对齐 ROS2:
 *        map_load_time / resolution / width / height / origin)。
 */
struct MapMetaData {
  ayanami::Timestamp map_load_time;
  double resolution = 0.05;
  uint32_t width = 0;
  uint32_t height = 0;
  ayanami::geometry_msgs::Pose origin;
  using Ptr = std::shared_ptr<MapMetaData>;
  using ConstPtr = std::shared_ptr<const MapMetaData>;
};

/**
 * @brief 占据栅格 (等价 nav_msgs/OccupancyGrid)，data 是 row-major 的 int8 概率值。
 */
struct OccupancyGrid {
  ayanami::geometry_msgs::Header header;
  MapMetaData info;
  std::vector<int8_t> data;
  using Ptr = std::shared_ptr<OccupancyGrid>;
  using ConstPtr = std::shared_ptr<const OccupancyGrid>;
};

/**
 * @brief 路径 (等价 nav_msgs/Path)，poses 是带 frame 的位姿序列。
 */
struct Path {
  using poses_type = std::vector<ayanami::geometry_msgs::PoseStamped>;
  ayanami::geometry_msgs::Header header;
  poses_type poses;
  using Ptr = std::shared_ptr<Path>;
  using ConstPtr = std::shared_ptr<const Path>;
};

/**
 * @brief 位姿 + 6x6 协方差 (等价 geometry_msgs/PoseWithCovariance, row-major)。
 */
struct PoseWithCovariance {
  ayanami::geometry_msgs::Pose pose;
  std::array<double, 36> covariance{};
  using Ptr = std::shared_ptr<PoseWithCovariance>;
  using ConstPtr = std::shared_ptr<const PoseWithCovariance>;
};

/**
 * @brief 速度 + 6x6 协方差 (等价 geometry_msgs/TwistWithCovariance, row-major)。
 */
struct TwistWithCovariance {
  ayanami::geometry_msgs::Twist twist;
  std::array<double, 36> covariance{};
  using Ptr = std::shared_ptr<TwistWithCovariance>;
  using ConstPtr = std::shared_ptr<const TwistWithCovariance>;
};

/**
 * @brief 里程计 (等价 nav_msgs/Odometry, 布局对齐 ROS2:
 *        header / child_frame_id / PoseWithCovariance pose / TwistWithCovariance twist)。
 */
struct Odometry {
  ayanami::geometry_msgs::Header header;
  std::string child_frame_id;
  PoseWithCovariance pose;
  TwistWithCovariance twist;
  using Ptr = std::shared_ptr<Odometry>;
  using ConstPtr = std::shared_ptr<const Odometry>;
};

} // namespace nav_msgs
} // namespace ayanami
