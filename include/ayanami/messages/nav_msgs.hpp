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
 * @brief Map metadata (equivalent to nav_msgs/MapMetaData, layout aligned
 *        with ROS2: map_load_time / resolution / width / height / origin).
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
 * @brief Occupancy grid (equivalent to nav_msgs/OccupancyGrid); data is row-major int8 probability values.
 */
struct OccupancyGrid {
  ayanami::geometry_msgs::Header header;
  MapMetaData info;
  std::vector<int8_t> data;
  using Ptr = std::shared_ptr<OccupancyGrid>;
  using ConstPtr = std::shared_ptr<const OccupancyGrid>;
};

/**
 * @brief Signed distance field snapshot (ayanami-native; no ROS nav_msgs
 *        equivalent). Rendered by the ROS bridge as a PointCloud2 whose
 *        intensity carries the distance (rose_navigation esdf_out style).
 *
 * distance: row-major signed meters, positive = free-space distance to the
 * nearest obstacle, negative = penetration depth inside an obstacle, clamped
 * to +-max_distance; cell (x, y) at index y * size_x + x, cell center at
 * world (origin_x + (x + 0.5) * resolution, origin_y + (y + 0.5) * resolution).
 * occupied: binarized raster of the same shape (1 = obstacle) -- the
 * visualization surface (distance alone cannot distinguish "obstacle here"
 * from "clamp plateau"). sequence increments per published rebuild.
 */
struct EsdfGrid {
  ayanami::geometry_msgs::Header header;
  double resolution = 0.05;
  int size_x = 0;
  int size_y = 0;
  double origin_x = 0.0;
  double origin_y = 0.0;
  double max_distance = 3.0;
  std::uint64_t sequence = 0;
  std::vector<float> distance;
  std::vector<unsigned char> occupied;
  using Ptr = std::shared_ptr<EsdfGrid>;
  using ConstPtr = std::shared_ptr<const EsdfGrid>;
};

/**
 * @brief Path (equivalent to nav_msgs/Path); poses is a sequence of framed poses.
 */
struct Path {
  using poses_type = std::vector<ayanami::geometry_msgs::PoseStamped>;
  ayanami::geometry_msgs::Header header;
  poses_type poses;
  using Ptr = std::shared_ptr<Path>;
  using ConstPtr = std::shared_ptr<const Path>;
};

/**
 * @brief Pose + 6x6 covariance (equivalent to geometry_msgs/PoseWithCovariance, row-major).
 */
struct PoseWithCovariance {
  ayanami::geometry_msgs::Pose pose;
  std::array<double, 36> covariance{};
  using Ptr = std::shared_ptr<PoseWithCovariance>;
  using ConstPtr = std::shared_ptr<const PoseWithCovariance>;
};

/**
 * @brief Twist + 6x6 covariance (equivalent to geometry_msgs/TwistWithCovariance, row-major).
 */
struct TwistWithCovariance {
  ayanami::geometry_msgs::Twist twist;
  std::array<double, 36> covariance{};
  using Ptr = std::shared_ptr<TwistWithCovariance>;
  using ConstPtr = std::shared_ptr<const TwistWithCovariance>;
};

/**
 * @brief Odometry (equivalent to nav_msgs/Odometry, layout aligned with
 *        ROS2: header / child_frame_id / PoseWithCovariance pose /
 *        TwistWithCovariance twist).
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
