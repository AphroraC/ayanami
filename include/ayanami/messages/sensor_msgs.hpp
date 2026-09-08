#pragma once

#include "ayanami/messages/geometry_msgs.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace ayanami {
namespace sensor_msgs {

/**
 * @brief Point cloud field description (equivalent to sensor_msgs/PointField).
 */
struct PointField {
  enum DataType : uint8_t {
    INT8 = 1,
    UINT8 = 2,
    INT16 = 3,
    UINT16 = 4,
    INT32 = 5,
    UINT32 = 6,
    FLOAT32 = 7,
    FLOAT64 = 8,
  };
  std::string name;
  uint32_t offset = 0;
  uint8_t datatype = FLOAT32;
  uint32_t count = 1;
  using Ptr = std::shared_ptr<PointField>;
  using ConstPtr = std::shared_ptr<const PointField>;
};

/**
 * @brief Point cloud (equivalent to sensor_msgs/PointCloud2); data is per-point bytes, a simplified Eigen-independent version.
 */
struct PointCloud2 {
  ayanami::geometry_msgs::Header header;
  uint32_t height = 0;
  uint32_t width = 0;
  std::vector<PointField> fields;
  bool is_bigendian = false;
  uint32_t point_step = 0;
  uint32_t row_step = 0;
  std::vector<uint8_t> data;
  bool is_dense = false;
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
