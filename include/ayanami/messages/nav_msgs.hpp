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
 * @brief Incremental costmap update (equivalent to map_msgs/OccupancyGridUpdate).
 *
 * data holds width*height int8 values for the window whose top-left cell is
 * (x0, y0), row-major, in the coordinate system of the last full grid the
 * receiver holds for the same map name. Receivers that keep no full grid
 * must ignore updates and wait for the next full emission (geometry change
 * or first publish).
 */
struct OccupancyGridUpdate {
  uint32_t x0 = 0;
  uint32_t y0 = 0;
  uint32_t width = 0;
  uint32_t height = 0;
  std::vector<int8_t> data;
  using Ptr = std::shared_ptr<OccupancyGridUpdate>;
  using ConstPtr = std::shared_ptr<const OccupancyGridUpdate>;
};

/**
 * @brief Path (equivalent to nav_msgs/Path); poses is a sequence of framed poses.
 */
/// @brief Raw master-grid copy (lossless cost values, row-major). The
/// in-framework counterpart of the lossy occupancy visualization slots:
/// consumers that need the true cost gradient (the smoother's collision
/// check) subscribe to NavigationCallbacks::on_costmap_raw and keep their
/// own copy, so no live-map lock is ever held across an algorithm.
struct RawCostmap {
  /// @brief Publisher-side map name ("global_costmap" / "local_costmap").
  std::string name;
  std::string frame_id;
  double resolution = 0.05;
  double origin_x = 0.0;
  double origin_y = 0.0;
  uint32_t width = 0;
  uint32_t height = 0;
  /// @brief Monotonic per-name content revision: advances on every
  /// emission whose costs differ from the previous one; subscribers gate
  /// recompute work on it.
  uint64_t revision = 0;
  std::vector<unsigned char> costs;

  using Ptr = std::shared_ptr<RawCostmap>;
  using ConstPtr = std::shared_ptr<const RawCostmap>;

  /// @brief World -> cell; false when outside the copy's bounds (mirrors
  /// costmap_2d::Costmap2D::worldToMap).
  bool worldToMap(double wx, double wy, unsigned int& mx,
                  unsigned int& my) const {
    if (wx < origin_x || wy < origin_y || resolution <= 0.0) {
      return false;
    }
    mx = static_cast<unsigned int>((wx - origin_x) / resolution);
    my = static_cast<unsigned int>((wy - origin_y) / resolution);
    return mx < width && my < height;
  }
  unsigned char getCost(unsigned int mx, unsigned int my) const {
    return costs[static_cast<std::size_t>(my) * width + mx];
  }
};

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
