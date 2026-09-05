#pragma once

#include "ayanami/messages/geometry_msgs.hpp"
#include "ayanami/messages/nav_msgs.hpp"
#include <cstdint>
#include <memory>
#include <string>

namespace ayanami::map {

class MapCommandPort {
 public:
  using Ptr = std::shared_ptr<MapCommandPort>;
  virtual ~MapCommandPort() = default;
};

enum class MapCapability : std::uint32_t {
  None = 0,
  CollisionQuery = 1u << 0,
  GlobalPlanning = 1u << 1,
  LocalPlanning = 1u << 2,
  DynamicObstacleUpdate = 1u << 3,
  ClearRegion = 1u << 4,
};

class AbstractMapContext {
 public:
  using Ptr = std::shared_ptr<AbstractMapContext>;
  using ConstPtr = std::shared_ptr<const AbstractMapContext>;

  virtual ~AbstractMapContext() = default;

  virtual std::string mapFamily() const = 0;
  virtual std::string globalFrame() const = 0;
  virtual std::string localFrame() const = 0;
  virtual std::string robotFrame() const = 0;
  virtual bool isCurrent() const = 0;
  virtual std::uint64_t version() const = 0;
  virtual std::uint32_t capabilities() const = 0;
  virtual MapCommandPort::Ptr commandPort() const { return nullptr; }
  virtual bool getRobotState(
      geometry_msgs::PoseStamped& pose,
      geometry_msgs::TwistStamped& velocity) const = 0;

  /// @brief 路径有效性查询 (collision/query capability, MapCapability::
  /// CollisionQuery): 路径各点是否可通行由地图族定义 (costmap = 无 lethal/
  /// unknown 且在界内)。
  /// 默认实现: 非空即有效 —— 无碰撞查询能力的地图族的保守退化, 供 BT
  /// IsPathValid 条件在无 runtime/无地图环境回退。
  virtual bool isPathValid(const ayanami::nav_msgs::Path& path) const {
    return !path.poses.empty();
  }
};

}  // namespace ayanami::map
