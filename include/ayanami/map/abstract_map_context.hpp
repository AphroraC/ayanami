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

// Capability bits are objective map facts only (what the family can do),
// never usage intents (planning targets are assembly decisions, not
// capabilities).
enum class MapCapability : std::uint32_t {
  None = 0,
  CollisionQuery = 1u << 0,
  DynamicObstacleUpdate = 1u << 1,
  ClearRegion = 1u << 2,
};

/**
 * @brief Map-family-neutral map context contract.
 *
 * Family convention (frames): every family exposes a global reference frame
 * and a local working frame. Read contracts per plugin family:
 * - planner: reads the overlay (superposition) of the global and local map
 *   content;
 * - smoother: reads only the local map content;
 * - controller/recovery: family-defined (costmap: local).
 *
 * Robot state is NOT part of the map context: pose/velocity queries go
 * through RobotStateController exclusively (single query facade).
 * Freshness is expressed by isCurrent() only (no versioning).
 */
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
  virtual std::uint32_t capabilities() const = 0;
  virtual MapCommandPort::Ptr commandPort() const { return nullptr; }

  /// @brief Path validity query (collision/query capability,
  /// MapCapability::CollisionQuery): whether every path point is traversable
  /// is defined by the map family (costmap = no lethal/unknown and in
  /// bounds).
  /// Default implementation: valid iff non-empty -- the conservative
  /// degradation for map families without collision query, used by the BT
  /// IsPathValid condition to fall back in runtime-free/map-free
  /// environments.
  virtual bool isPathValid(const ayanami::nav_msgs::Path& path) const {
    return !path.poses.empty();
  }
};

}  // namespace ayanami::map
