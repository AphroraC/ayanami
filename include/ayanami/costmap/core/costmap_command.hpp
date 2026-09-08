#pragma once

#include "ayanami/map/abstract_map_context.hpp"

#include <memory>
#include <string>

namespace ayanami {
namespace costmap {

/**
 * @brief Clear-costmap command (typed clear command).
 *
 * Submitted via CostmapCommandPort::submit() to the sole map writer
 * (CostmapNavigationServer) for execution: the writer resolves the robot
 * pose in each target map's frame, clears the dynamic obstacle layer
 * within clear_radius and recomputes (updateMap -> publish a new epoch).
 * Recovery plugins themselves never touch the mutable wrapper / layer.
 */
struct ClearCostmapCommand {
  std::string affected_maps = "both";  ///< "global" | "local" | "both"
  double clear_radius = 1.0;           ///< Clearing radius centered on the robot (m).
  bool enabled = true;                 ///< false is treated as a no-op (idempotent skip).
};

/**
 * @brief Map change command port: the narrow interface through which
 * recovery/external components submit typed write commands to the writer.
 *
 * Derives from map::MapCommandPort and is exposed in a map-family-agnostic
 * form via AbstractMapContext::commandPort(); the implementation (writer
 * side) is provided only by CostmapNavigationServer; plugins just hold this
 * interface and submit, hence "map mutation only through command port".
 */
class CostmapCommandPort : public map::MapCommandPort {
 public:
  using Ptr = std::shared_ptr<CostmapCommandPort>;

  /// @brief Submit one clear-costmap command. Returns true when the writer has accepted it.
  virtual bool submit(const ClearCostmapCommand& command) = 0;
};

}  // namespace costmap
}  // namespace ayanami