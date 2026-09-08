#pragma once

#include <memory>
#include <string>

#include "ayanami/abstract/core/abstract_recovery.hpp"
#include "ayanami/costmap/core/costmap_command.hpp"
#include "ayanami/costmap/components/costmap_map_context.hpp"
#include "ayanami/types.hpp"

namespace ayanami {
namespace costmap {

using ayanami::ErrorCode;
using ayanami::ResultT;
/**
 * @brief Recovery behavior plugin base class (costmap family, MBF-style
 * layering).
 *
 * On top of abstract::AbstractRecovery (the start/tick/cancel/finish
 * unified protocol), only the following is added:
 * - configure(context, map): injects the plugin logical name + its own
 *   config + a const pointer to the map module (for collision-type
 *   recoveries to read the map).
 * Recoveries that need to modify the map (e.g. clear) fetch the
 * CostmapCommandPort via map->commandPort() inside configure and submit
 * commands, executed by the sole writer (CostmapNavigationServer); recovery
 * plugins never write the map directly under a lock.
 */
class CostmapRecovery : public ayanami::abstract::AbstractRecovery {
 public:
  using Ptr = std::shared_ptr<CostmapRecovery>;

  virtual void configure(const PluginContext& context,
                         CostmapMapContext::ConstPtr map) = 0;
};

}  // namespace costmap
}  // namespace ayanami
