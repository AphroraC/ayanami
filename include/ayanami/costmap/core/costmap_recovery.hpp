#pragma once

#include <functional>
#include <memory>
#include <string>

#include "ayanami/abstract/core/abstract_recovery.hpp"
#include "ayanami/costmap/components/costmap_commands.hpp"
#include "ayanami/costmap/components/costmap_map_context.hpp"
#include "ayanami/types.hpp"

namespace ayanami {
namespace costmap {

using ayanami::ErrorCode;
using ayanami::ResultT;

/**
 * @brief Recovery behavior module base class (costmap family; revived for
 * the FSM era from the BT-era header).
 *
 * On top of abstract::AbstractRecovery (the start/tick/finish unified
 * protocol) only a three-part injection is added:
 * - the module context (name + its own module json),
 * - a const map context: collision-type recoveries (spin/back_up) read the
 *   global grid through readGlobal() under its shared lock,
 * - a clear-command port bound to
 *   CostmapNavigationServer::executeClearCommand (the sole-writer clear
 *   path; the map-only cleanup recovery submits through it and never
 *   writes a wrapper under a lock itself).
 * Recoveries that need neither simply ignore the injected handles.
 */
class CostmapRecovery : public ayanami::abstract::AbstractRecovery {
 public:
  using Ptr = std::shared_ptr<CostmapRecovery>;
  using ClearCommandFn = std::function<bool(const ClearCostmapCommand&)>;

  virtual void configure(const ModuleContext& context,
                         CostmapMapContext::ConstPtr map,
                         ClearCommandFn clear_command) = 0;
};

}  // namespace costmap
}  // namespace ayanami
