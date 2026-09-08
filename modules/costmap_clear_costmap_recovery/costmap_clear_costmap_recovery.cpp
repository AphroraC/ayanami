// modules/costmap_clear_costmap_recovery -- clear costmap recovery module.
//
// Pure C++ (zero ROS): unified recovery protocol (start/tick/finish). On the first cycle it builds a typed
// ClearCostmapCommand and submits it via CostmapCommandPort to the single writer
// (CostmapNavigationServer) to execute the clear, followed by SUCCESS (no motion, zero velocity).
// The plugin does not sleep on its own and never directly touches mutable wrappers / layers.

#include "costmap_clear_costmap_recovery.hpp"

namespace ayanami {
namespace modules {

using ayanami::costmap::ClearCostmapCommand;
using ayanami::costmap::CostmapCommandPort;

void CostmapClearCostmapRecovery::configure(
    const ayanami::PluginContext& context,
    ayanami::costmap::CostmapMapContext::ConstPtr map) {
  // Single source of parameters: the recovery section of this module's json (the file config_recovery points to).
  param.clear_radius =
      context.config.param<double>("recovery", "clear_radius", param.clear_radius);
  param.affected_maps = context.config.param<std::string>(
      "recovery", "affected_maps", param.affected_maps);
  name = context.name;
  // The map write-command port is obtained via the map context (AbstractMapContext::commandPort).
  command_port =
      std::dynamic_pointer_cast<CostmapCommandPort>(
          map ? map->commandPort() : nullptr);
}

void CostmapClearCostmapRecovery::start() {
  canceled.store(false);
  submitted = false;
}

ResultT<void> CostmapClearCostmapRecovery::tick(RecoveryCycleData& cycle) {
  if (canceled.load()) {
    return ResultT<void>::failure(ErrorCode::CANCELED,
                                    "clear costmap recovery canceled");
  }
  if (!command_port) {
    return ResultT<void>::failure(
        ErrorCode::INTERNAL_ERROR, "clear costmap recovery has no command port");
  }

  if (!submitted) {
    // Build the typed clear command (map changes may only be submitted via the writer command port).
    ClearCostmapCommand command;
    command.affected_maps = param.affected_maps;
    command.clear_radius = param.clear_radius;
    if (!command_port->submit(command)) {
      return ResultT<void>::failure(
          ErrorCode::INTERNAL_ERROR,
          "clear costmap recovery command rejected by writer");
    }
    submitted = true;
  }

  cycle.status = RecoveryStatus::SUCCESS;
  return ResultT<void>::success();
}

bool CostmapClearCostmapRecovery::cancel() {
  canceled.store(true);
  return true;
}

}  // namespace modules
}  // namespace ayanami
