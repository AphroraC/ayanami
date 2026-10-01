// modules/costmap_cleanup_recovery -- clear costmap recovery module.
//
// Pure C++ (zero ROS): unified recovery protocol (start/tick/finish). On the first cycle it builds a typed
// ClearCostmapCommand and submits it via CostmapCommandPort to the single writer
// (CostmapNavigationServer) to execute the clear, followed by SUCCESS (no motion, zero velocity).
// The module does not sleep on its own and never directly touches mutable wrappers / layers.

#include "costmap_cleanup_recovery.hpp"

namespace ayanami {
namespace modules {

using ayanami::costmap::ClearCostmapCommand;
using ayanami::costmap::CostmapCommandPort;


void CostmapCleanupRecovery::configureParameters(const ayanami::ModuleContext& context) {
  // Parameter loading, separated from configuration: the loader calls this
  // right after a successful load, and every value lands in the module's own
  // CostmapCleanupRecoveryParam member (whose field initializers are the defaults for keys the
  // config omits).
  // Single source of parameters: this module's json (the file
  // config_recovery points to). Param section convention: the section key =
  // the module json "name" field, delivered as ModuleContext::name.
  const std::string& section = context.name;
  param.clear_radius =
      context.config.param<double>(section, "clear_radius", param.clear_radius);
  param.affected_maps = context.config.param<std::string>(
      section, "affected_maps", param.affected_maps);
}

void CostmapCleanupRecovery::configure(
    const ayanami::ModuleContext& context,
    ayanami::costmap::CostmapMapContext::ConstPtr map) {
  // Direct-construction callers (tests, or a host that builds the object
  // itself) never go through the loader's configureParameters call, so bind
  // the parameters here as well. Re-reading the same keys from the same
  // config is idempotent.
  configureParameters(context);
  name = context.name;
  // The map write-command port is obtained via the map context (AbstractMapContext::commandPort).
  command_port =
      std::dynamic_pointer_cast<CostmapCommandPort>(
          map ? map->commandPort() : nullptr);
}


void CostmapCleanupRecovery::start() {
  canceled.store(false);
  submitted = false;
}

ResultT<void> CostmapCleanupRecovery::tick(RecoveryCycleData& cycle) {
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

bool CostmapCleanupRecovery::cancel() {
  canceled.store(true);
  return true;
}

}  // namespace modules
}  // namespace ayanami
