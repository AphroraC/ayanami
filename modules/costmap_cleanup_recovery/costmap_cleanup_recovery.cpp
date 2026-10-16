// modules/costmap_cleanup_recovery -- map clearing recovery plugin (the
// nav2 ClearingActions entry: clear local + global in one run).
//
// Unified recovery protocol; on the first cycle the typed ClearCostmapCommand
// goes through the injected sole-writer port, then SUCCESS (no motion).

#include "costmap_cleanup_recovery.hpp"

namespace ayanami {
namespace modules {

using ayanami::costmap::ClearCostmapCommand;

void CostmapCleanupRecovery::configureParameters(
    const ayanami::ModuleContext& context) {
  const std::string& section = context.name;
  param.clear_radius =
      context.config.param<double>(section, "clear_radius", param.clear_radius);
  param.affected_maps = context.config.param<std::string>(
      section, "affected_maps", param.affected_maps);
}

void CostmapCleanupRecovery::configure(
    const ayanami::ModuleContext& context,
    ayanami::costmap::CostmapMapContext::ConstPtr map,
    ClearCommandFn clear) {
  configureParameters(context);
  name = context.name;
  (void)map;  // clearing goes through the writer port, never the reader
  clear_command = std::move(clear);
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
  if (!clear_command) {
    return ResultT<void>::failure(
        ErrorCode::INTERNAL_ERROR, "clear costmap recovery has no clear port");
  }

  if (!submitted) {
    ClearCostmapCommand command;
    command.affected_maps = param.affected_maps;
    command.clear_radius = param.clear_radius;
    if (!clear_command(command)) {
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
