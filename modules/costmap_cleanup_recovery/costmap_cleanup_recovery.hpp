#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <string>

#include <ayanami/costmap/core/costmap_recovery.hpp>
#include <ayanami/types.hpp>

namespace ayanami {
namespace modules {

using ayanami::costmap::ClearCostmapCommand;

/**
 * @brief Clear costmap recovery module (costmap_cleanup_recovery) -- the
 * nav2 ClearingActions entry of the recovery rotation.
 *
 * Unified recovery protocol (start/tick/finish), no motion. On the first
 * cycle it builds a typed ClearCostmapCommand and executes it through the
 * injected clear-command port (the sole writer's executeClearCommand --
 * clearCostmapArea under the wrapper write lock + master recompute +
 * static-layer repaint), then reports SUCCESS. The static base map is
 * restored by the StaticLayer; the module never touches a wrapper/layer
 * itself. Parameters come from ModuleContext.config (the module json that
 * config_recovery points to).
 */
class CostmapCleanupRecovery final : public ayanami::costmap::CostmapRecovery {
 public:
  CostmapCleanupRecovery() = default;
  virtual ~CostmapCleanupRecovery() = default;

  /// @brief Parameter-loading seam (ExtensionModule), called by the loader
  /// right after a successful load (and again by configure() for direct
  /// construction callers; re-reading the same keys is idempotent).
  void configureParameters(const ayanami::ModuleContext& context) override;

  void configure(const ayanami::ModuleContext& context,
                 ayanami::costmap::CostmapMapContext::ConstPtr map,
                 ClearCommandFn clear_command) override;

  void start() override;
  ResultT<void> tick(RecoveryCycleData& cycle) override;
  void finish() override {}

  bool cancel() override;

 private:
  struct CostmapCleanupRecoveryParam {
    double clear_radius = 1.0;
    std::string affected_maps = "both";  // global / local / both
  };

  CostmapCleanupRecoveryParam param;

  std::string name;
  ayanami::costmap::CostmapRecovery::ClearCommandFn clear_command;
  std::atomic<bool> canceled{false};
  bool submitted{false};
};

}  // namespace modules
}  // namespace ayanami
