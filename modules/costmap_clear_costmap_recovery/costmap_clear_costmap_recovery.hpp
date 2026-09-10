#pragma once

#include <atomic>
#include <memory>
#include <string>

#include <ayanami/costmap/core/costmap_recovery.hpp>
#include <ayanami/types.hpp>

namespace ayanami {
namespace modules {

using ayanami::costmap::ClearCostmapCommand;
using ayanami::costmap::CostmapCommandPort;

/**
 * @brief Clear costmap recovery module (costmap_clear_costmap_recovery).
 *
 * Pure C++ (zero ROS): unified recovery protocol (start/tick/finish). On the first cycle it builds a typed
 * ClearCostmapCommand and submits it via CostmapCommandPort to the single map writer
 * (CostmapNavigationServer) for execution, followed by SUCCESS (no motion, zero velocity). The module does not
 * sleep on its own and never touches any mutable wrapper / layer.
 * The command port is obtained at configure time via map->commandPort() (the map family's write-command interface).
 * The global static base map is restored by StaticLayer; master grid walls are not written to FREE directly.
 * Parameters are uniformly read from ModuleContext.config (the module json that config_recovery points to).
 */
class CostmapClearCostmapRecovery final : public ayanami::costmap::CostmapRecovery {
 public:
  CostmapClearCostmapRecovery() = default;
  // The derived layer adds a bare virtual ~ so that shared_ptr deletion goes through the virtual destructor.
  virtual ~CostmapClearCostmapRecovery() = default;

  /// @brief Parameter-loading seam (ExtensionModule): called by the loader
  /// right after a successful load, and by configure() for callers that build
  /// the object directly.
  void configureParameters(const ayanami::ModuleContext& context) override;

  void configure(const ayanami::ModuleContext& context,
                 ayanami::costmap::CostmapMapContext::ConstPtr map) override;

  void start() override;
  ResultT<void> tick(RecoveryCycleData& cycle) override;
  void finish() override {}

  bool cancel() override;

 private:
  struct CostmapClearCostmapRecoveryParam {
    double clear_radius = 1.0;
    std::string affected_maps = "both";  // global / local / both
  };

  CostmapClearCostmapRecoveryParam param;

  std::string name;
  CostmapCommandPort::Ptr command_port;
  std::atomic<bool> canceled{false};
  bool submitted{false};
};

}  // namespace modules
}  // namespace ayanami
