#pragma once

#include <memory>
#include <string>

#include <costmap_2d_layers/inflation_layer.hpp>

#include <ayanami/costmap/core/costmap_layer_plugin.hpp>
#include <ayanami/utility/config.hpp>

namespace ayanami {
namespace modules {

/// @brief Inflation-layer plugin (costmap layer family, modules/ so+json).
///
/// Owns a costmap_2d_layers::InflationLayer and reads its own json section:
/// inflation_radius / inscribed_radius / cost_scaling_factor. The inscribed
/// radius is a fallback only: a configured robot footprint propagates
/// through LayeredCostmap::setFootprint -> onFootprintChanged and overrides
/// it.
class CostmapInflationLayerPlugin final
    : public ayanami::costmap::CostmapLayerPlugin {
 public:
  CostmapInflationLayerPlugin() = default;
  /// @brief Out-of-line key function: pins the vtable emission to
  /// module.cpp (all-inline virtuals make it vague-linkage, which dlopen
  /// cannot resolve across the module boundary).
  ~CostmapInflationLayerPlugin() override;

  costmap_2d::Layer::Ptr layer() override { return impl_; }

  void configureParameters(const ayanami::ModuleContext& context) override;

 private:
  struct CostmapInflationLayerParam {
    double inflation_radius = 0.55;
    double inscribed_radius = 0.12;
    double cost_scaling_factor = 10.0;
  };

  CostmapInflationLayerParam param_;
  std::shared_ptr<costmap_2d_layers::InflationLayer> impl_{
      std::make_shared<costmap_2d_layers::InflationLayer>()};
};

}  // namespace modules
}  // namespace ayanami
