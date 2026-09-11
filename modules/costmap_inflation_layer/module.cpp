#include "costmap_inflation_layer_plugin.hpp"

#include <ayanami/utility/extension_module.hpp>

namespace ayanami {
namespace modules {

// Out-of-line key function (see the header note): pins the class vtable.
CostmapInflationLayerPlugin::~CostmapInflationLayerPlugin() = default;

void CostmapInflationLayerPlugin::configureParameters(
    const ayanami::ModuleContext& context) {
  const Config& file = context.config;
  const std::string& section = context.name;
  param_.inflation_radius =
      file.param<double>(section, "inflation_radius", param_.inflation_radius);
  param_.inscribed_radius =
      file.param<double>(section, "inscribed_radius", param_.inscribed_radius);
  param_.cost_scaling_factor = file.param<double>(
      section, "cost_scaling_factor", param_.cost_scaling_factor);
  impl_->setInflationRadius(param_.inflation_radius);
  impl_->setInscribedRadius(param_.inscribed_radius);
  impl_->setCostScalingFactor(param_.cost_scaling_factor);
}

}  // namespace modules
}  // namespace ayanami

extern "C" ayanami::ExtensionModule* create_extension_module() {
  return new ayanami::modules::CostmapInflationLayerPlugin();
}
