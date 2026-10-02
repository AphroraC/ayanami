#include "esdfmap_minco_smoother.hpp"
#include <ayanami/utility/extension_module.hpp>

extern "C" ayanami::ExtensionModule* create_extension_module() {
  return new ayanami::modules::EsdfmapMincoSmoother();
}
