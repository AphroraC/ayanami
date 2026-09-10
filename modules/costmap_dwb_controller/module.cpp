// module.cpp -- costmap_dwb_controller module factory entry point.
#include "costmap_dwb_controller.hpp"
#include <ayanami/utility/extension_module.hpp>

extern "C" ayanami::ExtensionModule* create_extension_module() {
  return new ayanami::modules::CostmapDwbController();
}