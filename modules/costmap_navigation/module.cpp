// modules/costmap_navigation module factory.
// Public dlopen entry: create_extension_module -> ayanami::ExtensionModule*
// (the host plugin CostmapNavigation indirectly derives AbstractNavigation/ExtensionModule).

#include "costmap_navigation.hpp"
#include <ayanami/utility/extension_module.hpp>

extern "C" ayanami::ExtensionModule* create_extension_module() {
  return new ayanami::modules::CostmapNavigation();
}