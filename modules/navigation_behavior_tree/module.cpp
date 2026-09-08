// module.cpp -- navigation_behavior_tree extension module factory.
// Public dlopen entry: create_extension_module -> ayanami::ExtensionModule*
// (derives from BehaviorTreeExtension).

#include "navigation_behavior_tree.hpp"
#include <ayanami/utility/extension_module.hpp>

extern "C" ayanami::ExtensionModule* create_extension_module() {
  return new ayanami::modules::NavigationBehaviorTreeExtension();
}