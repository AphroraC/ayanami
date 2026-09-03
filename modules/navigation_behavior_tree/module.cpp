// module.cpp — navigation_behavior_tree 扩展模块工厂。
// 公共 dlopen 入口: create_extension_module → ayanami::ExtensionModule*
// (派生自 BehaviorTreeExtension)。

#include "navigation_behavior_tree.hpp"
#include <ayanami/utility/extension_module.hpp>

extern "C" ayanami::ExtensionModule* create_extension_module() {
  return new ayanami::modules::NavigationBehaviorTreeExtension();
}