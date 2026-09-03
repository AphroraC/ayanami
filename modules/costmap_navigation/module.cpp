// modules/costmap_navigation 模块工厂 (spec §4.3)。
// 公共 dlopen 入口: create_extension_module → ayanami::ExtensionModule*
// (宿主插件 CostmapNavigation 间接派生 AbstractNavigation/ExtensionModule)。

#include "costmap_navigation.hpp"
#include <ayanami/utility/extension_module.hpp>

extern "C" ayanami::ExtensionModule* create_extension_module() {
  return new ayanami::modules::CostmapNavigation();
}