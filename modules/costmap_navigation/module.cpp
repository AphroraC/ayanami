// Task 25: modules/costmap_navigation 模块工厂 (spec §4.3)。
// dlopen 入口: create_navigation_module → AbstractNavigation* (宿主插件)。

#include <ayanami/modules/costmap_navigation/costmap_navigation.hpp>

extern "C" ayanami::abstract::AbstractNavigation* create_navigation_module() {
  return new ayanami::modules::CostmapNavigation();
}
