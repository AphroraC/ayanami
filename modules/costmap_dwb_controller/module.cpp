// module.cpp — costmap_dwb_controller 插件工厂入口。
#include "costmap_dwb_controller.hpp"
#include <ayanami/utility/extension_module.hpp>

extern "C" ayanami::ExtensionModule* create_extension_module() {
  return new ayanami::modules::CostmapDwBLocalPlanner();
}