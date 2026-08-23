// module.cpp — costmap_dwb_controller 插件工厂入口。
#include "costmap_dwb_controller.hpp"

extern "C" ayanami::abstract::AbstractController* create_controller_module() {
  return new ayanami::modules::CostmapDwBLocalPlanner();
}
