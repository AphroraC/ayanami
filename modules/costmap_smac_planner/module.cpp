#include <ayanami/modules/costmap_smac_planner/costmap_smac_planner.hpp>

extern "C" ayanami::costmap::CostmapPlanner* create_costmap_planner_module() {
  return new ayanami::modules::CostmapSmacPlanner();
}
