#include <ayanami/modules/costmap_savitzky_smoother/costmap_savitzky_smoother.hpp>

extern "C" ayanami::costmap::CostmapSmoother*
create_costmap_smoother_module() {
  return new ayanami::modules::CostmapSavitzkySmoother();
}
