#include "costmap_back_up_recovery.hpp"

extern "C" ayanami::costmap::CostmapRecovery* create_costmap_recovery_module() {
  return new ayanami::modules::CostmapBackUpRecovery();
}
