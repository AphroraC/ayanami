#include "costmap_spin_recovery.hpp"

extern "C" ayanami::costmap::CostmapRecovery* create_costmap_recovery_module() {
  return new ayanami::modules::CostmapSpinRecovery();
}
