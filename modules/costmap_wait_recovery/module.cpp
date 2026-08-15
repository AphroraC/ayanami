#include <ayanami/modules/costmap_wait_recovery/costmap_wait_recovery.hpp>

extern "C" ayanami::costmap::CostmapRecovery* create_costmap_recovery_module() {
  return new ayanami::modules::CostmapWaitRecovery();
}
