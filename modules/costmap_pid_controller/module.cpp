#include "costmap_pid_controller.hpp"

extern "C" ayanami::abstract::AbstractController* create_controller_module() {
  return new ayanami::modules::CostmapPidController();
}
