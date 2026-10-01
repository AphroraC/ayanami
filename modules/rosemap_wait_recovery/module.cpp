#include "rosemap_wait_recovery.hpp"
#include <ayanami/utility/extension_module.hpp>

extern "C" ayanami::ExtensionModule* create_extension_module() {
  return new ayanami::modules::RosemapWaitRecovery();
}
