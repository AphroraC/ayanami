#include <ayanami/utility/extension_module.hpp>

namespace {

class DummyExtension : public ayanami::ExtensionModule {};

} // namespace

extern "C" ayanami::ExtensionModule* create_extension_module() {
  return new DummyExtension;
}
