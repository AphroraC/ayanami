#include <ayanami/utility/extension_module.hpp>
#include <ayanami/utility/load_module.hpp>

namespace ayanami {

ExtensionModule::Ptr ExtensionModule::loadModule(const std::string& so_name) {
  auto create =
      loadModuleFromSo<ExtensionModule* (*)()>(so_name, "create_extension_module");
  if (create == nullptr) {
    return nullptr;
  }
  // No custom deleter: the dlopen handle is never dlclosed; the object lives as long as the process.
  return Ptr(create());
}

ExtensionModule::Ptr ExtensionModule::loadModule(
    const std::string& so_name, const ModuleContext& context) {
  Ptr module = loadModule(so_name);
  if (module) {
    // Only a module that loaded successfully gets its parameters read.
    module->configureParameters(context);
  }
  return module;
}

} // namespace ayanami
