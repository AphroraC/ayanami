#include <ayanami/utility/extension_module.hpp>
#include <ayanami/utility/load_module.hpp>

namespace ayanami {

ExtensionModule::Ptr ExtensionModule::load_module(const std::string& so_name) {
  auto create =
      loadModuleFromSo<ExtensionModule* (*)()>(so_name, "create_extension_module");
  if (create == nullptr) {
    return nullptr;
  }
  // 不带自定义 deleter：dlopen 句柄不 dlclose，对象与进程同寿命 。
  return Ptr(create());
}

} // namespace ayanami
