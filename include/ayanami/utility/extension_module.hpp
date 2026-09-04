#pragma once
#include <memory>
#include <string>

#include "ayanami/utility/loaded_module.hpp"

namespace ayanami {

/**
 * @brief Abstract base for extension modules (glim-style, spec §4.6).
 *
 * Pure C++, no ROS includes. Derived modules register callback slots (e.g.
 * NavigationCallbacks::on_xxx.add(...)) during construction and are loaded via
 * dlopen using the `create_extension_module` factory symbol.
 */
class ExtensionModule {
public:
  using Ptr = std::shared_ptr<ExtensionModule>;

  virtual ~ExtensionModule() = default;

  /**
   * @brief Load an extension module from a shared object via dlsym of
   *        "create_extension_module".
   *
   * @return shared_ptr without a custom deleter (process lifetime, spec §4.1),
   *         or nullptr on failure.
   */
  static Ptr load_module(const std::string& so_name);

  /// Future unload gate. Current modules remain resident by design.
  static bool can_unload(const LoadedModule::Ptr& module) noexcept {
    return module && module->unloadable();
  }
};

} // namespace ayanami
