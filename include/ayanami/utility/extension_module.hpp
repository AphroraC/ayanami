#pragma once
#include <cstdint>
#include <memory>
#include <string>

#include "ayanami/types.hpp"
#include "ayanami/utility/loaded_module.hpp"

namespace ayanami {

/**
 * @brief Abstract base for extension modules (glim-style).
 *
 * Pure C++, no ROS includes. Derived modules register callback slots (e.g.
 * NavigationCallbacks::on_xxx.add(...)) during construction and are loaded via
 * dlopen using the `create_extension_module` factory symbol.
 *
 * Parameter loading is a separate step that happens AFTER a successful load:
 * the factory stays argument-free, and the loader hands the module its
 * ModuleContext through configureParameters() once the module object exists and
 * has been accepted (type cast succeeded, not rejected by the init hook). That
 * ordering is deliberate -- a module reads its parameters only after it is
 * known to be constructible and loadable, and the module's own XxxParam member
 * is the single home of those values.
 */
class ExtensionModule {
public:
  using Ptr = std::shared_ptr<ExtensionModule>;

  virtual ~ExtensionModule() = default;

  /**
   * @brief Load an extension module from a shared object via dlsym of
   *        "create_extension_module". Parameters are NOT loaded yet; callers
   *        holding a ModuleContext use the overload below.
   *
   * @return shared_ptr without a custom deleter (process lifetime),
   *         or nullptr on failure.
   */
  static Ptr loadModule(const std::string& so_name);

  /**
   * @brief Load, then inject the module's context (in that order).
   *
   * Equivalent to loadModule(so_name) followed by
   * configureParameters(context) on success; a failed load leaves nothing to
   * configure and returns nullptr.
   */
  static Ptr loadModule(const std::string& so_name,
                        const ModuleContext& context);

  /**
   * @brief Parameter-loading seam, called once by the loader after a
   * successful load and before the module is registered anywhere.
   *
   * Implementations fill their own XxxParam member from context.config under
   * context.name (the section key), and must not depend on any other module.
   * Default: no-op, for module kinds that own their configuration (e.g. the
   * behavior-tree extension, which resolves its tree asset itself).
   */
  virtual void configureParameters(const ModuleContext& context) {
    (void)context;
  }

  /**
   * @brief Interface version of the module as compiled.
   *
   * Each plugin-family base overrides this with its own kApiVersion constant.
   * Because the method is virtual, a module built against an older family
   * header carries the old constant in its vtable, so the loader can compare it
   * against the family constant it was itself compiled with and refuse a stale
   * .so instead of letting two disagreeing layouts talk to each other (RTTI
   * alone cannot catch that: the class names still match).
   *
   * 0 means "no family version contract" -- module kinds that are not plugin
   * families (host, behavior-tree extension, middleware adapters) and the
   * default implementation.
   */
  static constexpr std::uint32_t kNoApiVersionContract = 0;
  virtual std::uint32_t apiVersion() const { return kNoApiVersionContract; }

  /// Future unload gate. Current modules remain resident by design.
  static bool canUnload(const LoadedModule::Ptr& module) noexcept {
    return module && module->unloadable();
  }
};

} // namespace ayanami
