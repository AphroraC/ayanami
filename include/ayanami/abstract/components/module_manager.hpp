#pragma once

#include "ayanami/utility/config.hpp"
#include "ayanami/utility/extension_module.hpp"

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace ayanami {
namespace components {

/**
 * @brief Module loading descriptor.
 *
 * name is the logical name (used by getModule/hasModule); so_name is the
 * .so to dlopen. The factory symbol is uniformly
 * `create_extension_module` (an ExtensionModule-derived type); there is no
 * longer an arbitrary create_symbol field.
 */
struct ModuleSpec {
  std::string name;
  std::string so_name;
};

/**
 * @brief Typed module manager (modeled after MBF AbstractPluginManager).
 *
 * loadModule goes through ExtensionModule::loadModule (dlopen + dlsym of
 * the common `create_extension_module` factory) to obtain an
 * ExtensionModule::Ptr, gets a ModuleType::Ptr via the typed
 * std::dynamic_pointer_cast<ModuleType> while keeping ownership (sharing
 * the reference count with ExtensionModule::Ptr), then invokes the init_fn
 * callback (the caller injects map objects); when init_fn returns false the
 * module is not registered. clearModules() releases all modules while the
 * dlopen handle is still alive (destructor discipline).
 *
 * Template methods are implemented inline in the header;
 * impl/module_manager.cpp performs explicit instantiations for the 4
 * navigation module abstractions, precompiling the dlopen loading path into
 * libayanami (all use sites).
 */
template <typename ModuleType>
class ModuleManager {
public:
  using InitFn = std::function<bool(const std::string&,
                                    std::shared_ptr<ModuleType>)>;

  /// @brief Why the last loadModule/loadSingleFromConfig call failed.
  /// Distinct reasons matter at startup: "the .so is not deployed" and "the .so
  /// was built against a different interface" need different fixes, and both
  /// used to collapse into a bare `false`.
  enum class LoadFailure : std::uint8_t {
    None = 0,             ///< no failure recorded
    InvalidSpec,          ///< empty name or so_name in the module json
    OpenOrSymbolFailed,   ///< dlopen failed, or create_extension_module missing
    TypeMismatch,         ///< loaded fine, but is not this family's interface
    ApiVersionMismatch,   ///< this family's interface, but a different version
    IdentityMismatch,     ///< the json's name/section/so_name disagree
    InitRejected,         ///< the family init hook refused the module
  };

  /// @brief Load one module: factory create_extension_module (no arguments) ->
  /// typed cast -> interface-version check -> parameter injection -> init_fn ->
  /// register.
  ///
  /// Ordering is load-bearing: the argument-free factory produces the object
  /// first, the cast decides whether it is even the right kind of module, the
  /// version check decides whether it agrees with this build's interface, and
  /// only then does configureParameters() hand it its context. A module that
  /// fails any of those steps never has its parameters read.
  ///
  /// On failure, lastFailure() says why.
  bool loadModule(const ModuleSpec& spec, const ModuleContext& context,
                  InitFn init_fn) {
    last_failure = LoadFailure::None;
    if (spec.name.empty() || spec.so_name.empty()) {
      return fail(spec, LoadFailure::InvalidSpec,
                  "module json lacks name/so_name");
    }
    auto loaded = ayanami::ExtensionModule::loadModule(spec.so_name);
    if (!loaded) {
      return fail(spec, LoadFailure::OpenOrSymbolFailed,
                  "dlopen failed or create_extension_module is missing");
    }
    // Typed conversion: only payloads truly derived from ModuleType are
    // accepted; after conversion the reference count is shared with
    // ExtensionModule::Ptr (ownership retained).
    std::shared_ptr<ModuleType> module =
        std::dynamic_pointer_cast<ModuleType>(std::move(loaded));
    if (!module) {
      return fail(spec, LoadFailure::TypeMismatch,
                  "the .so does not implement this family's interface");
    }
    // Interface version: RTTI only proves the class names line up; a .so built
    // against an older family header can still pass the cast while disagreeing
    // about the virtual surface.
    constexpr std::uint32_t expected = ModuleType::kApiVersion;
    if (module->apiVersion() != expected) {
      return fail(spec, LoadFailure::ApiVersionMismatch,
                  "reports interface version " +
                      std::to_string(module->apiVersion()) +
                      ", this build expects " + std::to_string(expected) +
                      " (rebuild the module against the current headers)");
    }
    // Parameters are read here, after a successful load, cast and version check.
    module->configureParameters(context);
    // Then report anything the module json declares that it never read: with a
    // silent default, a misspelled key is indistinguishable from an intentional
    // omission. Warn by default; `global.strict_params` makes it fatal.
    auditUnusedParameters(context.config, context.name,
                          "module '" + spec.name + "'");
    if (init_fn && !init_fn(spec.name, module)) {
      return fail(spec, LoadFailure::InitRejected,
                  "the family init hook refused the module");
    }
    modules[spec.name] = std::move(module);
    so_names[spec.name] = spec.so_name;
    return true;
  }

  /// @brief Read name/so_name from a sub-config json, then loadModule with a
  /// context of {name, that same config file}: a module's parameters live in
  /// the module json next to its family descriptor, under a section keyed by
  /// the module's own name.
  /// @param default_name Fallback logical name when the config lacks name.
  bool loadSingleFromConfig(const Config& cfg, const std::string& module_name,
                             const std::string& default_name, InitFn init_fn) {
    ModuleSpec spec;
    spec.name = cfg.param<std::string>(module_name, "name", default_name);
    spec.so_name = cfg.param<std::string>(module_name, "so_name", "");
    if (const std::string why = identityMismatch(cfg, spec); !why.empty()) {
      return fail(spec, LoadFailure::IdentityMismatch, why);
    }
    return loadModule(spec, ModuleContext{spec.name, cfg}, std::move(init_fn));
  }

  /// @brief Get a module; returns nullptr if not loaded.
  std::shared_ptr<ModuleType> getModule(const std::string& name) const {
    auto iter = modules.find(name);
    return iter == modules.end() ? nullptr : iter->second;
  }

  bool hasModule(const std::string& name) const {
    return modules.find(name) != modules.end();
  }

  /// @brief List of loaded module names (map-order traversal, sorted by name).
  std::vector<std::string> getLoadedNames() const {
    std::vector<std::string> names;
    names.reserve(modules.size());
    for (const auto& kv : modules) {
      names.push_back(kv.first);
    }
    return names;
  }

  /// @brief Release all module objects (dlopen handles are never dlclosed, /§4.5).
  void clearModules() {
    modules.clear();
    so_names.clear();
  }

  /// @brief Return a module's source .so; empty string if not loaded.
  std::string getSoName(const std::string& name) const {
    auto iter = so_names.find(name);
    return iter == so_names.end() ? std::string() : iter->second;
  }

  /// @brief Why the last load attempt failed (None when it succeeded).
  LoadFailure lastFailure() const { return last_failure; }

  /// @brief Human-readable form of the last failure, for startup diagnostics.
  const std::string& lastFailureMessage() const { return last_failure_message; }

  /// @brief Render a failure reason for logs.
  static const char* describe(LoadFailure failure) {
    switch (failure) {
      case LoadFailure::None: return "ok";
      case LoadFailure::InvalidSpec: return "invalid module spec";
      case LoadFailure::OpenOrSymbolFailed: return "dlopen/factory failed";
      case LoadFailure::TypeMismatch: return "interface type mismatch";
      case LoadFailure::ApiVersionMismatch: return "interface version mismatch";
      case LoadFailure::IdentityMismatch: return "module identity mismatch";
      case LoadFailure::InitRejected: return "rejected by the family init hook";
    }
    return "unknown";
  }

private:
  /// @brief "" when the module identity invariant holds, otherwise the reason.
  ///
  /// One module has one name, and that name is what everything else keys off:
  /// the parameter section the module reads (ModuleContext::name), the family
  /// json it is declared in, and the .so that gets loaded. When they disagree,
  /// the failure is silent rather than loud -- a module reading a section that
  /// does not exist runs on the field defaults in its XxxParam and looks
  /// perfectly healthy while ignoring the configuration entirely.
  static std::string identityMismatch(const Config& cfg,
                                      const ModuleSpec& spec) {
    if (spec.name.empty()) {
      return "empty module name";
    }
    if (!cfg.hasSection(spec.name)) {
      return "the module json declares no section named '" + spec.name +
             "', so the module would read no parameters and silently run on "
             "its built-in defaults";
    }
    // The .so basename is deliberately NOT compared against name: pointing one
    // .so at several logical names (each with its own parameter section) is a
    // legitimate way to run parameter variants of the same module.
    return {};
  }


  /// @brief Record the reason and report failure.
  bool fail(const ModuleSpec& spec, LoadFailure failure,
            const std::string& detail) {
    last_failure = failure;
    last_failure_message =
        spec.name + " (" + spec.so_name + "): " + describe(failure) + ": " +
        detail;
    return false;
  }

  std::map<std::string, std::shared_ptr<ModuleType>> modules;
  std::map<std::string, std::string> so_names;
  LoadFailure last_failure{LoadFailure::None};
  std::string last_failure_message;
};

} // namespace components
} // namespace ayanami
