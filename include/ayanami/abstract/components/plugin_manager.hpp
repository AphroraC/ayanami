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
 * @brief Plugin loading descriptor.
 *
 * name is the logical name (used by getPlugin/hasPlugin); so_name is the
 * .so to dlopen. The factory symbol is uniformly
 * `create_extension_module` (an ExtensionModule-derived type); there is no
 * longer an arbitrary create_symbol field.
 */
struct PluginSpec {
  std::string name;
  std::string so_name;
};

/**
 * @brief Typed plugin manager (modeled after MBF AbstractPluginManager).
 *
 * loadPlugin goes through ExtensionModule::load_module (dlopen + dlsym of
 * the common `create_extension_module` factory) to obtain an
 * ExtensionModule::Ptr, gets a PluginType::Ptr via the typed
 * std::dynamic_pointer_cast<PluginType> while keeping ownership (sharing
 * the reference count with ExtensionModule::Ptr), then invokes the init_fn
 * callback (the caller injects map objects); when init_fn returns false the
 * plugin is not registered. clearPlugins() releases all plugins while the
 * dlopen handle is still alive (destructor discipline).
 *
 * Template methods are implemented inline in the header;
 * impl/plugin_manager.cpp performs explicit instantiations for the 4
 * navigation plugin abstractions, precompiling the dlopen loading path into
 * libayanami (all use sites).
 */
template <typename PluginType>
class PluginManager {
public:
  using InitFn = std::function<bool(const std::string&,
                                    std::shared_ptr<PluginType>)>;

  /// @brief Load one plugin. Common factory create_extension_module ->
  /// typed cast -> invoke init_fn -> register. Returns false on failure
  /// (.so cannot be opened / type mismatch / init_fn rejects).
  bool loadPlugin(const PluginSpec& spec, InitFn init_fn) {
    if (spec.name.empty() || spec.so_name.empty()) {
      return false;
    }
    auto module = ayanami::ExtensionModule::load_module(spec.so_name);
    if (!module) {
      return false;
    }
    // Typed conversion: only payloads truly derived from PluginType are
    // accepted; after conversion the reference count is shared with
    // ExtensionModule::Ptr (ownership retained).
    std::shared_ptr<PluginType> plugin =
        std::dynamic_pointer_cast<PluginType>(std::move(module));
    if (!plugin) {
      return false;
    }
    if (init_fn && !init_fn(spec.name, plugin)) {
      return false;
    }
    plugins[spec.name] = std::move(plugin);
    so_names[spec.name] = spec.so_name;
    return true;
  }

  /// @brief Read name/so_name from a sub-config json and then loadPlugin.
  /// @note No config injection: the plugin's own parameters are read
  /// explicitly by the plugin at construct/initialize time via
  /// GlobalConfig::getConfigPath (glim-style, no thread_local).
  /// @param default_name Fallback logical name when the config lacks name.
  bool loadSingleFromConfig(const Config& cfg, const std::string& module_name,
                             const std::string& default_name, InitFn init_fn) {
    PluginSpec spec;
    spec.name = cfg.param<std::string>(module_name, "name", default_name);
    spec.so_name = cfg.param<std::string>(module_name, "so_name", "");
    return loadPlugin(spec, std::move(init_fn));
  }

  /// @brief Get a plugin; returns nullptr if not loaded.
  std::shared_ptr<PluginType> getPlugin(const std::string& name) const {
    auto iter = plugins.find(name);
    return iter == plugins.end() ? nullptr : iter->second;
  }

  bool hasPlugin(const std::string& name) const {
    return plugins.find(name) != plugins.end();
  }

  /// @brief List of loaded plugin names (map-order traversal, sorted by name).
  std::vector<std::string> getLoadedNames() const {
    std::vector<std::string> names;
    names.reserve(plugins.size());
    for (const auto& kv : plugins) {
      names.push_back(kv.first);
    }
    return names;
  }

  /// @brief Release all plugin objects (dlopen handles are never dlclosed, /§4.5).
  void clearPlugins() {
    plugins.clear();
    so_names.clear();
  }

  /// @brief Return a plugin's source .so; empty string if not loaded.
  std::string getSoName(const std::string& name) const {
    auto iter = so_names.find(name);
    return iter == so_names.end() ? std::string() : iter->second;
  }

private:
  std::map<std::string, std::shared_ptr<PluginType>> plugins;
  std::map<std::string, std::string> so_names;
};

} // namespace components
} // namespace ayanami
