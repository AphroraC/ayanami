#pragma once
#include <any>
#include <optional>
#include <memory>
#include <string>
#include <vector>

namespace ayanami {

/**
 * @brief JSON-based configuration loader.
 *
 * Mirrors glim's Config value object: constructed from a JSON file path
 * (empty string => empty document), holds the parsed document as `std::any`
 * hiding `nlohmann::json` so the public header never depends on the heavy
 * JSON header (kept private in config_impl.hpp / src/utility/config.cpp).
 *
 * Params are addressed as `module_name / param_name` (top-level JSON object
 * keys). No thread-local "active config", no Scope, no injection: each user
 * explicitly constructs its own Config from GlobalConfig::getConfigPath().
 */
class Config {
public:
  /**
   * @brief Load a configuration from a JSON file.
   * @param config_filename Path to the JSON file.
   *                        Empty string results in an empty config.
   * @note If the file cannot be opened or parsed, an error is logged but the
   *       config is kept as an empty JSON document so `param` falls back to
   *       the default value instead of aborting.
   */
  explicit Config(const std::string& config_filename);
  virtual ~Config();

  Config(const Config&) = delete;
  Config& operator=(const Config&) = delete;

  /// @brief 可移动 (PluginContext 按值携带配置; GlobalConfig 单例不可移动)。
  Config(Config&&) = default;
  Config& operator=(Config&&) = default;

  /**
   * @brief Check if a parameter exists.
   */
  bool hasParam(const std::string& module_name,
                 const std::string& param_name) const;

  /**
   * @brief Get an optional parameter (std::nullopt if not found).
   */
  template <typename T>
  std::optional<T> param(const std::string& module_name,
                         const std::string& param_name) const;

  /**
   * @brief Get a parameter with a default value.
   */
  template <typename T>
  T param(const std::string& module_name, const std::string& param_name,
          const T& default_value) const;

  /**
   * @brief Get a required parameter (aborts if not found).
   */
  template <typename T>
  T param_cast(const std::string& module_name,
               const std::string& param_name) const;

  /**
   * @brief Override a parameter value in memory (does not touch the file).
   */
  template <typename T>
  void overrideParam(const std::string& module_name,
                      const std::string& param_name, const T& value);

protected:
  std::any config;
};

/**
 * @brief Process-wide singleton bootstrap for the root config directory.
 *
 * Mirrors glim's GlobalConfig: holds the main `config.json` document and is
 * the single entry point to resolve sub-config file paths via a flat
 * single-level index in the `global` section
 * (`getConfigPath(name) = directory + "/" + global[name]`, defaulting to
 * `name + ".json"`). Initialization is protected by `std::once_flag`;
 * `override_path=true` explicitly rebuilds the instance (single-threaded
 * lifecycle operation).
 */
class GlobalConfig : public Config {
private:
  explicit GlobalConfig(const std::string& config_path);
  ~GlobalConfig() override;

  static GlobalConfig* inst;
  static void create(const std::string& config_path);

public:
  /**
   * @brief Get or create the global config singleton.
   * @param config_path   Path to a config file or directory (see §2.2).
   * @param override_path If true, destroy the existing instance and rebuild
   *                      from config_path; otherwise reuse the existing one.
   */
  static GlobalConfig* instance(const std::string& config_path = "",
                                bool override_path = false);

  /**
   * @brief Resolve a named sub-config's file path (single-level index).
   *        directory + "/" + param("global", config_name, config_name + ".json")
   */
  static std::string getConfigPath(const std::string& config_name);

  /// Register a resolved module configuration path for the current process.
  /// This keeps module files independent from the root global index.
  static void registerConfigPath(const std::string& config_name,
                                 const std::string& path);
};

} // namespace ayanami
