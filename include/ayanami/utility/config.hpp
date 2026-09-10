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

  /// Copyable and movable (ModuleContext carries the config by value; the
  /// json held in std::any is deep-copied).
  /// The GlobalConfig subclass has a private constructor; the singleton's
  /// non-copyable/non-movable nature is unaffected.
  Config(const Config&) = default;
  Config& operator=(const Config&) = default;
  Config(Config&&) = default;
  Config& operator=(Config&&) = default;

  /**
   * @brief Check if a parameter exists.
   */
  bool hasParam(const std::string& module_name,
                 const std::string& param_name) const;

  /**
   * @brief Check if a top-level section exists.
   */
  bool hasSection(const std::string& section) const;

  /**
   * @brief Parameter names present in a section (empty when it is absent).
   */
  std::vector<std::string> keys(const std::string& section) const;

  /**
   * @brief Top-level section names present in the document.
   */
  std::vector<std::string> sections() const;

  /**
   * @brief Get an optional parameter (std::nullopt if not found).
   *
   * A value of the wrong JSON type is reported as "not found" (nullopt) with an
   * error log, not as an exception: every call site reads parameters with a
   * default, so a type mismatch must degrade exactly like a missing key instead
   * of escaping through the module's configure/load path.
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
   * @brief Get a REQUIRED parameter.
   * @throws std::runtime_error when the key is absent or has the wrong type.
   *         Aborting was the previous behaviour; throwing lets the startup path
   *         report the whole configuration problem and exit cleanly.
   */
  template <typename T>
  T paramCast(const std::string& module_name,
              const std::string& param_name) const;

  /**
   * @brief Read a value that may be written either as a single string or as an
   * array of strings, returning the list form.
   *
   * Several config keys (the module-file pointers, the BT extension list, the
   * recovery family) accept both spellings, and the same
   * try-vector-then-string dance used to be hand-written at every one of those
   * call sites.
   *
   * @return the list; a single string yields a one-element list; an absent key
   *         yields an empty list.
   */
  std::vector<std::string> paramStringOrList(const std::string& module_name,
                                             const std::string& param_name) const;

  /**
   * @brief Override a parameter value in memory (does not touch the file).
   */
  template <typename T>
  void overrideParam(const std::string& module_name,
                      const std::string& param_name, const T& value);

  // ---- Parameter audit ----

  /**
   * @brief Keys in `section` that no param()/paramCast() call has read.
   *
   * The widest usability hole in a config-driven framework is the silent
   * default: a misspelled key behaves exactly like an intentional omission, so
   * nothing tells you the setting never took effect. Comparing what the file
   * declares against what was actually read turns that into a report.
   *
   * Reads are recorded per Config instance, so the caller must ask the same
   * instance the module read from (ModuleContext carries its own copy).
   */
  std::vector<std::string> unusedKeys(const std::string& section) const;

  /// @brief True if param()/paramCast() has read this key from this instance.
  bool wasConsumed(const std::string& module_name,
                   const std::string& param_name) const;

  /// @brief "section.key" for everything read from this instance (diagnostics).
  std::vector<std::string> consumedKeys() const;

 protected:
  std::any config;

  /// @brief Record a successful key lookup (see unusedKeys).
  /// @note Mutated on read: the audit is filled in during single-threaded
  ///       startup loading, not from the navigation hot path.
  void recordConsumed(const std::string& module_name,
                      const std::string& param_name) const;

 private:
  mutable std::vector<std::string> consumed;
};

/**
 * @brief Report parameters in `section` that nothing read.
 *
 * Warns by default. Throwing is opt-in via `global.strict_params = true`, so a
 * deployment can adopt the stricter setting once its configs are clean -- the
 * two-step rollout the audit is designed around. `owner` labels the report
 * (a module name, a family name, ...).
 *
 * @return the unused keys, so callers (tests) can assert on them.
 */
std::vector<std::string> auditUnusedParameters(const Config& config,
                                               const std::string& section,
                                               const std::string& owner);

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
};

} // namespace ayanami
