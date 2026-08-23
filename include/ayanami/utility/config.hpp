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
 * The underlying document is stored as `std::any` holding an
 * `nlohmann::json` so that the public header never depends on the heavy
 * JSON header (kept private in config_impl.hpp / src/utility/config.cpp).
 */
class Config {
public:
  class Scope {
   public:
    explicit Scope(const Config& config);
    ~Scope();
    Scope(const Scope&) = delete;
    Scope& operator=(const Scope&) = delete;

   private:
    const Config* previous_{nullptr};
  };

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

  /// @brief 当前插件工厂调用所绑定的配置; 非插件装载上下文返回 nullptr。
  static const Config* active();

  Config(const Config&) = delete;
  Config& operator=(const Config&) = delete;

  /**
   * @brief Check if a parameter exists.
   */
  bool has_param(const std::string& module_name,
                 const std::string& param_name) const;

  /**
   * @brief Get a required parameter (aborts if not found).
   */
  template <typename T>
  T param_cast(const std::string& module_name,
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
   * @brief Read a parameter that is either a single string or an array of
   *        strings (modules 块条目: 单文件族用 string, 多文件族如 recovery 用
   *        array)。不存在或类型不符返回空。
   */
  std::vector<std::string> param_string_or_list(
      const std::string& module_name, const std::string& param_name) const;

  /**
   * @brief Override a parameter value in memory (does not touch the file).
   */
  template <typename T>
  void override_param(const std::string& module_name,
                      const std::string& param_name, const T& value);

protected:
  std::any config_;
};

/**
 * @brief Process-wide singleton bootstrap for the root config directory.
 *
 * Resolves the config directory (see spec §2.2) and writes the resolved
 * directory back into the `global.config_path` parameter, forming a
 * self-bootstrapping loop. Initialization is protected by `std::once_flag`;
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
   * @brief Resolve a named sub-config's file path.
   *        directory + "/" + param("global", config_name, config_name + ".json")
   */
  static std::string get_config_path(const std::string& config_name);
  static std::vector<std::string> get_config_paths(
      const std::string& config_name);

  /**
   * @brief 从 owner_config 指向的配置文件的 "modules" 段解析 config_name 指向
   *        的模块 json 路径。
   *
   * 查找链:
   *   1. global[owner_config] 解析宿主配置文件 (如 config_navigation →
   *      modules/costmap_navigation.json)
   *   2. 宿主文件的 "modules" 段读 config_name (值可为 string 或 string 数组)
   *   3. 条目相对 config_path (config 根目录) 解析
   *   4. modules 段缺失/无条目时回退 config_path/<config_name>.json
   */
  static std::string get_config_path(const std::string& owner_config,
                                     const std::string& config_name);
  static std::vector<std::string> get_config_paths(
      const std::string& owner_config, const std::string& config_name);
};

class ComponentConfig {
 public:
  explicit ComponentConfig(const std::string& config_key = "");
  const Config& get() const { return *config_; }

 private:
  std::unique_ptr<Config> owned_;
  const Config* config_{nullptr};
};

} // namespace ayanami
