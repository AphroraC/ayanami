#include <filesystem>
#include <fstream>
#include <mutex>
#include <unordered_map>

#include <ayanami/utility/config_impl.hpp>

namespace ayanami {

namespace {

std::once_flag g_global_config_once;
std::mutex g_global_config_mutex;
std::unordered_map<std::string, std::string> g_resolved_config_paths;

} // namespace

GlobalConfig* GlobalConfig::inst = nullptr;

Config::Config(const std::string& config_filename) {
  nlohmann::json json;
  if (!config_filename.empty()) {
    std::ifstream ifs(config_filename);
    if (!ifs) {
      spdlog::error("failed to open {}", config_filename);
    } else {
      try {
        json = nlohmann::json::parse(ifs, nullptr, true, true);
      } catch (const nlohmann::json::parse_error& e) {
        spdlog::error("failed to parse {}: {}", config_filename, e.what());
      }
    }
  }
  config = json;
}

Config::~Config() {}

bool Config::hasParam(const std::string& module_name,
                       const std::string& param_name) const {
  const auto& json = std::any_cast<const nlohmann::json&>(config);
  const auto module = json.find(module_name);
  if (module == json.end()) {
    return false;
  }
  return module->find(param_name) != module->end();
}

GlobalConfig::GlobalConfig(const std::string& config_path)
    : Config(config_path) {}

GlobalConfig::~GlobalConfig() {}

void GlobalConfig::create(const std::string& config_path) {
  if (inst != nullptr) {
    return;
  }

  std::filesystem::path directory = config_path.empty() ? "." : config_path;
  if (std::filesystem::is_regular_file(directory)) {
    // 文件 -> 父目录
    directory = directory.parent_path();
  } else if (!std::filesystem::exists(directory / "config.json")) {
    // 目录: 兜底兼容 config/ 与 configs/
    if (std::filesystem::exists(directory / "config" / "config.json")) {
      directory /= "config";
    } else if (std::filesystem::exists(directory / "configs" / "config.json")) {
      directory /= "configs";
    }
  }

  inst = new GlobalConfig((directory / "config.json").string());
  inst->overrideParam("global", "config_path", directory.string());
}

GlobalConfig* GlobalConfig::instance(const std::string& config_path,
                                     bool override_path) {
  if (override_path) {
    // 显式重建 (单线程生命周期操作): 删旧实例并重新解析路径
    std::lock_guard<std::mutex> lock(g_global_config_mutex);
    delete inst;
    inst = nullptr;
    g_resolved_config_paths.clear();
    create(config_path);
    return inst;
  }

  // 常规惰性路径: std::call_once 保证首个初始化仅执行一次且线程安全
  std::call_once(g_global_config_once, [&config_path]() {
    std::lock_guard<std::mutex> lock(g_global_config_mutex);
    create(config_path);
  });
  return inst;
}

std::string GlobalConfig::getConfigPath(const std::string& config_name) {
  {
    std::lock_guard<std::mutex> lock(g_global_config_mutex);
    const auto it = g_resolved_config_paths.find(config_name);
    if (it != g_resolved_config_paths.end()) {
      return it->second;
    }
  }
  const GlobalConfig* config = instance();
  const std::filesystem::path directory =
      config->param<std::string>("global", "config_path", ".");
  std::string filename;
  if (config->hasParam("global", config_name)) {
    try {
      filename = config->param<std::string>(
          "global", config_name, config_name + ".json");
    } catch (const std::exception&) {
      const auto files = config->param<std::vector<std::string>>(
          "global", config_name, {});
      filename = files.empty() ? config_name + ".json" : files.front();
    }
  } else {
    // Internal migration lookup. The public schema keeps plugin file pointers
    // in navigation.json; legacy modules may still ask for their historical
    // config_* key while they are being migrated to injected configuration.
    const std::string navigation_file = config->param<std::string>(
        "global", "config_navigation", "");
    if (!navigation_file.empty()) {
      const auto nav_path = std::filesystem::path(navigation_file);
      const auto resolved = nav_path.is_absolute()
                                ? nav_path
                                : (directory / nav_path);
      Config navigation(resolved.string());
      const std::string key =
          config_name.rfind("config_", 0) == 0 ? config_name : "";
      if (!key.empty() && navigation.hasParam("navigation", key)) {
        try {
          filename = navigation.param<std::string>("navigation", key, "");
        } catch (const std::exception&) {
        }
      }
    }
    if (filename.empty()) filename = config_name + ".json";
  }
  return (directory / filename).string();
}

void GlobalConfig::registerConfigPath(const std::string& config_name,
                                      const std::string& path) {
  std::lock_guard<std::mutex> lock(g_global_config_mutex);
  g_resolved_config_paths[config_name] = path;
}

DEFINE_CONFIG_IO_SPECIALIZATION(bool)
DEFINE_CONFIG_IO_SPECIALIZATION(int)
DEFINE_CONFIG_IO_SPECIALIZATION(double)
DEFINE_CONFIG_IO_SPECIALIZATION(std::string)
DEFINE_CONFIG_IO_SPECIALIZATION(std::vector<std::string>)
DEFINE_CONFIG_IO_SPECIALIZATION(std::vector<std::vector<std::string>>)

DEFINE_CONFIG_IO_SPECIALIZATION(Eigen::Vector2d)
DEFINE_CONFIG_IO_SPECIALIZATION(Eigen::Vector3d)
DEFINE_CONFIG_IO_SPECIALIZATION(std::vector<Eigen::Vector2d>)
DEFINE_CONFIG_IO_SPECIALIZATION(Eigen::Isometry3d)
DEFINE_CONFIG_IO_SPECIALIZATION(std::vector<Eigen::Isometry3d>)

} // namespace ayanami
