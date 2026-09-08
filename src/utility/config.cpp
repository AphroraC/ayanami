#include <filesystem>
#include <fstream>
#include <mutex>
#include <unordered_map>

#include <ayanami/utility/config_impl.hpp>

namespace ayanami {

namespace {

std::once_flag g_global_config_once;
std::mutex g_global_config_mutex;

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
    // File -> parent directory
    directory = directory.parent_path();
  } else if (!std::filesystem::exists(directory / "config.json")) {
    // Directory: fallback compatibility for config/ and configs/
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
    // Explicit rebuild (single-threaded lifecycle operation): delete the old instance and re-resolve the path
    std::lock_guard<std::mutex> lock(g_global_config_mutex);
    delete inst;
    inst = nullptr;
    create(config_path);
    return inst;
  }

  // Regular lazy path: std::call_once guarantees the first initialization runs exactly once and thread-safely
  std::call_once(g_global_config_once, [&config_path]() {
    std::lock_guard<std::mutex> lock(g_global_config_mutex);
    create(config_path);
  });
  return inst;
}

std::string GlobalConfig::getConfigPath(const std::string& config_name) {
  const GlobalConfig* config = instance();
  const std::filesystem::path directory =
      config->param<std::string>("global", "config_path", ".");
  std::string filename = config_name + ".json";
  // The global section's single-level index only resolves root-level keys
  // (config_navigation/config_ros);
  // plugin file pointers live uniformly in config_navigation's navigation
  // section, enumerated directly by NavigationServerBase::loadPlugins,
  // bypassing this entry point.
  if (config->hasParam("global", config_name)) {
    try {
      filename = config->param<std::string>(
          "global", config_name, filename);
    } catch (const std::exception&) {
      const auto files = config->param<std::vector<std::string>>(
          "global", config_name, {});
      if (!files.empty()) filename = files.front();
    }
  }
  return (directory / filename).string();
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
