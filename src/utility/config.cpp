#include <filesystem>
#include <fstream>
#include <mutex>

#include <ayanami/utility/config_impl.hpp>

namespace ayanami {

namespace {

std::once_flag g_global_config_once;
std::mutex g_global_config_mutex;
thread_local const Config* g_active_config = nullptr;

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
  config_ = json;
}

Config::~Config() {}

Config::Scope::Scope(const Config& config) : previous_(g_active_config) {
  g_active_config = &config;
}

Config::Scope::~Scope() { g_active_config = previous_; }

const Config* Config::active() { return g_active_config; }

bool Config::has_param(const std::string& module_name,
                       const std::string& param_name) const {
  const auto& json = std::any_cast<const nlohmann::json&>(config_);
  const auto module = json.find(module_name);
  if (module == json.end()) {
    return false;
  }
  return module->find(param_name) != module->end();
}

std::vector<std::string> Config::param_string_or_list(
    const std::string& module_name, const std::string& param_name) const {
  const auto& json = std::any_cast<const nlohmann::json&>(config_);
  const auto module = json.find(module_name);
  if (module == json.end()) {
    return {};
  }
  const auto parameter = module->find(param_name);
  if (parameter == module->end()) {
    return {};
  }
  std::vector<std::string> out;
  if (parameter->is_string()) {
    out.push_back(parameter->get<std::string>());
  } else if (parameter->is_array()) {
    for (const auto& item : *parameter) {
      if (item.is_string()) {
        out.push_back(item.get<std::string>());
      }
    }
  }
  return out;
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
    // 目录: 兜底兼容 config/ 与 configs/ (spec §2.2)
    if (std::filesystem::exists(directory / "config" / "config.json")) {
      directory /= "config";
    } else if (std::filesystem::exists(directory / "configs" / "config.json")) {
      directory /= "configs";
    }
  }

  inst = new GlobalConfig((directory / "config.json").string());
  inst->override_param("global", "config_path", directory.string());
}

GlobalConfig* GlobalConfig::instance(const std::string& config_path,
                                     bool override_path) {
  if (override_path) {
    // 显式重建 (单线程生命周期操作): 删旧实例并重新解析路径
    std::lock_guard<std::mutex> lock(g_global_config_mutex);
    delete inst;
    inst = nullptr;
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

std::string GlobalConfig::get_config_path(const std::string& config_name) {
  const GlobalConfig* config = instance();
  const std::filesystem::path directory =
      config->param<std::string>("global", "config_path", ".");
  const std::filesystem::path filename =
      config->param<std::string>("global", config_name, config_name + ".json");
  return (directory / filename).string();
}

std::vector<std::string> GlobalConfig::get_config_paths(
    const std::string& config_name) {
  const GlobalConfig* config = instance();
  const std::filesystem::path directory =
      config->param<std::string>("global", "config_path", ".");
  const auto filenames =
      config->param<std::vector<std::string>>("global", config_name, {});
  std::vector<std::string> paths;
  paths.reserve(filenames.size());
  for (const auto& filename : filenames) {
    paths.push_back((directory / filename).string());
  }
  return paths;
}

std::vector<std::string> GlobalConfig::get_config_paths(
    const std::string& owner_config, const std::string& config_name) {
  const GlobalConfig* config = instance();
  const std::filesystem::path directory =
      config->param<std::string>("global", "config_path", ".");
  const std::string owner_file =
      config->param<std::string>("global", owner_config, "");

  std::vector<std::string> entries;
  if (!owner_file.empty()) {
    const Config owner((directory / owner_file).string());
    entries = owner.param_string_or_list("modules", config_name);
  }
  if (entries.empty()) {
    // modules 段缺失/无条目: 回退 config 根下 <config_name>.json
    entries.push_back(config_name + ".json");
  }

  std::vector<std::string> paths;
  paths.reserve(entries.size());
  for (const auto& entry : entries) {
    const std::filesystem::path p(entry);
    paths.push_back(p.is_absolute() ? entry : (directory / p).string());
  }
  return paths;
}

std::string GlobalConfig::get_config_path(const std::string& owner_config,
                                          const std::string& config_name) {
  const auto paths = get_config_paths(owner_config, config_name);
  return paths.empty() ? std::string() : paths.front();
}

ComponentConfig::ComponentConfig(const std::string& config_key) {
  config_ = Config::active();
  if (!config_ && !config_key.empty()) {
    owned_ = std::make_unique<Config>(
        GlobalConfig::get_config_path("config_navigation", config_key));
    config_ = owned_.get();
  }
  if (!config_) {
    owned_ = std::make_unique<Config>("");
    config_ = owned_.get();
  }
}

DEFINE_CONFIG_IO_SPECIALIZATION(bool)
DEFINE_CONFIG_IO_SPECIALIZATION(int)
DEFINE_CONFIG_IO_SPECIALIZATION(long)
DEFINE_CONFIG_IO_SPECIALIZATION(double)
DEFINE_CONFIG_IO_SPECIALIZATION(std::string)
DEFINE_CONFIG_IO_SPECIALIZATION(std::vector<std::string>)
DEFINE_CONFIG_IO_SPECIALIZATION(std::vector<std::vector<std::string>>)

DEFINE_CONFIG_IO_SPECIALIZATION(Eigen::Vector2d)
DEFINE_CONFIG_IO_SPECIALIZATION(Eigen::Vector3d)
DEFINE_CONFIG_IO_SPECIALIZATION(Eigen::Vector4d)
DEFINE_CONFIG_IO_SPECIALIZATION(std::vector<Eigen::Vector2d>)
DEFINE_CONFIG_IO_SPECIALIZATION(Eigen::Quaterniond)
DEFINE_CONFIG_IO_SPECIALIZATION(Eigen::Isometry3d)
DEFINE_CONFIG_IO_SPECIALIZATION(std::vector<Eigen::Isometry3d>)

} // namespace ayanami
