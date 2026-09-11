#include <filesystem>
#include <stdexcept>
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

Config::Config() : config(nlohmann::json::object()) {}

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

void Config::recordConsumed(const std::string& module_name,
                             const std::string& param_name) const {
  const std::string full = module_name + "." + param_name;
  for (const auto& seen : consumed) {
    if (seen == full) {
      return;
    }
  }
  consumed.push_back(full);
}

bool Config::wasConsumed(const std::string& module_name,
                          const std::string& param_name) const {
  const std::string full = module_name + "." + param_name;
  for (const auto& seen : consumed) {
    if (seen == full) {
      return true;
    }
  }
  return false;
}

std::vector<std::string> Config::consumedKeys() const { return consumed; }

std::vector<std::string> Config::unusedKeys(const std::string& section) const {
  std::vector<std::string> unused;
  for (const auto& key : keys(section)) {
    if (!wasConsumed(section, key)) {
      unused.push_back(key);
    }
  }
  return unused;
}

std::vector<std::string> auditUnusedParameters(const Config& config,
                                               const std::string& section,
                                               const std::string& owner) {
  const std::vector<std::string> unused = config.unusedKeys(section);
  if (unused.empty()) {
    return unused;
  }
  std::string joined;
  for (std::size_t i = 0; i < unused.size(); ++i) {
    joined += (i == 0 ? "" : ", ") + unused[i];
  }
  const bool strict =
      config.param<bool>("global", "strict_params", false);
  if (strict) {
    throw std::runtime_error("unused config parameter(s) in section '" +
                             section + "' for " + owner + ": " + joined);
  }
  spdlog::warn(
      "unused config parameter(s) in section '{}' for {}: {} "
      "(typo, or a leftover from an older version; set global.strict_params "
      "to true to make this fatal)",
      section, owner, joined);
  return unused;
}

bool Config::hasSection(const std::string& section) const {
  const auto& json = std::any_cast<const nlohmann::json&>(config);
  return json.find(section) != json.end();
}

Config Config::child(const std::string& section,
                     const std::string& key) const {
  const auto& json = std::any_cast<const nlohmann::json&>(config);
  const auto module = json.find(section);
  if (module == json.end() || !module->is_object()) {
    return Config();
  }
  const auto sub = module->find(key);
  if (sub == module->end() || !sub->is_object()) {
    return Config();
  }
  Config child_config;
  child_config.config = nlohmann::json{{key, *sub}};
  return child_config;
}

std::vector<std::string> Config::keys(const std::string& section) const {
  std::vector<std::string> keys;
  const auto& json = std::any_cast<const nlohmann::json&>(config);
  const auto found = json.find(section);
  if (found == json.end() || !found->is_object()) {
    return keys;
  }
  keys.reserve(found->size());
  for (auto iter = found->begin(); iter != found->end(); ++iter) {
    keys.push_back(iter.key());
  }
  return keys;
}

std::vector<std::string> Config::sections() const {
  std::vector<std::string> sections;
  const auto& json = std::any_cast<const nlohmann::json&>(config);
  if (!json.is_object()) {
    return sections;
  }
  sections.reserve(json.size());
  for (auto iter = json.begin(); iter != json.end(); ++iter) {
    sections.push_back(iter.key());
  }
  return sections;
}

std::vector<std::string> Config::paramStringOrList(
    const std::string& module_name, const std::string& param_name) const {
  // Either spelling is legal (a single string or an array of strings). The
  // dispatch inspects the JSON value's type BEFORE any typed read: probing
  // with param<std::vector<std::string>> first logged a bogus "cannot read as
  // the requested type" error for every plain-string value (the common
  // spelling) and only then fell back to the string -- an error that was
  // really a control-flow probe, spamming the startup log.
  const auto& json = std::any_cast<const nlohmann::json&>(config);
  const auto module = json.find(module_name);
  if (module == json.end() || !module->is_object()) {
    return {};
  }
  const auto value = module->find(param_name);
  if (value == module->end() || value->is_null()) {
    return {};
  }
  // The key exists and was asked for: keep the audit contract identical to
  // param<T> (consumed even when the value turns out to be unusable).
  recordConsumed(module_name, param_name);
  if (value->is_string()) {
    const auto& str = value->get_ref<const std::string&>();
    return str.empty() ? std::vector<std::string>{} : std::vector<std::string>{str};
  }
  if (value->is_array()) {
    try {
      return value->get<std::vector<std::string>>();
    } catch (const std::exception& e) {
      // A malformed array is a genuine config problem worth reporting.
      spdlog::error(
          "param {}/{}: array cannot be read as a string list ({})",
          module_name, param_name, e.what());
      return {};
    }
  }
  // Neither spelling: a genuine type mistake -- report once, loudly.
  spdlog::error("param {}/{}: expected a string or an array of strings",
                module_name, param_name);
  return {};
}

std::string GlobalConfig::getConfigPath(const std::string& config_name) {
  const GlobalConfig* config = instance();
  const std::filesystem::path directory =
      config->param<std::string>("global", "config_path", ".");
  std::string filename = config_name + ".json";
  // The global section's single-level index only resolves root-level keys
  // (config_navigation/config_ros); module file pointers live in
  // config_navigation's navigation section, enumerated by
  // NavigationServerBase::loadModules, bypassing this entry point.
  // A single string or a one-element array are both accepted: one shared helper
  // handles the two spellings (this used to be a hand-written try/catch).
  const std::vector<std::string> configured =
      config->paramStringOrList("global", config_name);
  if (!configured.empty()) {
    filename = configured.front();
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
