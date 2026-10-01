// navigation_behavior_tree extension module implementation.
// The header holds the node registration helpers and the embedded fallback
// tree; this translation unit wires the config asset (tree XML file +
// recovery role mapping read from the ordered config_recovery family).

#include "navigation_behavior_tree.hpp"

#include <filesystem>
#include <map>

#include <spdlog/spdlog.h>
#include <ayanami/utility/logging.hpp>

#include <ayanami/utility/config.hpp>

namespace ayanami {
namespace modules {

void NavigationBehaviorTreeExtension::registerNodes(
    BT::BehaviorTreeFactory& factory) const {
  ayanami::behavior_tree::registerNavigationBehaviorTreeNodes(factory);
}

std::vector<std::string> NavigationBehaviorTreeExtension::registerTrees(
    BT::BehaviorTreeFactory& factory) const {
  // The tree XML is the extension's only source of navigation policy: it is a
  // config asset (navigation.behavior_tree_xml, resolved relative to the config
  // directory) and it must exist. The extension used to keep a copy embedded as a
  // fallback, which made a missing deployment file invisible -- the stack would
  // come up running a policy that no file on disk described, and the embedded
  // copy could drift from the shipped one without anything noticing. One source,
  // and a missing one is a startup error.
  const Config nav_cfg(GlobalConfig::getConfigPath("config_navigation"));
  const std::string xml_file =
      nav_cfg.param<std::string>("navigation", "behavior_tree_xml", "");
  if (xml_file.empty()) {
    throw std::runtime_error(
        "NavigationBehaviorTreeExtension: navigation.behavior_tree_xml is not "
        "configured; there is no built-in default tree to fall back on");
  }
  const std::filesystem::path path(xml_file);
  const std::string directory = GlobalConfig::instance()->param<std::string>(
      "global", "config_path", ".");
  const std::string resolved =
      path.is_absolute() ? xml_file : (std::filesystem::path(directory) / xml_file).string();
  std::error_code ec;
  if (!std::filesystem::exists(resolved, ec)) {
    throw std::runtime_error(
        "NavigationBehaviorTreeExtension: behavior_tree_xml '" + resolved +
        "' does not exist");
  }
  factory.registerBehaviorTreeFromFile(resolved);
  SPDLOG_LOGGER_INFO(ayanami::btLogger(), "NavigationBehaviorTreeExtension: registered tree(s) from {}", resolved);
  return {k_default_tree_id};
}

namespace {

/// @brief The tree's recovery role list: config-owned
/// (navigation.recovery_roles, single string or list). The default keeps
/// the costmap-family ladder for backward compatibility.
std::vector<std::string> configuredRoles() {
  std::vector<std::string> roles;
  if (!GlobalConfig::instance()) {
    return roles;
  }
  Config cfg(GlobalConfig::getConfigPath("config_navigation"));
  roles = cfg.paramStringOrList("navigation", "recovery_roles");
  if (roles.empty()) {
    roles = {"clear_costmap", "spin", "wait", "backup"};
  }
  return roles;
}

/// @brief Role -> module logical name mapping from the ordered
/// config_recovery family: each module json declares recovery.role; the
/// tree references modules by role id ("<role>_id"), never by what the
/// recovery actually does. Returns the mapping in config order.
std::map<std::string, std::string> recoveryRoleMapping() {
  std::map<std::string, std::string> role_to_name;
  if (!GlobalConfig::instance()) {
    return role_to_name;
  }
  Config cfg(GlobalConfig::getConfigPath("config_navigation"));
  // Single string or array of strings: one shared helper.
  const std::vector<std::string> recovery_files =
      cfg.paramStringOrList("navigation", "config_recovery");
  const std::string directory =
      GlobalConfig::instance()->param<std::string>(
          "global", "config_path", ".");
  for (const auto& file : recovery_files) {
    if (file.empty()) {
      continue;
    }
    Config recovery_cfg(std::filesystem::path(file).is_absolute()
                            ? file
                            : (directory + "/" + file));
    const std::string name =
        recovery_cfg.param<std::string>("recovery", "name", "");
    const std::string role =
        recovery_cfg.param<std::string>("recovery", "role", "");
    if (name.empty() || role.empty()) {
      continue;
    }
    role_to_name[role] = name;
  }
  return role_to_name;
}

}  // namespace

std::vector<const char*>
NavigationBehaviorTreeExtension::requiredBlackboardEntries() const {
  // Tree role entries ("<role>_id") referenced by the tree. The role list is
  // config-owned (navigation.recovery_roles, default = the costmap-family
  // ladder) so different families ship different recovery ladders without
  // touching the core; the strings are cached on the extension object (the
  // interface returns const char* and the extension outlives tree creation).
  if (role_id_cache.empty()) {
    for (const auto& role : configuredRoles()) {
      role_id_cache.push_back(role + "_id");
    }
  }
  std::vector<const char*> entries;
  entries.reserve(role_id_cache.size());
  for (const auto& entry : role_id_cache) {
    entries.push_back(entry.c_str());
  }
  return entries;
}

void NavigationBehaviorTreeExtension::provideBlackboardEntries(
    BT::Blackboard::Ptr blackboard) const {
  // Role mapping: the tree references modules by role id ("<role>_id"),
  // never by what the recovery actually does. Every role key the default
  // tree references is provided (empty string when undeclared) so tree
  // creation never fails on assembly; an undeclared role only surfaces when
  // the BT actually dispatches that recovery (unknown recovery id).
  const auto mapping = recoveryRoleMapping();
  for (const auto& role : configuredRoles()) {
    auto it = mapping.find(role);
    if (it != mapping.end()) {
      blackboard->set<std::string>(role + "_id", it->second);
    } else {
      blackboard->set<std::string>(role + "_id", std::string());
      SPDLOG_LOGGER_WARN(
        ayanami::btLogger(),
        "NavigationBehaviorTreeExtension: no recovery declares role '{}'; "
        "the corresponding recovery branch will fail if dispatched",
        role);
    }
  }
}

}  // namespace modules
}  // namespace ayanami
