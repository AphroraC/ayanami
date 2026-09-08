// navigation_behavior_tree extension module implementation.
// The header holds the node registration helpers and the embedded fallback
// tree; this translation unit wires the config asset (tree XML file +
// recovery role mapping read from the ordered config_recovery family).

#include "navigation_behavior_tree.hpp"

#include <filesystem>
#include <map>

#include <spdlog/spdlog.h>

#include <ayanami/utility/config.hpp>

namespace ayanami {
namespace modules {

void NavigationBehaviorTreeExtension::registerNodes(
    BT::BehaviorTreeFactory& factory) const {
  ayanami::behavior_tree::registerNavigationBehaviorTreeNodes(factory);
}

std::vector<std::string> NavigationBehaviorTreeExtension::registerTrees(
    BT::BehaviorTreeFactory& factory) const {
  // The tree XML is a config asset (navigation.behavior_tree_xml, resolved
  // relative to the config directory); the embedded copy is the fallback so
  // the extension alone is still enough to run.
  std::string xml_file;
  std::string directory;
  if (GlobalConfig::instance()) {
    Config nav_cfg(GlobalConfig::getConfigPath("config_navigation"));
    xml_file = nav_cfg.param<std::string>("navigation", "behavior_tree_xml",
                                          "");
    directory = GlobalConfig::instance()->param<std::string>(
        "global", "config_path", ".");
  }
  bool loaded = false;
  if (!xml_file.empty()) {
    const std::filesystem::path p(xml_file);
    const std::string xml_path =
        p.is_absolute()
            ? xml_file
            : (std::filesystem::path(directory) / xml_file).string();
    std::error_code ec;
    if (std::filesystem::exists(xml_path, ec)) {
      factory.registerBehaviorTreeFromFile(xml_path);
      loaded = true;
    } else {
      spdlog::warn(
          "NavigationBehaviorTreeExtension: behavior_tree_xml '{}' not "
          "found, falling back to the embedded default tree",
          xml_path);
    }
  }
  if (!loaded) {
    factory.registerBehaviorTreeFromText(k_default_tree_xml);
  }
  return {k_default_tree_id};
}

std::vector<const char*>
NavigationBehaviorTreeExtension::requiredBlackboardEntries() const {
  // Tree role entries ("<role>_id") referenced by the default tree.
  return {"clear_costmap_id", "spin_id", "wait_id", "backup_id"};
}

namespace {

/// @brief Role -> plugin logical name mapping from the ordered
/// config_recovery family: each module json declares recovery.role; the
/// tree references plugins by role id ("<role>_id"), never by what the
/// recovery actually does. Returns the mapping in config order.
std::map<std::string, std::string> recoveryRoleMapping() {
  std::map<std::string, std::string> role_to_name;
  if (!GlobalConfig::instance()) {
    return role_to_name;
  }
  Config cfg(GlobalConfig::getConfigPath("config_navigation"));
  std::vector<std::string> recovery_files;
  try {
    recovery_files =
        cfg.param<std::vector<std::string>>("navigation", "config_recovery", {});
  } catch (const std::exception&) {
  }
  if (recovery_files.empty()) {
    const std::string single =
        cfg.param<std::string>("navigation", "config_recovery", "");
    if (!single.empty()) {
      recovery_files.push_back(single);
    }
  }
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

void NavigationBehaviorTreeExtension::provideBlackboardEntries(
    BT::Blackboard::Ptr blackboard) const {
  // Role mapping: the tree references plugins by role id ("<role>_id"),
  // never by what the recovery actually does. Every role key the default
  // tree references is provided (empty string when undeclared) so tree
  // creation never fails on assembly; an undeclared role only surfaces when
  // the BT actually dispatches that recovery (unknown recovery id).
  const auto mapping = recoveryRoleMapping();
  for (const char* role : {"clear_costmap", "spin", "wait", "backup"}) {
    auto it = mapping.find(role);
    if (it != mapping.end()) {
      blackboard->set<std::string>(std::string(role) + "_id", it->second);
    } else {
      blackboard->set<std::string>(std::string(role) + "_id", std::string());
      spdlog::warn(
          "NavigationBehaviorTreeExtension: no recovery declares role '{}'; "
          "the corresponding recovery branch will fail if dispatched",
          role);
    }
  }
}

}  // namespace modules
}  // namespace ayanami
