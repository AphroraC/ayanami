// navigation_behavior_tree extension module implementation.
// The header holds the node registration and the embedded default tree; this
// translation unit provides the constexpr symbol definitions and the
// recovery-role blackboard contract (role -> plugin id mapping read from
// config_recovery, moved out of the core pilot).

#include "navigation_behavior_tree.hpp"

#include <filesystem>

#include <ayanami/utility/config.hpp>

namespace ayanami {
namespace modules {

constexpr const char* NavigationBehaviorTreeExtension::k_default_tree_id;
constexpr const char* NavigationBehaviorTreeExtension::k_default_navigation_xml;

std::vector<const char*>
NavigationBehaviorTreeExtension::requiredBlackboardEntries() const {
  using namespace ayanami::behavior_tree;
  return {k_clear_costmap_id, k_spin_id, k_wait_id, k_backup_id};
}

void NavigationBehaviorTreeExtension::provideBlackboardEntries(
    BT::Blackboard::Ptr blackboard) const {
  using namespace ayanami::behavior_tree;
  // Role mapping: each file in the ordered config_recovery family declares
  // recovery.role; the tree references plugins by role id, never by what
  // the recovery actually does.
  std::string clear_costmap_id, spin_id, wait_id, backup_id;
  ayanami::Config cfg(ayanami::GlobalConfig::getConfigPath("config_navigation"));
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
      ayanami::GlobalConfig::instance()->param<std::string>(
          "global", "config_path", ".");
  for (const auto& file : recovery_files) {
    if (file.empty()) {
      continue;
    }
    ayanami::Config recovery_cfg(std::filesystem::path(file).is_absolute()
                                     ? file
                                     : (directory + "/" + file));
    const std::string name =
        recovery_cfg.param<std::string>("recovery", "name", "");
    const std::string role =
        recovery_cfg.param<std::string>("recovery", "role", "");
    if (role == "clear_costmap") clear_costmap_id = name;
    if (role == "spin") spin_id = name;
    if (role == "wait") wait_id = name;
    if (role == "backup") backup_id = name;
  }
  blackboard->set<std::string>(k_clear_costmap_id, clear_costmap_id);
  blackboard->set<std::string>(k_spin_id, spin_id);
  blackboard->set<std::string>(k_wait_id, wait_id);
  blackboard->set<std::string>(k_backup_id, backup_id);
}

}  // namespace modules
}  // namespace ayanami