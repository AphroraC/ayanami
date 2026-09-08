#pragma once

#include "ayanami/abstract/components/navigation_server_base.hpp"

#include <behaviortree_cpp/tree_node.h>

#include <memory>
#include <string>

namespace ayanami {
namespace behavior_tree {

// Blackboard injection key: BT nodes fetch the NavigationRuntime from the
// blackboard at construction time (spec §6.2 "construction-time injection";
// parameterless registerNodeType + the blackboard already populated when
// createTree runs). Not one of the required blackboard entries
// (requiredBlackboardEntries is set explicitly by the pilot/tests).
inline constexpr const char* k_navigation_runtime = "navigation_runtime";

/// @brief Same as above, NodeConfig overload (for construction-time member initializer lists).
inline components::NavigationRuntime::Ptr lookupNavigationRuntime(
    const BT::NodeConfig& config) {
  if (!config.blackboard) {
    return nullptr;
  }
  components::NavigationRuntime::Ptr nav_server{nullptr};
  (void)config.blackboard->get<components::NavigationRuntime::Ptr>(
      k_navigation_runtime, nav_server);
  return nav_server;
}

/// @brief Inject the NavigationRuntime from the blackboard at construction
/// time; returns nullptr when missing (nodes null-check at tick time and
/// take the degraded path, no abort).
inline components::NavigationRuntime::Ptr lookupNavigationRuntime(
    const BT::TreeNode& node) {
  return lookupNavigationRuntime(node.config());
}

}  // namespace behavior_tree
}  // namespace ayanami
