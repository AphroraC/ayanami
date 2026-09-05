#pragma once

#include "ayanami/abstract/components/navigation_server_base.hpp"

#include <behaviortree_cpp/tree_node.h>

#include <memory>
#include <string>

namespace ayanami {
namespace behavior_tree {

// 黑板注入键: BT 节点构造期从 blackboard 取 NavigationRuntime (spec
// §6.2 "构造期注入"; registerNodeType 无参 + createTree 时黑板已就绪)。
// 不属于必需板项 (requiredBlackboardEntries 由 pilot/测试显式 set)。
inline constexpr const char* k_navigation_runtime = "navigation_runtime";

/// @brief 同上, 以 NodeConfig 重载 (构造期成员初始化列表用)。
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

/// @brief 构造期从 blackboard 注入 NavigationRuntime; 缺失返回 nullptr
/// (节点 tick 时判空走退化路径, 不 abort)。
inline components::NavigationRuntime::Ptr lookupNavigationRuntime(
    const BT::TreeNode& node) {
  return lookupNavigationRuntime(node.config());
}

}  // namespace behavior_tree
}  // namespace ayanami
