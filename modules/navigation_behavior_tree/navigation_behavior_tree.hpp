// navigation_behavior_tree extension module header.
//
// The default navigation tree (XML) + all custom navigation BT node registrations sink to this module: the core
// BehaviorTreePilot contains no default navigation XML/policies; it only loads and holds this extension.

#pragma once

#include "ayanami/behavior_tree/extension.hpp"
#include "ayanami/behavior_tree/navigation.hpp"

#include <string>
#include <vector>

namespace ayanami {
namespace modules {

/**
 * @brief Default navigation BT extension (derives BehaviorTreeExtension).
 *
 * - registerNodes: registers all custom navigation nodes (ComputePathToPose / SmoothPath /
 *   FollowPath / Recovery / conditions / control nodes, via the core registerNavigationBehaviorTreeNodes).
 * - registerTrees: registers the navigation tree XML. The XML is a config asset
 *   (navigation.behavior_tree_xml in the navigation config, resolved relative to
 *   the config directory) and is the ONLY source of navigation policy: there is
 *   no embedded fallback, and a missing asset is a startup error.
 * - defaultTreeId: "NavigateWithReplanningAndSmoothing".
 */
class NavigationBehaviorTreeExtension
    final : public ayanami::behavior_tree::BehaviorTreeExtension {
 public:
  std::string name() const override { return "navigation"; }

  void registerNodes(BT::BehaviorTreeFactory& factory) const override;

  std::vector<std::string> registerTrees(
      BT::BehaviorTreeFactory& factory) const override;

  std::string defaultTreeId() const override { return k_default_tree_id; }

  // Recovery role ids are a tree-level contract (this tree references them
  // by role), declared and provided here -- the core pilot stays agnostic
  // of what the recoveries do.
  std::vector<const char*> requiredBlackboardEntries() const override;

  void provideBlackboardEntries(BT::Blackboard::Ptr blackboard) const override;

 private:
  static constexpr const char* k_default_tree_id =
      "NavigateWithReplanningAndSmoothing";

};

}  // namespace modules
}  // namespace ayanami
