#pragma once

#include "ayanami/utility/extension_module.hpp"

#include <behaviortree_cpp/blackboard.h>
#include <behaviortree_cpp/bt_factory.h>

#include <string>
#include <vector>

namespace ayanami {
namespace behavior_tree {

/**
 * @brief Behavior tree extension interface (spec requirement: dynamically
 * loaded objects derive from ExtensionModule directly or indirectly).
 *
 * Navigation-related BT node registration and default tree XML are all
 * pushed down into concrete extension .so files (e.g.
 * navigation_behavior_tree), so the BehaviorTreePilot core contains no
 * default navigation XML or policy: the pilot only loads and holds
 * BehaviorTreeExtension(s), calls registerNodes + registerTrees in order
 * during buildTree, and finally createTree with the configured tree_id.
 *
 * Implementation notes:
 * - registerNodes: register this extension's custom BT node types into the
 *   factory (needed by the pilot's tree execution).
 * - registerTrees: register this extension's named trees (XML) into the
 *   factory, returning the list of registered tree ids.
 * - defaultTreeId: fallback tree name when the config gives no explicit
 *   tree_id.
 *
 * Resident-tree discipline (nav2 parity): the pilot builds the tree ONCE
 * and reuses it across navigation attempts; every attempt ends with
 * bt_tree.haltTree() and the next attempt re-baselines at each node's IDLE
 * edge. Nodes registered by an extension MUST therefore be halt-safe:
 * - clear per-node cached state (counters, cursors, caches, blackboard
 *   artifacts) in the halt() override (idempotent -- the pilot halts an
 *   already-halted tree defensively), or
 * - re-establish their baseline on the first tick after a halt
 *   (`status() == BT::NodeStatus::IDLE` edge, nav2 GoalUpdatedCondition
 *   pattern).
 * A node that does neither leaks state into the next attempt -- the same
 * responsibility level nav2 places on its plugin authors.
 */
class BehaviorTreeExtension : public virtual ExtensionModule {
public:
  using Ptr = std::shared_ptr<BehaviorTreeExtension>;

  ~BehaviorTreeExtension() override = default;

  /// @brief Extension name (for logging/debugging).
  virtual std::string name() const = 0;

  /// @brief Register this extension's custom nodes into the factory.
  virtual void registerNodes(BT::BehaviorTreeFactory& factory) const = 0;

  /// @brief Register this extension's trees (XML) into the factory; returns the registered tree ids.
  virtual std::vector<std::string> registerTrees(BT::BehaviorTreeFactory& factory) const = 0;

  /// @brief Fallback tree name when the config gives no explicit tree_id (empty by default = no fallback).
  virtual std::string defaultTreeId() const { return ""; }

  /// @brief Blackboard entries this extension requires to exist before the
  /// tree is created (validated by the pilot after createTree). The core
  /// protocol keeps only navigation-core entries (goal/paths/frequencies/
  /// session/revision/failure context); family- or tree-specific entries
  /// (e.g. recovery role ids) are declared here so the core stays agnostic
  /// of what the recoveries actually do -- BT only orchestrates.
  virtual std::vector<const char*> requiredBlackboardEntries() const { return {}; }

  /// @brief Write this extension's blackboard entries (called once per
  /// extension in buildTree, before createTree). Read your own config via
  /// GlobalConfig (asuka pattern) -- the pilot passes no parameters.
  virtual void provideBlackboardEntries(BT::Blackboard::Ptr blackboard) const { (void)blackboard; }
};

}  // namespace behavior_tree
}  // namespace ayanami