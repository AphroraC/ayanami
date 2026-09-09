#pragma once

namespace ayanami {
namespace behavior_tree {

/**
 * @brief Common state-reset interface for navigation tree nodes.
 *
 * The resident-tree model needs a logical reset at every tree-life terminal
 * state: BT.CPP v4's halt only propagates to RUNNING children, so subtrees
 * cached in a terminal state keep their internal caches/counters/cursors
 * (RateController single-shot cache, RecoveryNode retry budget,
 * RecoverySequence/RoundRobin cursors) across tree lives.
 *
 * resetNavigationTreeState() walks the whole tree and calls resetState()
 * through this interface only -- a new node type that implements it is
 * picked up automatically, without touching the walker.
 */
class NavigationTreeNodeState {
 public:
  virtual ~NavigationTreeNodeState() = default;

  /// @brief Logical reset after a tree-life terminal state. Must reset all
  /// per-tree-life internal state, but keep change-detection caches
  /// (e.g. reset_token) that carry semantics across tree lives.
  virtual void resetState() = 0;
};

}  // namespace behavior_tree
}  // namespace ayanami

#include <behaviortree_cpp/tree_node.h>

#include <cstdint>

namespace ayanami {
namespace behavior_tree {

/**
 * @brief Change-detection token mixin shared by the navigation control /
 * decorator nodes (RateController, RecoveryNode, RoundRobin,
 * RecoverySequence).
 *
 * A token input (e.g. {goal_revision}, {plan_version}) carries "the upstream
 * artifact changed; reset this node's cached state". The mixin owns the
 * cached token value and the read-and-compare step, so each node only
 * reacts to consumeResetToken() returning true. The cache is deliberately
 * NOT part of resetState(): token change detection must keep its baseline
 * across tree lives.
 */
class ResettableNode {
 public:
  virtual ~ResettableNode() = default;

 protected:
  /// @brief Read the token port; return true (exactly once) when the value
  /// changed since the previous call. @p node is the BT node owning the
  /// port (usually *this).
  bool consumeResetToken(const BT::TreeNode& node, const char* port) {
    uint64_t token = 0;
    (void)node.getInput(port, token);
    if (token == reset_token) {
      return false;
    }
    reset_token = token;
    return true;
  }

 private:
  uint64_t reset_token{0};
};

}  // namespace behavior_tree
}  // namespace ayanami
