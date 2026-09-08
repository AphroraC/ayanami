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
