#pragma once

#include <behaviortree_cpp/tree_node.h>

#include <cstdint>

namespace ayanami {
namespace behavior_tree {

/**
 * @brief Change-detection token mixin shared by the navigation control /
 * decorator nodes (RateController, RecoveryNode, RecoverySequence).
 *
 * A token input (e.g. {goal_revision}, {raw_path_revision}) carries "the
 * upstream artifact changed; reset this node's cached state". The mixin owns the
 * cached token value and the read-and-compare step, so each node only reacts to
 * consumeResetToken() returning true.
 *
 * Scope: this is the WITHIN-attempt reset. The pilot builds one tree instance per
 * navigation attempt, so a new attempt needs no reset at all; tokens exist for
 * the case that remains -- the goal being revised while the tree is still
 * running, where the running instance must notice that the task changed.
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
