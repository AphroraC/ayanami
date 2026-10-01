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
 * Scope: the WITHIN-attempt reset (a goal revision while the tree is
 * running). On the pilot's resident tree the token caches survive across
 * navigation attempts by design: tokens ({goal_revision},
 * {raw_path_revision}) are monotonic for the process life -- the pilot
 * never resets them -- so a kept cache still detects every change, at a
 * revision change or an attempt boundary alike.
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
