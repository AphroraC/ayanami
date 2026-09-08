#pragma once

#include "ayanami/behavior_tree/action_nodes.hpp"
#include "ayanami/behavior_tree/blackboard_protocol.hpp"
#include "ayanami/behavior_tree/condition_nodes.hpp"
#include "ayanami/behavior_tree/pipeline_sequence.hpp"
#include "ayanami/behavior_tree/navigation_node_state.hpp"
#include "ayanami/behavior_tree/rate_controller.hpp"
#include "ayanami/behavior_tree/recovery_node.hpp"

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/control_node.h>

#include <cstddef>
#include <string>

namespace ayanami {
namespace behavior_tree {

/**
 * @brief RoundRobin (retained node): polls children with a memory cursor.
 *
 * - Current child RUNNING -> overall RUNNING, cursor stays (next tick ticks
 *   it again);
 * - Current child SUCCESS -> cursor moves to the next (mod count) and the
 *   node returns SUCCESS;
 * - Current child FAILURE -> move to the next; if all were tried and still
 *   FAILURE -> overall FAILURE.
 *
 * reset_token (resident-tree reset protocol): when the token changes (a new
 * navigation round's goal_revision advance), reset the cursor and halt
 * in-flight children -- recovery polling positions do not carry across
 * goals (nav2 avoids this naturally via full tree rebuilds; a resident tree
 * must reset explicitly).
 */
class RoundRobin : public BT::ControlNode,
                    public NavigationTreeNodeState,
                    public ResettableNode {
 public:
  RoundRobin(const std::string& name, const BT::NodeConfig& config)
    : BT::ControlNode(name, config), current_child_idx(0) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<uint64_t>("reset_token", 0,
                              "revision that resets the round-robin cursor"),
    };
  }

  BT::NodeStatus tick() override {
    if (consumeResetToken(*this, "reset_token")) {
      haltChildren();  // cancel in-flight recoveries (no-op for IDLE children)
      current_child_idx = 0;
    }
    if (childrenCount() == 0) {
      return BT::NodeStatus::FAILURE;
    }
    while (current_child_idx < childrenCount()) {
      const BT::NodeStatus status =
          children_nodes_[current_child_idx]->executeTick();
      if (status == BT::NodeStatus::RUNNING) {
        return status;
      }
      if (status == BT::NodeStatus::SUCCESS) {
        current_child_idx = (current_child_idx + 1) % childrenCount();
        resetChildren();
        return BT::NodeStatus::SUCCESS;
      }
      // FAILURE -> next child
      ++current_child_idx;
    }
    current_child_idx = 0;
    return BT::NodeStatus::FAILURE;
  }

  void halt() override {
    BT::ControlNode::halt();
    current_child_idx = 0;
  }

  /// @brief Logical reset after a tree-life terminal state (v4 halt only propagates to RUNNING children).
  void resetState() override { current_child_idx = 0; }

 private:
  size_t current_child_idx;
  uint64_t reset_token{0};
};

/// Tries recovery children in declaration order. A successful recovery returns
/// SUCCESS to its parent so the surrounding RecoveryNode retries navigation;
/// a failed recovery advances to the next child.
///
/// reset_token (resident-tree reset protocol): when the token changes,
/// reset the cursor and halt in-flight children; a new navigation round
/// restarts from the first recovery (the previous cursor is not carried
/// over).
class RecoverySequence : public BT::ControlNode,
                       public NavigationTreeNodeState,
                       public ResettableNode {
 public:
  RecoverySequence(const std::string& name, const BT::NodeConfig& config)
      : BT::ControlNode(name, config), current_child_idx(0) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<uint64_t>("reset_token", 0,
                              "revision that resets the recovery cursor"),
    };
  }

  BT::NodeStatus tick() override {
    if (consumeResetToken(*this, "reset_token")) {
      haltChildren();
      current_child_idx = 0;
    }
    while (current_child_idx < childrenCount()) {
      const auto status = children_nodes_[current_child_idx]->executeTick();
      if (status == BT::NodeStatus::RUNNING) return status;
      if (status == BT::NodeStatus::SUCCESS) {
        haltChildren();
        current_child_idx = 0;
        return BT::NodeStatus::SUCCESS;
      }
      if (status != BT::NodeStatus::FAILURE) return status;
      children_nodes_[current_child_idx]->haltNode();
      ++current_child_idx;
    }
    haltChildren();
    current_child_idx = 0;
    return BT::NodeStatus::FAILURE;
  }

  void halt() override {
    BT::ControlNode::halt();
    current_child_idx = 0;
  }

  /// @brief Logical reset after a tree-life terminal state (v4 halt only propagates to RUNNING children).
  void resetState() override { current_child_idx = 0; }

 private:
  std::size_t current_child_idx;
  uint64_t reset_token{0};
};

/// @brief Full logical reset after a tree-life terminal state (the pilot
/// calls it when a tick returns non-RUNNING).
///
/// BT.CPP v4 changed the halt semantics: ControlNode::haltChildren only
/// calls haltNode on children with status==RUNNING; subtrees cached in a
/// terminal state (SUCCESS/FAILURE) are not reset -- in the resident-tree
/// model the planner/smoother RateController caches, RecoveryNode retry
/// counters, and RecoverySequence/RoundRobin cursors would leak across tree
/// lives, causing "planner never replans + recovery storm" (captured with a
/// rosbag in rviz field tests). This function is the equivalent of nav2's
/// (BT.CPP v3) strong halt semantics: reset each node's internal state
/// without touching the BT state machine (executeTick already auto-resets).
inline void resetNavigationTreeState(BT::Tree& tree) {
  tree.applyVisitor([](BT::TreeNode* node) {
    if (auto* n = dynamic_cast<NavigationTreeNodeState*>(node)) {
      n->resetState();
    }
    // v3-style strong halt for the tree-life boundary: BT.CPP v4 returns a
    // terminal node's cached SUCCESS/FAILURE without re-ticking it, so every
    // non-IDLE node must go back to IDLE or the next tree life never
    // re-executes it (root cause of "planner never replans"). At a tree-life
    // terminal state no child is RUNNING, so no in-flight onHalted work is
    // lost by resetting the status directly.
    if (node->status() != BT::NodeStatus::IDLE) {
      node->haltNode();
    }
  });
}

/// @brief Register the navigation BT node classes (retained nodes +
/// control/condition nodes).
/// Action nodes receive NavigationRuntime::Ptr via the blackboard
/// (action_nodes.hpp).
///
/// Note: the default navigation tree XML and whole-tree registration have
/// been pushed down into the navigation_behavior_tree extension module
/// (BehaviorTreeExtension); this function only registers node types, reused
/// by the extension module's registerNodes and by tests. The core
/// (BehaviorTreePilot) contains no default navigation XML or policy.
inline void registerNavigationBehaviorTreeNodes(BT::BehaviorTreeFactory& factory) {
  // 4 action nodes (TaskHandle direct)
  factory.registerNodeType<ComputePathToPoseNode>("ComputePathToPose");
  factory.registerNodeType<SmoothPathNode>("SmoothPath");
  factory.registerNodeType<FollowPathNode>("FollowPath");
  factory.registerNodeType<RecoveryActionNode>("Recovery");
  // condition nodes
  factory.registerNodeType<IsPathValid>("IsPathValid");
  factory.registerNodeType<GoalUpdated>("GoalUpdated");
  // Control nodes (+ RoundRobin)
  factory.registerNodeType<PipelineSequence>("PipelineSequence");
  factory.registerNodeType<RateController>("RateController");
  factory.registerNodeType<RecoveryNode>("RecoveryNode");
  factory.registerNodeType<RoundRobin>("RoundRobin");
  factory.registerNodeType<RecoverySequence>("RecoverySequence");
}

} // namespace behavior_tree
} // namespace ayanami
