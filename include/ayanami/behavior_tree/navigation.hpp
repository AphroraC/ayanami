#pragma once

#include "ayanami/behavior_tree/action_nodes.hpp"
#include "ayanami/behavior_tree/deadline.hpp"
#include "ayanami/behavior_tree/resettable_node.hpp"
#include "ayanami/behavior_tree/blackboard_protocol.hpp"
#include "ayanami/behavior_tree/condition_nodes.hpp"
#include "ayanami/behavior_tree/pipeline_sequence.hpp"
#include "ayanami/behavior_tree/rate_controller.hpp"
#include "ayanami/behavior_tree/recovery_node.hpp"
#include "ayanami/behavior_tree/selector_nodes.hpp"

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/control_node.h>

#include <algorithm>
#include <cstddef>
#include <set>
#include <string>
#include <vector>

namespace ayanami {
namespace behavior_tree {

/**
 * @brief RecoverySequence (retained node): polls recovery children with a
 * ROTATING memory cursor (nav2 RoundRobin semantics; the production
 * recovery branch -- spin -> wait -> backup rotation).
 *
 * - Current child RUNNING -> overall RUNNING, cursor stays (next tick ticks
 *   it again);
 * - Current child SUCCESS -> cursor moves to the next (mod count) and the
 *   node returns SUCCESS -- the cursor keeps advancing across successes, so
 *   consecutive recoveries rotate instead of repeating the first one;
 * - Current child FAILURE -> move to the next; if all were tried and still
 *   FAILURE -> overall FAILURE.
 * - RESOURCE_BUSY is TRANSIENT: a dispatch rejected because another
 *   recovery still holds the exclusive slot (typically the local branch's
 *   clear-costmap winding down after RecoveryNode's haltChild canceled it).
 *   Burning the whole ladder on it in one tick aborts recoverable
 *   navigations -- instead stay on the same child and retry next tick
 *   (bounded by the outer retry budget and the same-cause pruning).
 *
 * reset_token: when the token changes ({goal_revision} -- a goal revision
 * mid-attempt, or the next attempt's advance), reset the cursor and halt
 * in-flight children -- recovery polling positions do not carry across
 * goals.
 */
class RecoverySequence : public BT::ControlNode, public ResettableNode {
public:
  RecoverySequence(const std::string& name, const BT::NodeConfig& config)
  : BT::ControlNode(name, config),
    current_child_idx(0) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<uint64_t>("reset_token", 0, "revision that resets the recovery cursor"),
      BT::InputPort<ayanami::FailureContext>(
        k_failure_context, ayanami::FailureContext{},
        "the failed child's cause; RESOURCE_BUSY retries in place"),
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
      const BT::NodeStatus status = children_nodes_[current_child_idx]->executeTick();
      if (status == BT::NodeStatus::RUNNING) {
        return status;
      }
      if (status == BT::NodeStatus::SUCCESS) {
        current_child_idx = (current_child_idx + 1) % childrenCount();
        resetChildren();
        return BT::NodeStatus::SUCCESS;
      }
      // FAILURE -> next child, EXCEPT the transient dispatch conflict:
      // RESOURCE_BUSY means the recovery never ran (the exclusive slot was
      // busy) -- retry the SAME child next tick instead of burning the
      // ladder (and, exhausted, the whole navigation) on a scheduling race.
      ayanami::FailureContext ctx;
      if (getInput(k_failure_context, ctx) &&
          ctx.code == ayanami::ErrorCode::RESOURCE_BUSY) {
        return BT::NodeStatus::RUNNING;
      }
      ++current_child_idx;
    }
    current_child_idx = 0;
    return BT::NodeStatus::FAILURE;
  }

  /// @brief Clear the rotation cursor (round state). Called from the
  /// tree-level halt (pilot terminal/halt/new-attempt paths); must stay
  /// idempotent. The reset_token cache is kept (ResettableNode,
  /// change-detection state; token monotonicity makes the kept cache sound).
  void halt() override {
    BT::ControlNode::halt();
    current_child_idx = 0;
  }

private:
  size_t current_child_idx;
};

/// @brief Register the navigation BT node classes (retained nodes +
/// control/condition nodes + module selectors).
/// Action nodes receive NavigationRuntime::Ptr via the blackboard
/// (action_nodes.hpp).
///
/// Note: the default navigation tree XML and whole-tree registration have
/// been pushed down into the navigation_behavior_tree extension module
/// (BehaviorTreeExtension); this function only registers node types, reused
/// by the extension module registerNodes and by tests. The core
/// (BehaviorTreePilot) contains no default navigation XML or policy.
inline void registerNavigationBehaviorTreeNodes(BT::BehaviorTreeFactory& factory) {
  // 4 action nodes (TaskHandle direct) + the smoother-branch degrade node + the smoother-branch degrade node
  factory.registerNodeType<ComputePathToPoseNode>("ComputePathToPose");
  factory.registerNodeType<SmoothPathNode>("SmoothPath");
  factory.registerNodeType<DegradeToRawPathNode>("DegradeToRawPath");
  factory.registerNodeType<FollowPathNode>("FollowPath");
  factory.registerNodeType<RecoveryActionNode>("Recovery");
  // condition nodes
  factory.registerNodeType<IsPathValid>("IsPathValid");
  factory.registerNodeType<GoalUpdated>("GoalUpdated");
  factory.registerNodeType<IsRobotProgressing>("IsRobotProgressing");
  // Control nodes
  factory.registerNodeType<PipelineSequence>("PipelineSequence");
  // BT-level deadline for slow module calls (watchdog second line)
  factory.registerNodeType<Deadline>("Deadline");
  factory.registerNodeType<RateController>("RateController");
  factory.registerNodeType<RecoveryNode>("RecoveryNode");
  // rotating recovery sequence (nav2 RoundRobin semantics, production name)
  factory.registerNodeType<RecoverySequence>("RecoverySequence");
  // module selectors (nav2 PlannerSelector/ControllerSelector analogues;
  // one class under three names, output feeds the *_id action ports)
  factory.registerNodeType<ModuleSelectorNode>("PlannerSelector");
  factory.registerNodeType<ModuleSelectorNode>("ControllerSelector");
  factory.registerNodeType<ModuleSelectorNode>("SmootherSelector");
}

}  // namespace behavior_tree
}  // namespace ayanami
