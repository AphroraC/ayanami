#pragma once

#include <behaviortree_cpp/control_node.h>

#include "ayanami/behavior_tree/blackboard_protocol.hpp"
#include "ayanami/behavior_tree/navigation_node_state.hpp"
#include "ayanami/messages/failure.hpp"

#include <cstdint>

namespace ayanami {
namespace behavior_tree {

/**
 * @brief nav2-style RecoveryNode: 2 children, child0 is the main task and
 * child1 is the recovery.
 *
 * - child0 SUCCESS -> reset the retry count and return SUCCESS;
 * - child0 FAILURE with retries left -> halt child0, tick child1;
 * - child1 SUCCESS -> retries++, reset child0 back to the main task, return
 *   RUNNING so the main task retries;
 * - child1 FAILURE -> overall FAILURE;
 * - child0 FAILURE with retries exhausted -> overall FAILURE.
 *
 * Non-recoverable short-circuit (mbf navigator outcome classification): when
 * child0 fails with a code no recovery can fix (invalid input/goal,
 * persistent transform loss, cancellation -- see isRecoverable()), the
 * recovery branch is skipped and the failure propagates immediately; there
 * is no point burning map clears and retries on such a failure. The failing
 * action node writes the structured FailureContext to the blackboard before
 * returning FAILURE; a missing context (e.g. a bare condition failure) is
 * treated as recoverable (conservative).
 *
 * Budget reset (resident-tree discipline): when the budget_reset_token
 * input changes (e.g. {goal_revision} -- a new goal arrived), the retry
 * budget resets WITHOUT halting an in-flight recovery. Goal updates
 * therefore never consume the recovery budget (the GoalUpdated short
 * circuit in the recovery branch is free).
 */
class RecoveryNode : public BT::ControlNode,
                     public NavigationTreeNodeState,
                     public ResettableNode {
 public:
  RecoveryNode(const std::string& name, const BT::NodeConfig& config)
    : BT::ControlNode(name, config),
      max_retries(1),
      retry_count(0),
      recovering(false) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<uint32_t>("number_of_retries", 1u, "number of retries"),
      BT::InputPort<uint64_t>("budget_reset_token", 0u,
                              "revision that resets the retry budget"),
    };
  }

  BT::NodeStatus tick() override {
    if (!getInput("number_of_retries", max_retries)) {
      return BT::NodeStatus::FAILURE;
    }
    if (consumeResetToken(*this, "budget_reset_token")) {
      retry_count = 0;  // budget refresh; an in-flight recovery keeps running
    }

    if (!recovering) {
      const BT::NodeStatus status = children_nodes_[0]->executeTick();
      if (status == BT::NodeStatus::SUCCESS) {
        retry_count = 0;
        resetChildren();
        return BT::NodeStatus::SUCCESS;
      }
      if (status == BT::NodeStatus::RUNNING) {
        return BT::NodeStatus::RUNNING;
      }
      // child0 FAILURE: skip the recovery branch when the failure code is
      // one no recovery can fix.
      if (!childFailureRecoverable()) {
        haltChildren();
        retry_count = 0;
        return BT::NodeStatus::FAILURE;
      }
      if (retry_count >= max_retries) {
        haltChildren();
        retry_count = 0;
        return BT::NodeStatus::FAILURE;
      }
      haltChild(0);
      recovering = true;
    }

    const BT::NodeStatus status = children_nodes_[1]->executeTick();
    if (status == BT::NodeStatus::SUCCESS) {
      ++retry_count;
      recovering = false;
      resetChildren();
      return BT::NodeStatus::RUNNING;
    }
    if (status == BT::NodeStatus::RUNNING) {
      return BT::NodeStatus::RUNNING;
    }
    // child1 FAILURE
    haltChildren();
    retry_count = 0;
    recovering = false;
    return BT::NodeStatus::FAILURE;
  }

  void halt() override {
    BT::ControlNode::halt();
    retry_count = 0;
    recovering = false;
  }

  /// @brief Logical reset after a tree-life terminal state (v4 halt only
  /// propagates to RUNNING children). The budget token cache is kept
  /// (change-detection semantics, owned by ResettableNode).
  void resetState() override {
    retry_count = 0;
    recovering = false;
  }

 private:
  /// @brief Read the failure context written by the failing child and
  /// classify it. No context (code NONE) counts as recoverable.
  bool childFailureRecoverable() const {
    if (!config().blackboard) {
      return true;
    }
    FailureContext ctx;
    if (!config().blackboard->get<FailureContext>(k_failure_context, ctx)) {
      return true;
    }
    return isRecoverable(ctx.code);
  }

  uint32_t max_retries;
  uint32_t retry_count;
  bool recovering;
};

} // namespace behavior_tree
} // namespace ayanami
