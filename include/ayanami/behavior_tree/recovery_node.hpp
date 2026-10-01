#pragma once

#include "ayanami/behavior_tree/resettable_node.hpp"
#include <behaviortree_cpp/control_node.h>

#include <spdlog/spdlog.h>
#include <ayanami/utility/logging.hpp>

#include "ayanami/behavior_tree/blackboard_protocol.hpp"
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
 * Budget reset: when the budget_reset_token input changes (e.g.
 * {goal_revision} -- a goal revision mid-attempt, or the next attempt's
 * advance), the retry budget resets WITHOUT halting an in-flight recovery.
 * Goal updates therefore never consume the recovery budget (the GoalUpdated
 * short circuit in the recovery branch is free).
 */
class RecoveryNode : public BT::ControlNode, public ResettableNode {
public:
  RecoveryNode(const std::string& name, const BT::NodeConfig& config)
  : BT::ControlNode(name, config),
    max_retries(1),
    retry_count(0),
    recovering(false) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<uint32_t>("number_of_retries", 1u, "number of retries"),
      BT::InputPort<uint64_t>("budget_reset_token", 0u, "revision that resets the retry budget"),
      BT::InputPort<uint32_t>(
        "max_same_cause", 0u,
        "fail after this many consecutive child0 failures with the SAME "
        "cause code (0 = off): a cause that repeats is proven unrecoverable "
        "and the remaining budget is not burned on it"),
      BT::InputPort<FailureContext>(
        k_failure_context,
        FailureContext{},
        "failure cause written by the failed child (classified here, then "
        "consumed by the dispatched Recovery action)"),
    };
  }

  BT::NodeStatus tick() override {
    if (!getInput("number_of_retries", max_retries)) {
      return BT::NodeStatus::FAILURE;
    }
    (void)getInput("max_same_cause", max_same_cause);
    if (consumeResetToken(*this, "budget_reset_token")) {
      retry_count = 0;  // budget refresh; an in-flight recovery keeps running
      // A goal revision re-baselines the same-cause watch too: the revised
      // task may legitimately hit the same code once more.
      same_cause_count = 0;
      last_cause_code = ErrorCode::NONE;
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
      // Same-cause pruning: a nominally-recoverable code that keeps
      // repeating (dead sensor -> MAP_STALE every round) has proven the
      // recovery ladder cannot fix it -- fail now instead of rotating
      // through the remaining budget. Distinct codes restart the watch.
      if (max_same_cause > 0) {
        FailureContext ctx;
        (void)getInput(k_failure_context, ctx);
        if (ctx.code == ErrorCode::NONE) {
          // No cause recorded (an inner gate cleared its context before the
          // failure propagated outward): "no cause" is not a cause -- it
          // must not accumulate as a repeating one (a whole ladder was once
          // pruned on three consecutive code-0 failures).
          same_cause_count = 0;
          last_cause_code = ErrorCode::NONE;
        } else if (ctx.code == last_cause_code) {
          ++same_cause_count;
        } else {
          same_cause_count = 1;
          last_cause_code = ctx.code;
        }
        if (same_cause_count >= max_same_cause) {
          SPDLOG_LOGGER_WARN(
              ayanami::btLogger(),
              "RecoveryNode '{}': cause code {} repeated {} consecutive "
              "times -- pruning the remaining recovery budget",
              name(), static_cast<int>(ctx.code), same_cause_count);
          haltChildren();
          retry_count = 0;
          same_cause_count = 0;
          last_cause_code = ErrorCode::NONE;
          if (config().blackboard) {
            config().blackboard->set<FailureContext>(k_failure_context,
                                                     FailureContext{});
          }
          return BT::NodeStatus::FAILURE;
        }
      }
      if (retry_count >= max_retries) {
        haltChildren();
        retry_count = 0;
        // Budget exhausted without dispatching a Recovery action: no one
        // else will consume the cause, so clear it here.
        if (config().blackboard) {
          config().blackboard->set<FailureContext>(k_failure_context, FailureContext{});
        }
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

  /// @brief Clear the retry budget (round state). Called from the
  /// tree-level halt (pilot terminal/halt/new-attempt paths); must stay
  /// idempotent. The budget token cache is kept (ResettableNode,
  /// change-detection state; token monotonicity makes the kept cache sound).
  void halt() override {
    BT::ControlNode::halt();
    retry_count = 0;
    recovering = false;
    same_cause_count = 0;
    last_cause_code = ErrorCode::NONE;
  }

private:
  /// @brief Read the failure context written by the failing child (through
  /// the declared port) and classify it. No context (code NONE) counts as
  /// recoverable. When the recovery branch is dispatched the context stays
  /// alive for the Recovery action's cause port; on a short-circuit /
  /// exhausted budget it is cleared so no stale cause leaks onward.
  bool childFailureRecoverable() {
    FailureContext ctx;
    if (!getInput(k_failure_context, ctx)) {
      return true;
    }
    const bool recoverable = isRecoverable(ctx.code);
    if (!recoverable) {
      // Short-circuit: nothing downstream will consume the cause.
      if (config().blackboard) {
        config().blackboard->set<FailureContext>(k_failure_context, FailureContext{});
      }
    }
    return recoverable;
  }

  uint32_t max_retries;
  uint32_t retry_count;
  bool recovering;
  uint32_t max_same_cause{0};
  uint32_t same_cause_count{0};
  ErrorCode last_cause_code{ErrorCode::NONE};
};

}  // namespace behavior_tree
}  // namespace ayanami
