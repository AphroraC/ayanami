#pragma once

#include "ayanami/behavior_tree/action_nodes.hpp"
#include "ayanami/behavior_tree/blackboard_protocol.hpp"
#include "ayanami/behavior_tree/resettable_node.hpp"
#include "ayanami/messages/failure.hpp"
#include "ayanami/types.hpp"

#include <behaviortree_cpp/decorator_node.h>

#include <spdlog/spdlog.h>
#include <ayanami/utility/logging.hpp>

#include <chrono>
#include <string>

namespace ayanami {
namespace behavior_tree {

/**
 * @brief BT-level deadline decorator -- the watchdog for slow module calls.
 *
 * Why the BT layer: ExecutionBase has NO execution thread (the action worker
 * runs runBlocking() inline), so a module call that never returns occupies
 * that action's worker forever and no execution-layer watchdog can interrupt
 * it. The module-side cooperative checkpoints (astar cancel poll, minco
 * deadline, minsnap whole-call budget) are the first line; this decorator is
 * the second: on timeout it HALTS the child subtree (the action node's
 * onHalted cancels the in-flight task cooperatively) and fails with a
 * structured TIMEOUT FailureContext, so the surrounding RecoveryNode walks
 * the normal clear-and-retry/budget path -- the attempt stays bounded even
 * when a module never returns.
 *
 * Clock semantics: the clock starts when the child first returns RUNNING and
 * stops at any terminal child status. Wrapping a RateController-gated branch
 * (the production placement) therefore gives one budget window PER RATE
 * ROUND: between rounds the RateController returns its cached terminal
 * status, which stops the clock. The reset_token ({goal_revision}) clears
 * the clock on a goal revision, so the revised task gets a fresh window.
 * msec = 0 disables the decorator (pure passthrough; also the port-default
 * fallback when the blackboard key is absent).
 */
class Deadline : public BT::DecoratorNode, public ResettableNode {
public:
  Deadline(const std::string& name, const BT::NodeConfig& config)
  : BT::DecoratorNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<unsigned>(
          "msec", 0u,
          "per-round deadline for the child [ms]; 0 = unlimited"),
      BT::InputPort<uint64_t>("reset_token", 0u,
                              "revision that restarts the clock"),
      BT::BidirectionalPort<FailureContext>(
          k_failure_context, FailureContext{},
          "the child's failure context (read, then overwritten with TIMEOUT "
          "on a deadline hit)"),
    };
  }

  BT::NodeStatus tick() override {
    if (consumeResetToken(*this, "reset_token")) {
      clock_started = false;
    }
    unsigned msec = 0;
    (void)getInput("msec", msec);

    const BT::NodeStatus status = child_node_->executeTick();
    if (status != BT::NodeStatus::RUNNING) {
      // Terminal child status: stop the clock and propagate as-is. Between
      // rate rounds (RateController returns its cached SUCCESS) this is the
      // path that re-arms a fresh window for the next round.
      clock_started = false;
      return status;
    }

    const auto now = std::chrono::steady_clock::now();
    if (!clock_started) {
      clock_started = true;
      deadline_at = now + std::chrono::milliseconds(msec);
      return BT::NodeStatus::RUNNING;
    }
    if (msec > 0 && now >= deadline_at) {
      SPDLOG_LOGGER_WARN(
          ayanami::btLogger(),
          "Deadline: child of '{}' exceeded {} ms -- halting it and failing "
          "with TIMEOUT",
          name(), msec);
      // Halting the child cancels its in-flight task (cooperative); the
      // module is expected to honor it -- a module that does not is caught
      // by ActionBase's bounded shutdown instead.
      haltChild();
      clock_started = false;
      writeTimeoutContext(msec);
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }

  /// @brief Clear the clock (round state). Called from the tree-level halt
  /// (pilot terminal/halt/new-attempt paths); idempotent. The reset_token
  /// cache is kept (ResettableNode change-detection state).
  void halt() override {
    BT::DecoratorNode::halt();
    clock_started = false;
  }

private:
  /// @brief Overwrite the failure context with TIMEOUT so the RecoveryNode
  /// classification and the dispatched Recovery's cause see the deadline.
  /// The child was RUNNING (it wrote no context of its own); the stage is
  /// left Unknown (the decorator cannot know the child's stage generically).
  void writeTimeoutContext(unsigned msec) {
    FailureContext ctx;
    (void)getInput(k_failure_context, ctx);
    ctx.code = ErrorCode::TIMEOUT;
    ctx.message = "child exceeded its " + std::to_string(msec) + " ms deadline";
    writeFailureContext(*this, ctx);
  }

  bool clock_started{false};
  std::chrono::steady_clock::time_point deadline_at{};
};

}  // namespace behavior_tree
}  // namespace ayanami
