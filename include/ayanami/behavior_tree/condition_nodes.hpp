#pragma once

#include "ayanami/behavior_tree/blackboard_protocol.hpp"
#include "ayanami/behavior_tree/runtime_lookup.hpp"
#include "ayanami/messages/failure.hpp"
#include "ayanami/messages/geometry_msgs.hpp"
#include "ayanami/messages/nav_msgs.hpp"

#include <behaviortree_cpp/condition_node.h>

#include <spdlog/spdlog.h>

#include <cstdint>

namespace ayanami {
namespace behavior_tree {

/**
 * @brief Path validity condition.
 *
 * With a NavigationRuntime injected (resident-tree assembly) this runs the
 * map family's real validation (NavigationRuntime::isPathValid ->
 * CostmapMapContext::isPathValid: in bounds and not lethal/unknown on the
 * global+local overlay); without an injection (pure-blackboard unit tests)
 * it degrades to a non-empty check.
 *
 * Before returning FAILURE the node writes a structured FailureContext
 * (PATH_INVALID) so the surrounding RecoveryNode can classify the failure.
 */
class IsPathValid : public BT::ConditionNode {
 public:
  IsPathValid(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config),
      nav_server(lookupNavigationRuntime(config)) {}

  static BT::PortsList providedPorts() {
    return { BT::InputPort<nav_msgs::Path>("path", "path to validate") };
  }

  BT::NodeStatus tick() override {
    nav_msgs::Path path;
    if (!getInput("path", path) || path.poses.empty()) {
      writeFailureContext(ErrorCode::PATH_INVALID, "path port missing or empty");
      return BT::NodeStatus::FAILURE;
    }
    if (nav_server && !nav_server->isPathValid(path)) {
      spdlog::warn("[bt] IsPathValid FAIL: server rejected the path (node={})",
                   name());
      writeFailureContext(ErrorCode::PATH_INVALID, "path rejected by map family");
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
  }

 private:
  /// @brief Write a minimal failure context (session/version filled from the
  /// runtime when available) so RecoveryNode can classify the cause.
  void writeFailureContext(ErrorCode code, const std::string& message) {
    if (!config().blackboard) {
      return;
    }
    FailureContext ctx;
    ctx.stage = FailureStage::Planner;
    ctx.code = code;
    ctx.message = message;
    if (nav_server) {
      ctx.session_id = nav_server->currentSessionId();
      ctx.plan_version = nav_server->currentPlanVersion();
    }
    config().blackboard->set<FailureContext>(k_failure_context, ctx);
  }

  components::NavigationRuntime::Ptr nav_server;
};

/**
 * @brief Goal update condition: SUCCESS when the goal port changed between
 * ticks, FAILURE otherwise.
 *
 * The first tick only establishes the baseline (init) and is never reported
 * as an update.
 */
class GoalUpdated : public BT::ConditionNode {
 public:
  GoalUpdated(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config), initialized(false) {}

  static BT::PortsList providedPorts() {
    return { BT::InputPort<geometry_msgs::PoseStamped>(k_goal, "navigation goal") };
  }

  BT::NodeStatus tick() override {
    geometry_msgs::PoseStamped goal;
    if (!getInput(k_goal, goal)) {
      return BT::NodeStatus::FAILURE;
    }
    if (!initialized) {
      initialized = true;
      last_goal = goal;
      return BT::NodeStatus::FAILURE;
    }
    if (goalChanged(goal)) {
      last_goal = goal;
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }

 private:
  static bool goalChanged(const geometry_msgs::PoseStamped& lhs,
                          const geometry_msgs::PoseStamped& rhs) {
    return !goalEquals(lhs, rhs);
  }

  bool goalChanged(const geometry_msgs::PoseStamped& goal) const {
    return goalChanged(goal, last_goal);
  }

  bool initialized;
  geometry_msgs::PoseStamped last_goal;
};

} // namespace behavior_tree
} // namespace ayanami
