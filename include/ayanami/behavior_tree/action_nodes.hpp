#pragma once

#include "ayanami/behavior_tree/blackboard_protocol.hpp"
#include "ayanami/behavior_tree/runtime_lookup.hpp"
#include "ayanami/abstract/components/navigation_server_base.hpp"
#include "ayanami/messages/actions.hpp"
#include "ayanami/messages/failure.hpp"
#include "ayanami/messages/geometry_msgs.hpp"
#include "ayanami/messages/nav_msgs.hpp"

#include <behaviortree_cpp/action_node.h>

#include <spdlog/spdlog.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>

namespace ayanami {
namespace behavior_tree {

/// @brief Write the structured failure context to the blackboard (key
/// discipline extension). Called before the node returns FAILURE so that
/// Recovery nodes can read the precise cause.
inline void writeFailureContext(const BT::TreeNode& node,
                                const FailureContext& context) {
  if (node.config().blackboard) {
    node.config().blackboard->set<FailureContext>(k_failure_context, context);
  }
 // Failure observability: FailureContext previously only went to the
 // blackboard (debugging required gdb); log one line here (failures are
 // low-frequency events, warn level does not flood; message is diagnostic
 // only, decisions still look only at code).
  spdlog::warn("[bt] stage={} failure: code={} msg='{}' plugin='{}' "
               "session={} pv={}",
               static_cast<int>(context.stage), static_cast<int>(context.code),
               context.message, context.plugin_id, context.session_id,
               context.plan_version);
}

/// @brief NaN = "port not provided" sentinel (BT InputPort double default).
inline constexpr double k_behavior_param_unspecified =
    std::numeric_limits<double>::quiet_NaN();

/// @brief Read a double port; write into the optional only when not NaN (NaN keeps it unset).
inline void applyBehaviorPort(const BT::TreeNode& node, const char* port,
                              std::optional<double>& out) {
  double value = k_behavior_param_unspecified;
  if (node.getInput(port, value) && !std::isnan(value)) {
    out = value;
  }
}

/**
 * @brief ComputePathToPose async action: TaskHandle straight to the
 * mainline.
 *
 * onStart: read the goal port, assemble the GetPathGoal (fill the server's
 * current session/plan_version +
 * requestPath holds the TaskHandle, register onComplete ->
 * emitWakeUpSignal. onRunning: wait for the task to be ready;
 * sessionIsCurrent + planVersionIsCurrent double-check prevents stale
 * races (key discipline); after passing, write the path port (XML binding
 * {raw_path}) and incrementPlanVersion. onHalted: cancel the in-flight
 * task. Before any FAILURE, write the precise FailureContext to the
 * blackboard (so Recovery can obtain the cause).
 *
 * Ports: goal={goal} (input), path={raw_path} (output),
 * path_id={path_id} (output, monotonically increasing on every successful
 * plan; the smoother branch uses it to decide re-smoothing).
 */
class ComputePathToPoseNode : public BT::StatefulActionNode {
 public:
  ComputePathToPoseNode(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config),
       nav_server(lookupNavigationRuntime(config)) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<geometry_msgs::PoseStamped>(k_goal, "navigation goal"),
      BT::OutputPort<nav_msgs::Path>("path", "planned path"),
      BT::OutputPort<uint64_t>(k_path_id,
                               "monotonic id bumped on every fresh plan"),
    };
  }

  BT::NodeStatus onStart() override {
    task = {};
    if (!nav_server) {
      FailureContext ctx;
      ctx.stage = FailureStage::Planner;
       ctx.code = ErrorCode::INTERNAL_ERROR;
      ctx.message = "navigation server not injected";
      writeFailureContext(*this, ctx);
      return BT::NodeStatus::FAILURE;
    }
    geometry_msgs::PoseStamped goal;
    if (!getInput(k_goal, goal)) {
      FailureContext ctx;
      ctx.stage = FailureStage::Planner;
       ctx.code = ErrorCode::INVALID_INPUT;
      ctx.message = "goal port missing";
      ctx.session_id = nav_server->currentSessionId();
      ctx.plan_version = nav_server->currentPlanVersion();
      writeFailureContext(*this, ctx);
      return BT::NodeStatus::FAILURE;
    }
    submitted_goal = goal;
    actions::GetPathGoal g;
    g.goal = goal;
    g.session_id = nav_server->currentSessionId();
    g.plan_version = nav_server->currentPlanVersion();
    task = nav_server->requestPath(g);
    if (!task.valid()) {
      FailureContext ctx;
      ctx.stage = FailureStage::Planner;
       ctx.code = ErrorCode::INTERNAL_ERROR;
      ctx.message = "requestPath returned invalid task";
      ctx.plugin_id = "planner";
      ctx.session_id = nav_server->currentSessionId();
      ctx.plan_version = nav_server->currentPlanVersion();
      writeFailureContext(*this, ctx);
      return BT::NodeStatus::FAILURE;
    }
    task.onComplete([this] { emitWakeUpSignal(); });
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override {
    geometry_msgs::PoseStamped current_goal;
    if (getInput(k_goal, current_goal) && !goalEquals(current_goal, submitted_goal)) {
      actions::GetPathGoal update;
      update.goal = current_goal;
      update.session_id = nav_server->currentSessionId();
      update.plan_version = nav_server->currentPlanVersion();
      task = nav_server->requestPath(update);
      if (!task.valid()) return BT::NodeStatus::FAILURE;
      submitted_goal = current_goal;
      task.onComplete([this] { emitWakeUpSignal(); });
      return BT::NodeStatus::RUNNING;
    }
    if (!task.ready()) {
      return BT::NodeStatus::RUNNING;
    }
    const auto& r = task.result();
    if (!nav_server->sessionIsCurrent(r.session_id) ||
        !nav_server->planVersionIsCurrent(r.plan_version)) {
      FailureContext ctx;
      ctx.stage = FailureStage::Planner;
       ctx.code = ErrorCode::INTERNAL_ERROR;
      ctx.message = "stale session/plan_version result rejected";
       ctx.plugin_id = "planner";
      ctx.session_id = r.session_id;
      ctx.plan_version = r.plan_version;
      writeFailureContext(*this, ctx);
      return BT::NodeStatus::FAILURE;
    }
     if (r.error != ErrorCode::NONE) {
      FailureContext ctx;
      ctx.stage = FailureStage::Planner;
      ctx.code = r.error;
      ctx.message = r.message;
       ctx.plugin_id = "planner";
      ctx.session_id = r.session_id;
      ctx.plan_version = r.plan_version;
      writeFailureContext(*this, ctx);
      return BT::NodeStatus::FAILURE;  // decide on failure only, never parse message
    }
    setOutput("path", r.path);
    // path_id increases monotonically (this plan differs from the previous):
    // the downstream smoother branch's RateController uses {path_id} as the
    // reset_token; a new path triggers re-smoothing and bumps plan_version,
    // and the controller keeps up via in-place updatePlan; replans with an
    // unchanged goal trigger this too.
    setOutput(k_path_id, ++planned_path_id);
    return BT::NodeStatus::SUCCESS;
  }

  void onHalted() override { task.cancel(); }

 private:
  components::NavigationRuntime::Ptr nav_server;
  components::TaskHandle<actions::GetPathResult> task;
  geometry_msgs::PoseStamped submitted_goal;
  uint64_t planned_path_id{0};
};

/**
 * @brief SmoothPath one-shot async action (isomorphic).
 *
 * Ports: input_path={raw_path} (input), path={smoothed_path} (output),
 * uses the single smoother configured at configure time.
 */
class SmoothPathNode : public BT::StatefulActionNode {
 public:
  SmoothPathNode(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config),
       nav_server(lookupNavigationRuntime(config)) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<nav_msgs::Path>("input_path", "path to smooth"),
      BT::OutputPort<nav_msgs::Path>("path", "smoothed path"),
    };
  }

  BT::NodeStatus onStart() override {
    task = {};
    if (!nav_server) {
      FailureContext ctx;
      ctx.stage = FailureStage::Smoother;
       ctx.code = ErrorCode::INTERNAL_ERROR;
      ctx.message = "navigation server not injected";
      writeFailureContext(*this, ctx);
      return BT::NodeStatus::FAILURE;
    }
    nav_msgs::Path path;
    if (!getInput("input_path", path)) {
      FailureContext ctx;
      ctx.stage = FailureStage::Smoother;
       ctx.code = ErrorCode::INVALID_INPUT;
      ctx.message = "input_path port missing";
      ctx.session_id = nav_server->currentSessionId();
      ctx.plan_version = nav_server->currentPlanVersion();
      writeFailureContext(*this, ctx);
      return BT::NodeStatus::FAILURE;
    }
    actions::SmoothPathGoal g;
    g.path = path;
    g.session_id = nav_server->currentSessionId();
    g.plan_version = nav_server->currentPlanVersion();
    task = nav_server->requestSmoothing(g);
    if (!task.valid()) {
      FailureContext ctx;
      ctx.stage = FailureStage::Smoother;
       ctx.code = ErrorCode::INTERNAL_ERROR;
      ctx.message = "requestSmoothing returned invalid task";
      ctx.plugin_id = "smoother";
      ctx.session_id = nav_server->currentSessionId();
      ctx.plan_version = nav_server->currentPlanVersion();
      writeFailureContext(*this, ctx);
      return BT::NodeStatus::FAILURE;
    }
    task.onComplete([this] { emitWakeUpSignal(); });
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override {
    if (!task.ready()) {
      return BT::NodeStatus::RUNNING;
    }
    const auto& r = task.result();
    if (!nav_server->sessionIsCurrent(r.session_id) ||
        !nav_server->planVersionIsCurrent(r.plan_version)) {
      FailureContext ctx;
      ctx.stage = FailureStage::Smoother;
       ctx.code = ErrorCode::INTERNAL_ERROR;
      ctx.message = "stale session/plan_version result rejected";
      ctx.plugin_id = "smoother";
      ctx.session_id = r.session_id;
      ctx.plan_version = r.plan_version;
      writeFailureContext(*this, ctx);
      return BT::NodeStatus::FAILURE;
    }
     if (r.error != ErrorCode::NONE) {
      // Make smoothing failure explicit: write the failure_context to the
      // blackboard and return FAILURE for the upper RecoveryNode/recovery
      // branch to handle; no best-effort degradation (algorithm faults are
      // not masked).
      FailureContext ctx;
      ctx.stage = FailureStage::Smoother;
      ctx.code = r.error;
      ctx.message = r.message;
      ctx.plugin_id = "smoother";
      ctx.session_id = r.session_id;
      ctx.plan_version = r.plan_version;
      writeFailureContext(*this, ctx);
      return BT::NodeStatus::FAILURE;
    }
    setOutput("path", r.smoothed_path);
    nav_server->incrementPlanVersion();
    return BT::NodeStatus::SUCCESS;
  }

  void onHalted() override { task.cancel(); }

 private:
  components::NavigationRuntime::Ptr nav_server;
  components::TaskHandle<actions::SmoothPathResult> task;
};

/**
 * @brief FollowPath async action (isomorphic), persists until the
 * controller reaches the goal.
 *
 * When plan_version changes, take the in-place controller update path
 * (requestPlanUpdate): no cancel / no join / no new Execution; the running
 * ControllerAction updates path+version thread-safely, the execution
 * consumes setNewPlan on the next cycle, and the final result carries the
 * latest plan_version. No intermediate stops anywhere (no cancel -> no
 * intermediate zero-velocity publishing).
 *
 * Ports: path={smoothed_path} (input), frequency={controller_frequency}
 * (input), uses the single controller configured at configure time.
 */
class FollowPathNode : public BT::StatefulActionNode {
 public:
  FollowPathNode(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config),
       nav_server(lookupNavigationRuntime(config)) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<nav_msgs::Path>("path", "path to follow"),
      BT::InputPort<double>("frequency", 20.0, "controller frequency in Hz"),
    };
  }

  BT::NodeStatus onStart() override {
    task = {};
    if (!nav_server) {
      FailureContext ctx;
      ctx.stage = FailureStage::Controller;
       ctx.code = ErrorCode::INTERNAL_ERROR;
      ctx.message = "navigation server not injected";
      writeFailureContext(*this, ctx);
      return BT::NodeStatus::FAILURE;
    }
    nav_msgs::Path path;
    if (!getInput("path", path)) {
      FailureContext ctx;
      ctx.stage = FailureStage::Controller;
       ctx.code = ErrorCode::INVALID_INPUT;
      ctx.message = "path port missing";
      ctx.session_id = nav_server->currentSessionId();
      ctx.plan_version = nav_server->currentPlanVersion();
      writeFailureContext(*this, ctx);
      return BT::NodeStatus::FAILURE;
    }
    return startExecution(path) ? BT::NodeStatus::RUNNING
                                : BT::NodeStatus::FAILURE;
  }

  BT::NodeStatus onRunning() override {
    // First check whether the task is already finished: a finished goal no
    // longer attempts in-place updates (execution already released); consume
    // the result directly -- avoids updatePlan falsely reporting failure for
    // an already-finished goal.
    if (task.ready()) {
      return evaluateCompletedResult();
    }
    // Task still running and plan_version changed: update the executing
    // controller in place (no cancel / no join / no new Execution).
    if (nav_server->currentPlanVersion() != submitted_plan_version) {
      nav_msgs::Path path;
      if (!getInput("path", path)) {
        FailureContext ctx;
        ctx.stage = FailureStage::Controller;
         ctx.code = ErrorCode::INVALID_INPUT;
        ctx.message = "path port missing";
        ctx.plugin_id = "controller";
        ctx.session_id = nav_server->currentSessionId();
        ctx.plan_version = nav_server->currentPlanVersion();
        writeFailureContext(*this, ctx);
        return BT::NodeStatus::FAILURE;
      }
      if (!updateExecutionPlan(path)) {
        // Update miss: the task may have finished/terminated exactly during
        // submission (execution released or terminal, requestPlanUpdate
        // rejected the update). Re-check the finished task and consume its
        // result -- a successful-arrival result goes straight to SUCCESS; an
        // "update miss" is not misreported as failure.
        if (task.ready()) {
          return evaluateCompletedResult();
        }
        return BT::NodeStatus::RUNNING;  // not finished yet; retry update/consume next tick
      }
      return BT::NodeStatus::RUNNING;
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override { task.cancel(); }

 private:
  /// @brief Result evaluation for a finished task (shared by the top of
  /// onRunning and the update-miss re-check).
  ///
  /// Race discipline: a SUCCESS "goal reached" result is not rejected due to
  /// a pending-version difference -- the controller may finish with the old
  /// applied version (arrival happened while the update was in flight and
  /// unconsumed), a legitimate completion race; stale-version rejection
  /// applies only to failure/goal-not-reached results (genuinely stale
  /// artifacts).
  BT::NodeStatus evaluateCompletedResult() {
    const auto& r = task.result();
    if (!nav_server->sessionIsCurrent(r.session_id)) {
      FailureContext ctx;
      ctx.stage = FailureStage::Controller;
         ctx.code = ErrorCode::INTERNAL_ERROR;
      ctx.message = "stale session result rejected";
         ctx.plugin_id = "controller";
      ctx.session_id = r.session_id;
      ctx.plan_version = r.plan_version;
      writeFailureContext(*this, ctx);
      return BT::NodeStatus::FAILURE;
    }
    // Controller succeeded and reached the goal: that is the end within the session (goal reached); pending version differences are not rejected.
     if (r.error == ErrorCode::NONE && r.reached) {
      return BT::NodeStatus::SUCCESS;
    }
    if (!nav_server->planVersionIsCurrent(r.plan_version)) {
      FailureContext ctx;
      ctx.stage = FailureStage::Controller;
       ctx.code = ErrorCode::INTERNAL_ERROR;
      ctx.message = "stale session/plan_version result rejected";
       ctx.plugin_id = "controller";
      ctx.session_id = r.session_id;
      ctx.plan_version = r.plan_version;
      writeFailureContext(*this, ctx);
      return BT::NodeStatus::FAILURE;
    }
    // Failure or goal-not-reached: decide on failure only, never parse message.
    FailureContext ctx;
    ctx.stage = FailureStage::Controller;
     ctx.code = r.error != ErrorCode::NONE ? r.error
                                              : ErrorCode::GOAL_NOT_REACHED;
    ctx.message = r.message;
       ctx.plugin_id = "controller";
    ctx.session_id = r.session_id;
    ctx.plan_version = r.plan_version;
    writeFailureContext(*this, ctx);
    return BT::NodeStatus::FAILURE;
  }

  bool startExecution(const nav_msgs::Path& path) {
    actions::ExePathGoal goal;
    goal.path = path;
    goal.session_id = nav_server->currentSessionId();
    goal.plan_version = nav_server->currentPlanVersion();
    submitted_plan_version = goal.plan_version;
    task = nav_server->requestExecution(goal);
    if (!task.valid()) {
      FailureContext ctx;
      ctx.stage = FailureStage::Controller;
       ctx.code = ErrorCode::INTERNAL_ERROR;
      ctx.message = "requestExecution returned invalid task";
       ctx.plugin_id = "controller";
      ctx.session_id = goal.session_id;
      ctx.plan_version = goal.plan_version;
      writeFailureContext(*this, ctx);
      return false;
    }
    task.onComplete([this] { emitWakeUpSignal(); });
    return true;
  }

  /// @brief plan_version changed: update path+version in place for the
  /// still-running goal (no cancel/new Execution). Returns false on update
  /// failure (the node returns FAILURE accordingly).
  bool updateExecutionPlan(const nav_msgs::Path& path) {
    actions::ExePathGoal goal;
    goal.path = path;
    goal.session_id = nav_server->currentSessionId();
    goal.plan_version = nav_server->currentPlanVersion();
    if (!nav_server->requestPlanUpdate(task.goal_id(), goal)) {
      return false;
    }
    submitted_plan_version = goal.plan_version;
    return true;
  }

  components::NavigationRuntime::Ptr nav_server;
  components::TaskHandle<actions::ExePathResult> task;
  uint64_t submitted_plan_version{0};
};

/**
 * @brief Recovery one-shot async action (isomorphic).
 *
 * Ports: recovery={recovery_id}/{clear_costmap_id}/{spin_id}/{wait_id}/
 * {backup_id}
 * (input, XML `<Recovery recovery="{...}"/>`; blackboard keys are decided
 * by the pilot configuration).
 * requestRecovery's Goal.cause = the current blackboard failure_context
 * (structured trigger reason), so the recovery plugin can diagnose who
 * triggered the recovery.
 */
class RecoveryActionNode : public BT::StatefulActionNode {
 public:
  RecoveryActionNode(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config),
      nav_server(lookupNavigationRuntime(config)) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<std::string>("recovery", "recovery plugin id"),
      // Per-run override params (spec: common RecoveryGoal fields). NaN
      // default = not provided, keep the plugin's configure-time defaults.
      BT::InputPort<double>("target_yaw",
                            std::numeric_limits<double>::quiet_NaN(),
                            "override spin target angle [rad]"),
      BT::InputPort<double>("backup_distance",
                            std::numeric_limits<double>::quiet_NaN(),
                            "override back_up distance [m]"),
      BT::InputPort<double>("backup_speed",
                            std::numeric_limits<double>::quiet_NaN(),
                            "override back_up linear speed [m/s]"),
      BT::InputPort<double>("wait_duration",
                            std::numeric_limits<double>::quiet_NaN(),
                            "override wait duration [s]"),
    };
  }

  BT::NodeStatus onStart() override {
    task = {};
    if (!nav_server) {
      FailureContext ctx;
      ctx.stage = FailureStage::Recovery;
       ctx.code = ErrorCode::INTERNAL_ERROR;
      ctx.message = "navigation server not injected";
      writeFailureContext(*this, ctx);
      return BT::NodeStatus::FAILURE;
    }
    std::string recovery_id;
    if (!getInput("recovery", recovery_id)) {
      FailureContext ctx;
      ctx.stage = FailureStage::Recovery;
       ctx.code = ErrorCode::INVALID_INPUT;
      ctx.message = "recovery port missing";
      ctx.session_id = nav_server->currentSessionId();
      ctx.plan_version = nav_server->currentPlanVersion();
      writeFailureContext(*this, ctx);
      return BT::NodeStatus::FAILURE;
    }
    this->recovery_id = recovery_id;
    actions::RecoveryGoal g;
    g.recovery_id = recovery_id;
    readBehaviorParams(g);
    // cause: carry the failure context currently on the blackboard (trigger chain: written by the failing node).
    if (config().blackboard) {
      (void)config().blackboard->get<FailureContext>(k_failure_context, g.cause);
    }
    g.session_id = nav_server->currentSessionId();
    g.plan_version = nav_server->currentPlanVersion();
    task = nav_server->requestRecovery(g);
    if (!task.valid()) {
      FailureContext ctx;
      ctx.stage = FailureStage::Recovery;
       ctx.code = ErrorCode::INTERNAL_ERROR;
      ctx.message = "requestRecovery returned invalid task";
      ctx.plugin_id = recovery_id;
      ctx.session_id = nav_server->currentSessionId();
      ctx.plan_version = nav_server->currentPlanVersion();
      writeFailureContext(*this, ctx);
      return BT::NodeStatus::FAILURE;
    }
    task.onComplete([this] { emitWakeUpSignal(); });
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override {
    if (!task.ready()) {
      return BT::NodeStatus::RUNNING;
    }
    const auto& r = task.result();
    if (!nav_server->sessionIsCurrent(r.session_id) ||
        !nav_server->planVersionIsCurrent(r.plan_version)) {
      FailureContext ctx;
      ctx.stage = FailureStage::Recovery;
       ctx.code = ErrorCode::INTERNAL_ERROR;
      ctx.message = "stale session/plan_version result rejected";
      ctx.plugin_id = recovery_id;
      ctx.session_id = r.session_id;
      ctx.plan_version = r.plan_version;
      writeFailureContext(*this, ctx);
      return BT::NodeStatus::FAILURE;
    }
     if (r.error != ErrorCode::NONE) {
      FailureContext ctx;
      ctx.stage = FailureStage::Recovery;
      ctx.code = r.error;
      ctx.message = r.message;
      ctx.plugin_id = recovery_id;
      ctx.session_id = r.session_id;
      ctx.plan_version = r.plan_version;
      writeFailureContext(*this, ctx);
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
  }

  void onHalted() override { task.cancel(); }

 private:
  /// @brief Read per-run override ports (NaN = not provided) into goal.behavior.
  void readBehaviorParams(actions::RecoveryGoal& g) const {
    applyBehaviorPort(*this, "target_yaw", g.behavior.target_yaw);
    applyBehaviorPort(*this, "backup_distance", g.behavior.backup_distance);
    applyBehaviorPort(*this, "backup_speed", g.behavior.backup_speed);
    applyBehaviorPort(*this, "wait_duration", g.behavior.wait_duration);
  }

  components::NavigationRuntime::Ptr nav_server;
  components::TaskHandle<actions::RecoveryResult> task;
  std::string recovery_id;
};

} // namespace behavior_tree
} // namespace ayanami
