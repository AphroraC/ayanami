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
#include <cstring>
#include <limits>
#include <optional>
#include <string>

namespace ayanami {
namespace behavior_tree {

/// @brief Write the structured failure context to the node's output port
/// (the {failure_context} blackboard binding). Called before the node
/// returns FAILURE so that Recovery nodes can read the precise cause.
inline void writeFailureContext(BT::TreeNode& node,
                                const FailureContext& context) {
  node.setOutput(k_failure_context, context);
  // Failure observability: log one line here (failures are low-frequency
  // events, warn level does not flood; message is diagnostic only,
  // decisions still look only at code).
  spdlog::warn("[bt] stage={} failure: code={} msg='{}' module='{}' "
               "session={} pv={}",
               static_cast<int>(context.stage), static_cast<int>(context.code),
               context.message, context.module_id, context.session_id,
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
 * @brief Common async skeleton of the four navigation action nodes
 * (CRTP template-method pattern; collapses the former per-node boilerplate).
 *
 * Lifecycle owned by the base:
 * - onStart (final): resolve NavigationRuntime -> assembleGoal (ports) ->
 *   submit -> wake-up registration.
 * - onRunning (final): pollInFlight hook (goal-change resubmit / in-place
 *   plan update) -> stale session rejection -> acceptsResult stale policy ->
 *   error -> FAILURE context -> onSuccess.
 * - onHalted (final): cancel the in-flight task.
 * - fail(): writes the structured FailureContext (stage/label from the
 *   derived node) before returning FAILURE.
 *
 * Derived nodes only provide: stage + module label, ports -> goal assembly,
 * the request* call, and the success side effects (output ports / version
 * bump). Decisions look only at ErrorCode; message is diagnostic.
 */
template <typename Derived, typename GoalT, typename ResultT>
class NavigationActionNode : public BT::StatefulActionNode {
 public:
  using Task = components::TaskHandle<ResultT>;

  NavigationActionNode(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config),
      nav_server(lookupNavigationRuntime(config)) {}

  void onHalted() final { task.cancel(); }

 protected:
  components::NavigationRuntime::Ptr nav_server;
  Task task;

  // ---- Derived contract ----

  /// @brief FailureStage for the FailureContext (Planner/Smoother/...).
  FailureStage stage() const { return static_cast<const Derived*>(this)->stageImpl(); }
  /// @brief Module label for the FailureContext ("planner"/... or the
  /// dynamic recovery id).
  std::string moduleLabel() const { return static_cast<const Derived*>(this)->labelImpl(); }
  /// @brief Read the input ports into the request. false = INVALID_INPUT.
  bool assembleGoal(GoalT& goal) { return static_cast<Derived*>(this)->assembleGoalImpl(goal); }
  /// @brief Dispatch the request (requestPath/requestSmoothing/...). The
  /// server stamps session_id/plan_version; the *_id fields select the
  /// module (empty = family default).
  Task submit(const GoalT& goal) { return static_cast<Derived*>(this)->submitImpl(goal); }
  /// @brief Success side effects (write output ports, bump plan_version...).
  BT::NodeStatus onSuccess(const ResultT& result) { return static_cast<Derived*>(this)->onSuccessImpl(result); }

  // ---- Optional derived hooks ----

  /// @brief Mid-flight handling each tick before the result poll (ComputePath
  /// goal-change in-place update, FollowPath in-place plan update). Return a
  /// status to end the tick, nullopt to continue the standard poll.
  std::optional<BT::NodeStatus> pollInFlightImpl() { return std::nullopt; }
  /// @brief Stale-result policy on top of the session check. Default:
  /// reject results whose plan_version is no longer current. The planner and
  /// smoother nodes opt out (they bump the version themselves); FollowPath
  /// exempts a reached-goal success (legitimate arrival race).
  bool acceptsResultImpl(const ResultT& result) const {
    (void)result;
    return nav_server->planVersionIsCurrent(result.plan_version);
  }
  /// @brief Called after a successful submit (stamped goal available).
  void onSubmitted(const GoalT&) {}

  // ---- Helpers for derived nodes ----

  /// @brief Write the failure context and return FAILURE.
  BT::NodeStatus fail(ErrorCode code, const std::string& message,
                      uint64_t session_id, uint64_t plan_version) {
    FailureContext ctx;
    ctx.stage = stage();
    ctx.code = code;
    ctx.message = message;
    ctx.module_id = moduleLabel();
    ctx.session_id = session_id;
    ctx.plan_version = plan_version;
    writeFailureContext(*this, ctx);
    return BT::NodeStatus::FAILURE;
  }

  /// @brief fail() with session/version filled from the runtime (port-level
  /// failures that never reached a request).
  BT::NodeStatus failWithCurrent(ErrorCode code, const std::string& message) {
    if (nav_server) {
      return fail(code, message, nav_server->currentSessionId(),
                  nav_server->currentPlanVersion());
    }
    return fail(code, message, 0, 0);
  }

  /// @brief (Re)submit a goal: validate the handle, register the wake-up.
  bool resubmit(const GoalT& goal) {
    task = {};
    task = submit(goal);
    if (!task.valid()) {
      return false;
    }
    task.onComplete([this] { emitWakeUpSignal(); });
    static_cast<Derived*>(this)->onSubmitted(goal);
    return true;
  }

 private:
  BT::NodeStatus onStart() final {
    if (!nav_server) {
      return failWithCurrent(ErrorCode::INTERNAL_ERROR,
                             "navigation server not injected");
    }
    GoalT goal;
    if (!assembleGoal(goal)) {
      return failWithCurrent(ErrorCode::INVALID_INPUT, "goal port missing");
    }
    if (!resubmit(goal)) {
      return failWithCurrent(ErrorCode::INTERNAL_ERROR,
                             "request returned invalid task");
    }
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() final {
    if (auto handled = static_cast<Derived*>(this)->pollInFlightImpl()) {
      return *handled;
    }
    if (!task.ready()) {
      return BT::NodeStatus::RUNNING;
    }
    const auto& result = task.result();
    // Race discipline: results of a session that is no longer current are
    // always rejected (INTERNAL_ERROR class, not counted into retry stats).
    if (!nav_server->sessionIsCurrent(result.session_id)) {
      return fail(ErrorCode::INTERNAL_ERROR, "stale session result rejected",
                  result.session_id, result.plan_version);
    }
    if (!static_cast<const Derived*>(this)->acceptsResultImpl(result)) {
      return fail(ErrorCode::INTERNAL_ERROR,
                  "stale session/plan_version result rejected",
                  result.session_id, result.plan_version);
    }
    if (result.error != ErrorCode::NONE) {
      // Decide on failure only, never parse message.
      return fail(result.error, result.message, result.session_id,
                  result.plan_version);
    }
    return onSuccess(result);
  }
};

/**
 * @brief ComputePathToPose async action (TaskHandle straight to the
 * mainline).
 *
 * Ports: goal={goal} (input), planner_id (input, empty = family default),
 * path={raw_path} (output). On success the node advances the live plan_version
 * (planning +1, so the controller picks the new path up via in-place
 * updatePlan) AND the raw_path_revision blackboard key, which is what the
 * smoother branch keys its re-smoothing off. The revision is written straight
 * to the blackboard instead of through an output port: a port only exists when
 * the tree binds it, so routing the revision through one would make "the
 * smoother never re-runs" the silent failure mode of a tree that forgot the
 * binding. The planner_id port + goal-level selection mirror nav2 (a
 * PlannerSelector node writes the blackboard key the port binds to).
 */
class ComputePathToPoseNode
    : public NavigationActionNode<ComputePathToPoseNode,
                                  actions::GetPathGoal,
                                  actions::GetPathResult> {
 public:
  using Base = NavigationActionNode<ComputePathToPoseNode,
                                    actions::GetPathGoal,
                                    actions::GetPathResult>;
  using Base::Base;

  static BT::PortsList providedPorts() {
    auto ports = failureContextPorts();
    BT::PortsList own = {
      BT::InputPort<geometry_msgs::PoseStamped>(k_goal, "navigation goal"),
      BT::InputPort<std::string>("planner_id", "", "planner module id"),
      BT::OutputPort<nav_msgs::Path>("path", "planned path"),
    };
    ports.merge(own);
    return ports;
  }

  FailureStage stageImpl() const { return FailureStage::Planner; }
  std::string labelImpl() const { return "planner"; }

  bool assembleGoalImpl(actions::GetPathGoal& goal) {
    if (!getInput(k_goal, goal.goal)) {
      return false;
    }
    (void)getInput("planner_id", goal.planner_id);
    return true;
  }

  Task submitImpl(const actions::GetPathGoal& goal) {
    return nav_server->requestPath(goal);
  }

  void onSubmitted(const actions::GetPathGoal& goal) {
    // Baseline for the pollInFlight goal-change detection.
    submitted_goal = goal.goal;
  }

  /// @brief plan_version is bumped by THIS node on success: a version check
  /// against the request-time stamp would race with concurrent smoother
  /// bumps; round freshness is already enforced by the planner RateController
  /// reset_token (halt + resubmit on goal revision change).
  bool acceptsResultImpl(const actions::GetPathResult&) const { return true; }

  std::optional<BT::NodeStatus> pollInFlightImpl() {
    geometry_msgs::PoseStamped current_goal;
    if (getInput(k_goal, current_goal) &&
        !goalEquals(current_goal, submitted_goal)) {
      // Goal change mid-flight: take the in-place planner update path (the
      // requestPlanUpdate analogue). No cancel / no join / no new task --
      // the running execution adopts the latest goal via its revision
      // guard and replans on its own thread; the same TaskHandle keeps
      // running. An update miss means the task finished exactly during the
      // submit; re-check the finished task next tick (an update miss is
      // not misreported as failure).
      actions::GetPathGoal update;
      update.goal = current_goal;
      if (nav_server->requestPathUpdate(task.goal_id(), update)) {
        submitted_goal = current_goal;
        return BT::NodeStatus::RUNNING;
      }
      return std::nullopt;
    }
    return std::nullopt;
  }

  BT::NodeStatus onSuccessImpl(const actions::GetPathResult& result) {
    setOutput("path", result.path);
    // Live version: drives the running controller's in-place plan update and
    // the stale-result policy.
    nav_server->incrementPlanVersion();
    // Producer-scoped revision: the smoother branch's RateController binds its
    // reset_token here, so a new raw path re-runs smoothing while smoothing's
    // own plan_version bump cannot re-trigger it. Read-modify-write straight
    // on the blackboard keeps the counter monotonic without holding state in
    // the node (the tree is resident across goals; the pilot rewrites this key
    // per round) and without depending on a port binding.
    const auto& blackboard = config().blackboard;
    uint64_t revision = 0;
    (void)blackboard->get<uint64_t>(k_raw_path_revision, revision);
    blackboard->set<uint64_t>(k_raw_path_revision, revision + 1);
    return BT::NodeStatus::SUCCESS;
  }

 private:
  geometry_msgs::PoseStamped submitted_goal;
};

/**
 * @brief SmoothPath one-shot async action.
 *
 * Ports: input_path={raw_path} (input), smoother_id (input, empty =
 * default), path={smoothed_path} (output). On success the node advances the
 * live plan_version (smoothing +1) so the controller adopts the smoothed
 * path. Re-smoothing is driven by the smoother branch's RateController
 * reset_token bound to {raw_path_revision} plus the branch's own tick rate:
 * the BT owns the smoother's re-entry frequency, and the running local map
 * may change between re-runs even when the raw path has not.
 */
class SmoothPathNode : public NavigationActionNode<SmoothPathNode,
                                                  actions::SmoothPathGoal,
                                                  actions::SmoothPathResult> {
 public:
  using Base = NavigationActionNode<SmoothPathNode,
                                    actions::SmoothPathGoal,
                                    actions::SmoothPathResult>;
  using Base::Base;

  static BT::PortsList providedPorts() {
    auto ports = failureContextPorts();
    BT::PortsList own = {
      BT::InputPort<nav_msgs::Path>("input_path", "path to smooth"),
      BT::InputPort<std::string>("smoother_id", "", "smoother module id"),
      BT::OutputPort<nav_msgs::Path>("path", "smoothed path"),
    };
    ports.merge(own);
    return ports;
  }

  FailureStage stageImpl() const { return FailureStage::Smoother; }
  std::string labelImpl() const { return "smoother"; }

  bool assembleGoalImpl(actions::SmoothPathGoal& goal) {
    if (!getInput("input_path", goal.path)) {
      return false;
    }
    (void)getInput("smoother_id", goal.smoother_id);
    return true;
  }

  Task submitImpl(const actions::SmoothPathGoal& goal) {
    return nav_server->requestSmoothing(goal);
  }

  /// @brief Same reasoning as ComputePathToPose: this node bumps the version
  /// itself, so only the session freshness is enforced here.
  bool acceptsResultImpl(const actions::SmoothPathResult&) const { return true; }

  BT::NodeStatus onSuccessImpl(const actions::SmoothPathResult& result) {
    setOutput("path", result.smoothed_path);
    // Unconditional adoption: whenever the smoother publishes a result the
    // running controller switches to the new path via the plan_version bump
    // and FollowPath's in-place updatePlan (no content-signature skip).
    nav_server->incrementPlanVersion();
    return BT::NodeStatus::SUCCESS;
  }
};

/**
 * @brief FollowPath async action, persists until the controller reaches the
 * goal.
 *
 * Ports: path={smoothed_path} (input), controller_id (input, empty =
 * default), frequency={controller_frequency} (input, INFORMATIONAL ONLY).
 *
 * The "frequency" port is a blackboard mirror of
 * navigation.controller_frequency written by the pilot; it is deliberately
 * NOT a control path. The computeVelocityCommand cadence is owned by the
 * execution layer, which receives the rate from the server config at
 * assembly time (NavigationServerBase::configureControllerExecution ->
 * setControllerFrequency). The port exists so the tree documents its
 * expected control rate and for tree-level observability; changing it in
 * the XML does not change the execution rate.
 *
 * When plan_version changes, take the in-place controller update path
 * (requestPlanUpdate): no cancel / no join / no new Execution; the running
 * ControllerAction updates path+version thread-safely, the execution
 * consumes setNewPlan on the next cycle, and the final result carries the
 * latest plan_version. No intermediate stops anywhere (no cancel -> no
 * intermediate zero-velocity publishing).
 */
class FollowPathNode : public NavigationActionNode<FollowPathNode,
                                                  actions::ExePathGoal,
                                                  actions::ExePathResult> {
 public:
  using Base = NavigationActionNode<FollowPathNode,
                                    actions::ExePathGoal,
                                    actions::ExePathResult>;
  using Base::Base;

  static BT::PortsList providedPorts() {
    auto ports = failureContextPorts();
    BT::PortsList own = {
      BT::InputPort<nav_msgs::Path>("path", "path to follow"),
      BT::InputPort<std::string>("controller_id", "", "controller module id"),
      BT::InputPort<double>(
          "frequency", 20.0,
          "INFORMATIONAL: blackboard mirror of navigation.controller_"
          "frequency; the execution rate is set by the server config, not "
          "by this port"),
    };
    ports.merge(own);
    return ports;
  }

  FailureStage stageImpl() const { return FailureStage::Controller; }
  std::string labelImpl() const { return "controller"; }

  bool assembleGoalImpl(actions::ExePathGoal& goal) {
    if (!getInput("path", goal.path)) {
      return false;
    }
    (void)getInput("controller_id", goal.controller_id);
    return true;
  }

  Task submitImpl(const actions::ExePathGoal& goal) {
    return nav_server->requestExecution(goal);
  }

  void onSubmitted(const actions::ExePathGoal& goal) {
    // The server stamped the request-time plan_version into the goal.
    submitted_plan_version = goal.plan_version;
  }

  /// @brief Race discipline: a SUCCESS "goal reached" result is not rejected
  /// due to a pending-version difference -- the controller may finish with
  /// the old applied version (arrival happened while the update was in
  /// flight and unconsumed), a legitimate completion race; stale-version
  /// rejection applies only to failure/goal-not-reached results.
  bool acceptsResultImpl(const actions::ExePathResult& result) const {
    if (result.error == ErrorCode::NONE && result.reached) {
      return true;
    }
    return nav_server->planVersionIsCurrent(result.plan_version);
  }

  std::optional<BT::NodeStatus> pollInFlightImpl() {
    // Finished tasks are consumed by the base poll (an update must not
    // falsely report failure for an already-finished goal).
    if (task.ready()) {
      return std::nullopt;
    }
    // Task still running and plan_version changed: update the executing
    // controller in place (no cancel / no join / no new Execution).
    if (nav_server->currentPlanVersion() != submitted_plan_version) {
      nav_msgs::Path path;
      if (!getInput("path", path)) {
        return failWithCurrent(ErrorCode::INVALID_INPUT, "path port missing");
      }
      actions::ExePathGoal update;
      update.path = path;
      update.plan_version = nav_server->currentPlanVersion();
      if (!nav_server->requestPlanUpdate(task.goal_id(), update)) {
        // Update miss: the task may have finished/terminated exactly during
        // submission; re-check the finished task and consume its result next
        // tick -- an update miss is not misreported as failure.
        return std::nullopt;
      }
      submitted_plan_version = update.plan_version;
    }
    return std::nullopt;
  }

  BT::NodeStatus onSuccessImpl(const actions::ExePathResult&) {
    return BT::NodeStatus::SUCCESS;
  }

 private:
  uint64_t submitted_plan_version{0};
};

/**
 * @brief Recovery one-shot async action.
 *
 * Ports: recovery={recovery_id}/{clear_costmap_id}/{spin_id}/{wait_id}/
 * {backup_id} (input, XML `<Recovery recovery="{...}"/>`), plus per-run
 * override params (NaN = not provided, keep the module configure defaults).
 * The RecoveryGoal.cause = the blackboard failure_context -- consumed THEN
 * CLEARED: the slot is a single shared blackboard key, so leaving it set
 * would let a later recovery read a stale cause.
 */
class RecoveryActionNode : public NavigationActionNode<RecoveryActionNode,
                                                      actions::RecoveryGoal,
                                                      actions::RecoveryResult> {
 public:
  using Base = NavigationActionNode<RecoveryActionNode,
                                    actions::RecoveryGoal,
                                    actions::RecoveryResult>;
  using Base::Base;

  static BT::PortsList providedPorts() {
    // No failureContextPorts() merge here: this node needs the INPUT
    // direction (the cause); an OutputPort under the same name would win
    // the merge and flip the direction.
    BT::PortsList ports = {
      BT::InputPort<std::string>("recovery", "recovery module id"),
      BT::InputPort<FailureContext>(k_failure_context,
                                    "cause consumed by this recovery"),
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
    return ports;
  }

  FailureStage stageImpl() const { return FailureStage::Recovery; }
  std::string labelImpl() const { return recovery_id; }

  bool assembleGoalImpl(actions::RecoveryGoal& goal) {
    if (!getInput("recovery", goal.recovery_id)) {
      return false;
    }
    recovery_id = goal.recovery_id;
    readBehaviorParams(goal);
    // Cause: consume the failure context through the declared input port,
    // then clear the shared entry (the cause is consumed exactly once; a
    // stale context must not leak into a later recovery).
    FailureContext cause;
    if (static_cast<bool>(getInput(k_failure_context, cause))) {
      goal.cause = cause;
    }
    if (config().blackboard) {
      config().blackboard->set<FailureContext>(k_failure_context,
                                               FailureContext{});
    }
    return true;
  }

  Task submitImpl(const actions::RecoveryGoal& goal) {
    return nav_server->requestRecovery(goal);
  }

  BT::NodeStatus onSuccessImpl(const actions::RecoveryResult&) {
    return BT::NodeStatus::SUCCESS;
  }

 private:
  /// @brief Read per-run override ports (NaN = not provided) into goal.behavior.
  void readBehaviorParams(actions::RecoveryGoal& goal) const {
    applyBehaviorPort(*this, "target_yaw", goal.behavior.target_yaw);
    applyBehaviorPort(*this, "backup_distance", goal.behavior.backup_distance);
    applyBehaviorPort(*this, "backup_speed", goal.behavior.backup_speed);
    applyBehaviorPort(*this, "wait_duration", goal.behavior.wait_duration);
  }

  std::string recovery_id;
};

} // namespace behavior_tree
} // namespace ayanami
