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
#include <ayanami/utility/logging.hpp>

#include <chrono>
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
inline void writeFailureContext(BT::TreeNode& node, const FailureContext& context) {
  node.setOutput(k_failure_context, context);
  // Failure observability: log one line here (failures are low-frequency
  // events, warn level does not flood; message is diagnostic only,
  // decisions still look only at code).
  SPDLOG_LOGGER_WARN(
    ayanami::btLogger(),
    "stage={} failure: code={} msg='{}' module='{}' "
    "session={} pv={}",
    static_cast<int>(context.stage),
    static_cast<int>(context.code),
    context.message,
    context.module_id,
    context.session_id,
    context.plan_version);
}

/// @brief Advance the live plan version. The BT tree is the SOLE owner of
/// plan_version advancement (the server and the modules never bump it):
/// every producer node that lands a new path artifact -- a planned raw path,
/// a refined smoothed path, or a degradation to the raw path -- calls this
/// one helper, and the running controller adopts the artifact through
/// FollowPath's in-place updatePlan.
inline void advancePlanVersion(const components::NavigationRuntime::Ptr& nav_server) {
  nav_server->incrementPlanVersion();
}

/// @brief NaN = "port not provided" sentinel (BT InputPort double default).
inline constexpr double k_behavior_param_unspecified = std::numeric_limits<double>::quiet_NaN();

/// @brief Read a double port; write into the optional only when not NaN (NaN keeps it unset).
inline void applyBehaviorPort(const BT::TreeNode& node, const char* port, std::optional<double>& out) {
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
 * - onHalted (final): cancel the in-flight task, drop the handle and run
 *   the derived onHaltedImpl hook (resident-tree discipline: no stale
 *   TaskState survives a halt on the node).
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

  void onHalted() final {
    task.cancel();
    // Drop the handle: no finished/canceled TaskState outlives the halt on
    // the node (the resident tree keeps nodes across attempts, and the next
    // onStart must start from a clean slot anyway).
    task = {};
    static_cast<Derived*>(this)->onHaltedImpl();
  }

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
  /// @brief Called from onHalted when the tree halts this RUNNING node
  /// (BT.CPP StatefulActionNode only invokes onHalted while RUNNING).
  /// Derived cleanup of per-node state goes here.
  void onHaltedImpl() {}

  // ---- Helpers for derived nodes ----

  /// @brief Write the failure context and return FAILURE.
  BT::NodeStatus fail(ErrorCode code, const std::string& message, uint64_t session_id, uint64_t plan_version) {
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
      return fail(code, message, nav_server->currentSessionId(), nav_server->currentPlanVersion());
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
      return failWithCurrent(ErrorCode::INTERNAL_ERROR, "navigation server not injected");
    }
    GoalT goal;
    if (!assembleGoal(goal)) {
      return failWithCurrent(ErrorCode::INVALID_INPUT, "goal port missing");
    }
    if (!resubmit(goal)) {
      return failWithCurrent(ErrorCode::INTERNAL_ERROR, "request returned invalid task");
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
      return fail(ErrorCode::INTERNAL_ERROR, "stale session result rejected", result.session_id, result.plan_version);
    }
    if (!static_cast<const Derived*>(this)->acceptsResultImpl(result)) {
      return fail(
        ErrorCode::INTERNAL_ERROR,
        "stale session/plan_version result rejected",
        result.session_id,
        result.plan_version);
    }
    if (result.error != ErrorCode::NONE) {
      // Decide on failure only, never parse message.
      return fail(result.error, result.message, result.session_id, result.plan_version);
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
: public NavigationActionNode<ComputePathToPoseNode, actions::GetPathGoal, actions::GetPathResult> {
public:
  using Base = NavigationActionNode<ComputePathToPoseNode, actions::GetPathGoal, actions::GetPathResult>;
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

  Task submitImpl(const actions::GetPathGoal& goal) { return nav_server->requestPath(goal); }

  /// @brief plan_version is bumped by THIS node on success: a version check
  /// against the request-time stamp would race with concurrent smoother
  /// bumps; round freshness is already enforced by the planner RateController
  /// reset_token (halt + resubmit on goal revision change).
  bool acceptsResultImpl(const actions::GetPathResult&) const { return true; }

  BT::NodeStatus onSuccessImpl(const actions::GetPathResult& result) {
    setOutput("path", result.path);
    // Live version: drives the running controller's in-place plan update and
    // the stale-result policy (BT-layer responsibility, shared helper).
    advancePlanVersion(nav_server);
    // Producer-scoped revision: the smoother branch keys its
    // re-INITIALIZATION off this counter -- a new raw path restarts
    // smoothing from the raw path instead of refining the previous route's
    // smoothed path (the smoother's iterative input is {smoothed_path}
    // otherwise). Read-modify-write straight on the blackboard keeps the
    // counter monotonic without holding state in the node.
    const auto& blackboard = config().blackboard;
    uint64_t revision = 0;
    (void)blackboard->get<uint64_t>(k_raw_path_revision, revision);
    blackboard->set<uint64_t>(k_raw_path_revision, revision + 1);
    return BT::NodeStatus::SUCCESS;
  }

  void onHaltedImpl() {
    // nav2 ComputePathToPoseAction::halt parity: a halted planner node
    // leaves NO path artifact behind -- the next run must not observe the
    // aborted route's raw path through the output port.
    setOutput("path", nav_msgs::Path{});
  }
};

/**
 * @brief SmoothPath one-shot async action (iterative refinement semantics).
 *
 * The smoother's update object is {smoothed_path} itself: each run takes the
 * CURRENT smoothed path as its input and improves it against the fresh
 * costmap (that is what keeps the smoother reactive to local obstacles at
 * its configured rate). Only when the planner landed a NEW raw path
 * ({raw_path_revision} changed since this node's last run) -- or when no
 * smoothed path exists yet -- does the run (re)initialize from {raw_path};
 * a new planned route must never be refined out of the old route's
 * smoothed path.
 *
 * Ports: input_path={smoothed_path} (self-feedback input), raw_path={raw_path}
 * (initialization / re-initialization source), smoother_id (input, empty =
 * default), path={smoothed_path} (output). On success the node advances the
 * live plan_version through the shared BT-layer helper so the controller
 * adopts the refined path (the execution layer dedups content-identical
 * updates, so a converged path stops reaching controller->setPlan).
 * Re-smoothing cadence and re-initialization are owned by this branch's
 * RateController (hz={smoother_frequency}, reset_token={raw_path_revision}).
 */
class SmoothPathNode : public NavigationActionNode<SmoothPathNode, actions::SmoothPathGoal, actions::SmoothPathResult> {
public:
  using Base = NavigationActionNode<SmoothPathNode, actions::SmoothPathGoal, actions::SmoothPathResult>;
  using Base::Base;

  static BT::PortsList providedPorts() {
    auto ports = failureContextPorts();
    BT::PortsList own = {
      BT::InputPort<nav_msgs::Path>(
        "input_path",
        "path to refine (the current smoothed "
        "path in the production tree)"),
      BT::InputPort<nav_msgs::Path>(
        "raw_path",
        "initialization source: used when no smoothed path "
        "exists yet or a new raw path landed"),
      BT::InputPort<std::string>("smoother_id", "", "smoother module id"),
      BT::OutputPort<nav_msgs::Path>("path", "smoothed path"),
    };
    ports.merge(own);
    return ports;
  }

  FailureStage stageImpl() const { return FailureStage::Smoother; }
  std::string labelImpl() const { return "smoother"; }

  bool assembleGoalImpl(actions::SmoothPathGoal& goal) {
    nav_msgs::Path current;
    (void)getInput("input_path", current);
    uint64_t revision = 0;
    if (config().blackboard) {
      (void)config().blackboard->get<uint64_t>(k_raw_path_revision, revision);
    }
    if (revision != initialized_revision_ || current.poses.empty()) {
      // New raw path (or first run ever): start from the raw path. Trees
      // that bind only input_path (hand-built test trees binding
      // {raw_path} directly) fall back to it.
      if (!getInput("raw_path", goal.path) || goal.path.poses.empty()) {
        goal.path = current;
      }
    } else {
      // Iterative refinement: keep improving the current smoothed path.
      goal.path = current;
    }
    initialized_revision_ = revision;
    (void)getInput("smoother_id", goal.smoother_id);
    return !goal.path.poses.empty();
  }

  Task submitImpl(const actions::SmoothPathGoal& goal) { return nav_server->requestSmoothing(goal); }

  /// @brief Same reasoning as ComputePathToPose: this node bumps the version
  /// itself, so only the session freshness is enforced here.
  bool acceptsResultImpl(const actions::SmoothPathResult&) const { return true; }

  BT::NodeStatus onSuccessImpl(const actions::SmoothPathResult& result) {
    setOutput("path", result.smoothed_path);
    // Unconditional adoption: whenever the smoother lands a result the
    // version advances so the running controller re-evaluates it. The
    // execution layer's content-signature dedup makes a converged path a
    // no-op for controller->setPlan (version bookkeeping still advances).
    advancePlanVersion(nav_server);
    return BT::NodeStatus::SUCCESS;
  }

  void onHaltedImpl() {
    // Defense in depth: a halt mid-flight clears the refinement baseline so
    // the next run re-decides initialization from the blackboard (the
    // pilot's per-attempt blackboard reset is the primary guarantee -- this
    // covers a halt INSIDE one attempt, where the blackboard is not reset).
    initialized_revision_ = 0;
    setOutput("path", nav_msgs::Path{});
  }

private:
  /// @brief The raw_path_revision this node last initialized from (reset at
  /// the IDLE edge / on halt; the pilot's per-attempt blackboard reset is
  /// the primary cross-attempt guarantee).
  uint64_t initialized_revision_{0};
};

/**
 * @brief DegradeToRawPath: the smoother branch's failure fallback.
 *
 * Copies {raw_path} into {smoothed_path} so FollowPath keeps a followable
 * path when smoothing fails (smoothing is an enhancement, never a
 * precondition), logs a throttled SMOOTHER_FAILED warning carrying the
 * failure cause, advances the plan version (the controller must adopt the
 * raw path) and clears the consumed failure context (consume-then-clear
 * discipline, same as the Recovery action).
 */
class DegradeToRawPathNode : public BT::SyncActionNode {
public:
  DegradeToRawPathNode(const std::string& name, const BT::NodeConfig& config) : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<nav_msgs::Path>("input_path", "the raw path to fall back to"),
      BT::InputPort<FailureContext>(k_failure_context, "the smoother failure cause (consumed)"),
      BT::OutputPort<nav_msgs::Path>("path", "receives the raw path"),
    };
  }

  BT::NodeStatus tick() override {
    nav_msgs::Path raw;
    if (!getInput("input_path", raw) || raw.poses.empty()) {
      // Nothing to degrade to: the branch fails for real (the planner never
      // produced a path -- not a smoothing problem).
      return BT::NodeStatus::FAILURE;
    }
    setOutput("path", raw);
    FailureContext cause;
    (void)getInput(k_failure_context, cause);
    const auto now = std::chrono::steady_clock::now();
    if (now - last_warn_at >= std::chrono::seconds(1)) {
      last_warn_at = now;
      SPDLOG_LOGGER_WARN(
        ayanami::btLogger(),
        "SMOOTHER_FAILED: code={} msg='{}'; degrading to raw path "
        "follow ({} poses)",
        static_cast<int>(cause.code),
        cause.message,
        raw.poses.size());
    }
    if (auto runtime = lookupNavigationRuntime(config())) {
      advancePlanVersion(runtime);
    }
    if (config().blackboard) {
      config().blackboard->set<FailureContext>(k_failure_context, FailureContext{});
    }
    return BT::NodeStatus::SUCCESS;
  }

private:
  std::chrono::steady_clock::time_point last_warn_at{};
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
class FollowPathNode : public NavigationActionNode<FollowPathNode, actions::ExePathGoal, actions::ExePathResult> {
public:
  using Base = NavigationActionNode<FollowPathNode, actions::ExePathGoal, actions::ExePathResult>;
  using Base::Base;

  static BT::PortsList providedPorts() {
    auto ports = failureContextPorts();
    BT::PortsList own = {
      BT::InputPort<nav_msgs::Path>("path", "path to follow"),
      BT::InputPort<std::string>("controller_id", "", "controller module id"),
      BT::InputPort<double>(
        "frequency",
        20.0,
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

  Task submitImpl(const actions::ExePathGoal& goal) { return nav_server->requestExecution(goal); }

  void onSubmitted(const actions::ExePathGoal& goal) {
    // The server stamped the request-time plan_version into the goal.
    submitted_plan_version = goal.plan_version;
    // Kept for the framework-preemption resume (see pollInFlightImpl).
    last_submitted_goal = goal;
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
    // Framework-initiated preemption (a recovery dispatch canceled the
    // controller): the CANCELED result is NOT a failure -- resume the same
    // goal in place. Turning it into a failure made every map-refreshing
    // clear a stop -> stale-failure -> retry cascade (2026-09-29 log
    // forensics). A CANCELED result cannot originate from a tree halt
    // (onHalted drops the handle), so observing one here always means the
    // task was canceled out from under this RUNNING node. While the
    // recovery still holds its exclusive slot the resubmit is rejected;
    // the handle stays dropped and a later tick retries.
    if (!task.valid()) {
      auto resumed = submitImpl(last_submitted_goal);
      if (resumed.valid() && !resumed.ready()) {
        task = std::move(resumed);
        submitted_plan_version = nav_server->currentPlanVersion();
      }
      return BT::NodeStatus::RUNNING;
    }
    if (task.ready()) {
      if (task.result().error == ErrorCode::CANCELED) {
        task = {};
        return BT::NodeStatus::RUNNING;
      }
      return std::nullopt;
    }
    // Task still running and plan_version changed: update the executing
    // controller in place (no cancel / no join / no new Execution).
    if (nav_server->currentPlanVersion() != submitted_plan_version) {
      nav_msgs::Path path;
      if (!getInput("path", path)) {
        return failWithCurrent(ErrorCode::INVALID_INPUT, "path port missing");
      }
      // An EMPTY artifact is never a legitimate plan update: between the
      // smoother round's token-halt (which clears {smoothed_path}) and the
      // next successful round, a planner landing can bump the version while
      // the artifact is still empty -- pushing that empty path would kill
      // the running controller with EMPTY_PLAN. Keep the last plan; the
      // window closes when the smoother lands and bumps the version again.
      if (path.poses.empty()) {
        return std::nullopt;
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

  BT::NodeStatus onSuccessImpl(const actions::ExePathResult&) { return BT::NodeStatus::SUCCESS; }

private:
  uint64_t submitted_plan_version{0};
  actions::ExePathGoal last_submitted_goal;
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
class RecoveryActionNode
: public NavigationActionNode<RecoveryActionNode, actions::RecoveryGoal, actions::RecoveryResult> {
public:
  using Base = NavigationActionNode<RecoveryActionNode, actions::RecoveryGoal, actions::RecoveryResult>;
  using Base::Base;

  static BT::PortsList providedPorts() {
    // No failureContextPorts() merge here: this node needs the INPUT
    // direction (the cause); an OutputPort under the same name would win
    // the merge and flip the direction.
    BT::PortsList ports = {
      BT::InputPort<std::string>("recovery", "recovery module id"),
      BT::InputPort<FailureContext>(k_failure_context, "cause consumed by this recovery"),
      BT::InputPort<double>("target_yaw", std::numeric_limits<double>::quiet_NaN(), "override spin target angle [rad]"),
      BT::InputPort<double>(
        "backup_distance",
        std::numeric_limits<double>::quiet_NaN(),
        "override back_up distance [m]"),
      BT::InputPort<double>(
        "backup_speed",
        std::numeric_limits<double>::quiet_NaN(),
        "override back_up linear speed [m/s]"),
      BT::InputPort<double>("wait_duration", std::numeric_limits<double>::quiet_NaN(), "override wait duration [s]"),
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
      config().blackboard->set<FailureContext>(k_failure_context, FailureContext{});
    }
    return true;
  }

  Task submitImpl(const actions::RecoveryGoal& goal) { return nav_server->requestRecovery(goal); }

  BT::NodeStatus onSuccessImpl(const actions::RecoveryResult&) { return BT::NodeStatus::SUCCESS; }

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

}  // namespace behavior_tree
}  // namespace ayanami
