// BehaviorTreePilot implementation (resident tree + pending_goal blackboard
// overwrite + preemption owned by the tick thread).
//
// Preemption discipline: beginNavigation only performs latest-wins mailbox
// delivery (overwrite pending_goal + wake), **never halts the BT tree
// cross-thread**. All actual preemption lands on the run thread:
// applyPendingGoal: local haltTree (tick thread) ->
// controllerAction->cancelAll -> publish one zero velocity (Global
// Constraint P9) -> startNewSession (session+1 / plan_version reset to
// zero, sharing the same object with NavigationServerBase) -> rewrite the
// blackboard. halt is only a request flag (halt_requested), consumed
// locally by the run thread's consumeHaltRequest (for stopping the tree on
// stop/destruction; after join there are no concurrent ticks). The tree is
// resident and never rebuilt.

#include "ayanami/behavior_tree/pilot.hpp"

#include "ayanami/behavior_tree/action_nodes.hpp"
#include "ayanami/behavior_tree/navigation.hpp"
#include "ayanami/interfaces/callbacks.hpp"
#include "ayanami/messages/actions.hpp"
#include "ayanami/utility/config.hpp"
#include "ayanami/utility/extension_module.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <stdexcept>
#include <utility>

namespace ayanami {
namespace behavior_tree {

// Generic tree default extension .so: fallback name when
// navigation.behavior_tree_extension is absent (production configs always
// give it explicitly).
namespace {
constexpr const char* k_default_tree_extension_so =
    "libayanami_navigation_behavior_tree.so";
}  // namespace

BehaviorTreePilot::BehaviorTreePilot()
    : session_state(
          std::make_shared<components::NavigationSessionState>()) {}

BehaviorTreePilot::~BehaviorTreePilot() { stop(); }

void BehaviorTreePilot::configure(const std::string& config_path) {
  GlobalConfig::instance(config_path, /*override_path=*/true);
  Config cfg(GlobalConfig::getConfigPath("config_navigation"));

  const std::string k_sec = "navigation";
  planner_frequency = cfg.param<double>(k_sec, "planner_frequency", 10.0);
  smoother_frequency = cfg.param<double>(k_sec, "smoother_frequency", -1.0);
  controller_frequency =
      cfg.param<double>(k_sec, "controller_frequency", 20.0);
  tick_frequency = cfg.param<double>(k_sec, "tick_frequency", 100.0);

// Generic tree extensions: the config names extension .so(s), cached at
  // configure time and loaded during buildTree via
  // ExtensionModule::load_module. The navigation section's value may be a
  // single string or a string array.
  tree_extension_so_names.clear();
  try {
    tree_extension_so_names = cfg.param<std::vector<std::string>>(
        k_sec, "behavior_tree_extension", {});
  } catch (const std::exception&) {
    // behavior_tree_extension is a single string.
  }
  if (tree_extension_so_names.empty()) {
    const std::string single =
        cfg.param<std::string>(k_sec, "behavior_tree_extension", "");
    if (!single.empty()) {
      tree_extension_so_names.push_back(single);
    }
  }
  if (tree_extension_so_names.empty()) {
    tree_extension_so_names.push_back(k_default_tree_extension_so);
  }

  // Module id single source = each module json's name field (no separate
  // *_id config keys).
  // Recovery role mapping lives in the BT extension (see
  // BehaviorTreeExtension::provideBlackboardEntries): the core pilot stays
  // agnostic of recovery roles -- BT only orchestrates.
}

void BehaviorTreePilot::setNavigationRuntime(
    components::NavigationRuntime::Ptr runtime) {
  navigation_runtime = std::move(runtime);
  if (navigation_runtime) {
    // Shared session/version single source of truth: pilot and server use
    // the same NavigationSessionState, so the blackboard session and the
    // actual goal session no longer diverge.
    navigation_runtime->setSessionState(session_state);
  }
  if (bt_blackboard) {
    bt_blackboard->set<components::NavigationRuntime::Ptr>(
        k_navigation_runtime, navigation_runtime);
  }
  if (load_requested && !tree_built) {
    buildTree(xml_path, tree_id);
  }
}

void BehaviorTreePilot::loadFromFile(const std::string& xml_path,
                                     const std::string& tree_id) {
  if (tree_built || load_requested) {
    throw std::runtime_error(
        "BehaviorTreePilot: loadFromFile must be called only once");
  }
  this->xml_path = xml_path;
  this->tree_id = tree_id;

  bt_blackboard = BT::Blackboard::create();
  writeRequiredBlackboardEntries();
  load_requested = true;

  if (navigation_runtime) {
    buildTree(xml_path, tree_id);
  }
}

void BehaviorTreePilot::beginNavigation(
    const geometry_msgs::PoseStamped& goal) {
  if (!tree_built) {
    throw std::runtime_error(
        "BehaviorTreePilot: beginNavigation before loadFromFile/"
        "setNavigationRuntime");
  }
  {
    std::lock_guard<std::mutex> lk(goal_mutex);
    // latest wins: overwrite pending_goal (a new goal replaces the old one, no queuing).
    pending_goal = goal;
  }
  bt_tree.emitWakeUpSignal();  // wake the run loop (it may be in bt_tree.sleep)
}

void BehaviorTreePilot::run() {
  if (!tree_built) {
    spdlog::error("BehaviorTreePilot::run: bt_tree not loaded");
    return;
  }
  running.store(true);
  const double hz = tick_frequency > 0.0 ? tick_frequency : 100.0;
  const auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(1.0 / hz));
  const auto safe_period = std::max(period, std::chrono::milliseconds(1));
 // Navigation-finished idle state (nav2 navigator semantics): the tree
 // returning SUCCESS/FAILURE means this navigation attempt is over -- no
 // more idly re-running the finished tree (measured: after arrival it
 // idled at ~100Hz replanning+smoothing, inflating plan_version at 60/s);
 // wait for a new goal (beginNavigation mailbox) to wake the next tree
 // life.
  bool navigation_finished = false;
  while (true) {
    // Preemption belongs to the tick thread: halt requests / pending goals
    // are consumed locally (never cross-thread haltTree).
    consumeHaltRequest();
    if (kill_switch.load()) break;
    if (applyPendingGoal()) {
      navigation_finished = false;  // new goal / goal revision: resume ticking
    }
    if (navigation_finished && !hasPendingGoal()) {
      bt_tree.sleep(safe_period);
      continue;
    }
    const BT::NodeStatus tick_status = bt_tree.tickOnce();
    // Tree-life terminal reset (v4 halt semantics adaptation): BT.CPP v4's
    // halt only propagates to RUNNING children; subtrees cached in a
    // terminal state (RateController single-shot cache / RecoveryNode retry
    // count / RecoverySequence cursor) are not reset -- without the reset
    // the planner never replans and the recovery storm persists across tree
    // lives (root cause of the rviz recovery storm bug, see
    // production_tree_recovery_test). After SUCCESS/FAILURE this is
    // equivalent to nav2 v3 haltAllActions: enter idle awaiting a new
    // goal.
    if (tick_status != BT::NodeStatus::RUNNING) {
      emitNavigationResult(tick_status == BT::NodeStatus::SUCCESS
                               ? actions::GoalStatus::SUCCEEDED
                               : actions::GoalStatus::ABORTED);
      resetNavigationTreeState(bt_tree);
      navigation_finished = true;
    }
    // Wake race fix: tickOnce's inner layer may consume beginNavigation's
    // wake signal against a stale blackboard, making the subsequent sleep
    // last a full period. If a new goal was delivered during the tick,
    // consume it immediately and add one extra tick, guaranteeing a new goal
    // takes effect within at most one tick period.
    if (applyPendingGoal()) {
      navigation_finished = false;
      bt_tree.tickOnce();
    }
    bt_tree.sleep(safe_period);
  }
  running.store(false);
}

void BehaviorTreePilot::halt() {
  if (!tree_built) {
    return;
  }
  // Request flag: the actual haltTree runs in the run thread's consumeHaltRequest (never cross-thread).
  halt_requested.store(true);
  bt_tree.emitWakeUpSignal();  // if run is sleeping, wake it so it enters the next tick soon
}

void BehaviorTreePilot::consumeHaltRequest() {
  if (!halt_requested.exchange(false)) {
    return;
  }
  // This thread (the tick thread) owns bt_tree access, so halt is safe.
  bt_tree.haltTree();
  // Session-level terminal outcome for an in-flight navigation: CANCELED
  // (never emitted twice for the same tree life; no goal -> no emission).
  if (navigation_started && !navigation_result_emitted) {
    emitNavigationResult(actions::GoalStatus::CANCELED);
  }
  if (navigation_runtime) {
    // Preemption discipline: preempt the entire controller action map (with
    // multiple controllers configured, not just the default id), preventing
    // residual velocity output from non-default controllers. Non-blocking:
    // cancel() already published the zeroed command synchronously; the task
    // winds down on its own thread so the tick thread never blocks on a
    // slow plugin call.
    navigation_runtime->preemptAllControllers();
  }
}

void BehaviorTreePilot::emitNavigationResult(
    actions::GoalStatus status) {
  navigation_result_emitted = true;
  actions::NavigationResult result;
  result.status = status;
  result.context.stage = FailureStage::Navigation;
  result.context.session_id = session_state->currentSessionId();
  result.context.plan_version = session_state->currentPlanVersion();
  if (status == actions::GoalStatus::SUCCEEDED) {
    result.context.code = ErrorCode::NONE;
  } else if (status == actions::GoalStatus::CANCELED) {
    result.context.code = ErrorCode::CANCELED;
    result.context.message = "navigation canceled";
  } else {
    // Tree FAILURE: take the structured failure context the failing node
    // wrote; fall back to a Navigation-stage entry when absent.
    FailureContext ctx;
    if (bt_blackboard) {
      (void)bt_blackboard->get<FailureContext>(k_failure_context, ctx);
    }
    result.context = ctx;
    result.context.session_id = session_state->currentSessionId();
    result.context.plan_version = session_state->currentPlanVersion();
    if (result.context.code == ErrorCode::NONE) {
      result.context.stage = FailureStage::Navigation;
      result.context.code = ErrorCode::INTERNAL_ERROR;
      result.context.message = "navigation failed without failure context";
    }
  }
  interfaces::NavigationCallbacks::on_navigation_result(result);
}

void BehaviorTreePilot::stop() {
  halt_requested.store(true);
  kill_switch.store(true);
  if (tree_built) {
    bt_tree.emitWakeUpSignal();  // wake from sleep so run exits soon
  }
  if (tick_thread.joinable()) {
    tick_thread.join();
  }
}

bool BehaviorTreePilot::hasPendingGoal() {
  std::lock_guard<std::mutex> lk(goal_mutex);
  return pending_goal.has_value();
}

bool BehaviorTreePilot::applyPendingGoal() {
  std::optional<geometry_msgs::PoseStamped> goal;
  {
    std::lock_guard<std::mutex> lk(goal_mutex);
    if (!pending_goal) {
      return false;
    }
    goal = std::move(pending_goal);
    pending_goal.reset();
  }
  // The first goal starts a session. Later goals revise the active task: the
  // controller keeps following its current path until the revised path is
  // available, while the planner updates in place.
  navigation_result_emitted = false;  // new tree life: fresh emission
  if (!navigation_started) {
    session_state->startNewSession();
    navigation_started = true;
    goal_revision = 0;
  } else {
    ++goal_revision;
  }

  bt_blackboard->set<geometry_msgs::PoseStamped>(k_goal, *goal);
  // Per navigation round: plan_version reset (a new goal starts from version 0)
  bt_blackboard->set<uint64_t>(k_plan_version, session_state->currentPlanVersion());
  // Blackboard session reflects the shared session
  bt_blackboard->set<uint64_t>(k_session_id, session_state->currentSessionId());
  bt_blackboard->set<uint64_t>(k_goal_revision, goal_revision);
  // failure_context cleared each navigation round (failure context is only valid for this session).
  bt_blackboard->set<FailureContext>(k_failure_context, FailureContext{});  return true;
}

void BehaviorTreePilot::buildTree(const std::string& xml_path,
                                  const std::string& tree_id) {
  if (!bt_blackboard) {
    bt_blackboard = BT::Blackboard::create();
    writeRequiredBlackboardEntries();
  }
  bt_blackboard->set<components::NavigationRuntime::Ptr>(
      k_navigation_runtime, navigation_runtime);

  BT::BehaviorTreeFactory factory;
  std::string fallback_tree_id;
  // Generic tree: load and hold the configured BehaviorTreeExtension(s);
  // the extensions are responsible for registering all custom nodes and the
  // default tree XML; the core no longer embeds any navigation XML/policy.
  for (const auto& so_name : tree_extension_so_names) {
    auto ext_module = ExtensionModule::load_module(so_name);
    if (!ext_module) {
      throw std::runtime_error("BehaviorTreePilot: failed to load BT extension '" +
                               so_name + "'");
    }
    auto ext =
        std::dynamic_pointer_cast<BehaviorTreeExtension>(std::move(ext_module));
    if (!ext) {
      throw std::runtime_error("BehaviorTreePilot: '" + so_name +
                               "' is not a BehaviorTreeExtension");
    }
    ext->registerNodes(factory);
    ext->registerTrees(factory);
    ext->provideBlackboardEntries(bt_blackboard);
    if (fallback_tree_id.empty()) {
      fallback_tree_id = ext->defaultTreeId();
    }
    tree_extensions.push_back(std::move(ext));
  }
  // Resident-tree reset protocol as a BUILD-TIME check: every custom
  // control/decorator node must implement NavigationTreeNodeState, otherwise
  // its terminal-cached state leaks across tree lives (v4 halt only resets
  // RUNNING children). A violating extension fails here, not in the field.
  validateTreeNodeStateProtocol(factory);
  if (xml_path.empty()) {
    // The default tree was registered by the extension.
  } else {
    factory.registerBehaviorTreeFromFile(xml_path);
  }
  const std::string resolved_tree_id =
      tree_id.empty() ? fallback_tree_id : tree_id;
  if (resolved_tree_id.empty()) {
    throw std::runtime_error(
        "BehaviorTreePilot: no bt_tree id configured and extension provides no "
        "default");
  }
  bt_tree = factory.createTree(resolved_tree_id, bt_blackboard);
  validateRequiredBlackboardEntries(bt_blackboard);
  // Extension-declared entries (e.g. recovery role ids) are validated per
  // extension -- the core list stays family/tree agnostic.
  for (const auto& ext : tree_extensions) {
    for (const char* key : ext->requiredBlackboardEntries()) {
      if (!bt_blackboard->getEntry(key)) {
        throw std::runtime_error(std::string("blackboard entry '") + key +
                                 "' required by BT extension is missing");
      }
    }
  }
  tree_built = true;
}

void BehaviorTreePilot::writeRequiredBlackboardEntries() {
  bt_blackboard->set<geometry_msgs::PoseStamped>(k_goal,
                                               geometry_msgs::PoseStamped{});
  bt_blackboard->set<nav_msgs::Path>(k_raw_path, nav_msgs::Path{});
  bt_blackboard->set<nav_msgs::Path>(k_smoothed_path, nav_msgs::Path{});
  bt_blackboard->set<double>(k_planner_frequency, planner_frequency);
  bt_blackboard->set<double>(k_smoother_frequency, smoother_frequency);
  bt_blackboard->set<double>(k_controller_frequency, controller_frequency);
  bt_blackboard->set<uint64_t>(k_session_id, session_state->currentSessionId());
  bt_blackboard->set<uint64_t>(k_plan_version, 0u);
  bt_blackboard->set<uint64_t>(k_goal_revision, 0u);
  bt_blackboard->set<FailureContext>(k_failure_context, FailureContext{});
}

} // namespace behavior_tree
} // namespace ayanami
