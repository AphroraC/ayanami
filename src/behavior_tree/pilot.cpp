// BehaviorTreePilot implementation (one tree instance per navigation attempt +
// pending_goal blackboard overwrite + preemption owned by the tick thread).
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
// stop/destruction; after join there are no concurrent ticks). The FACTORY
// (nodes + tree definitions) is built once; the tree INSTANCE is per attempt,
// so a tree life is a tree object and no state can cross between attempts.

#include "ayanami/behavior_tree/pilot.hpp"
#include <ayanami/utility/bootstrap.hpp>

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
  ayanami::bootstrap(config_path);
  Config cfg(GlobalConfig::getConfigPath("config_navigation"));

  const std::string k_sec = "navigation";
  planner_frequency = cfg.param<double>(k_sec, "planner_frequency", 10.0);
  smoother_frequency = cfg.param<double>(k_sec, "smoother_frequency", -1.0);
  controller_frequency =
      cfg.param<double>(k_sec, "controller_frequency", 20.0);
  tick_frequency = cfg.param<double>(k_sec, "tick_frequency", 100.0);

// Generic tree extensions: the config names extension .so(s), cached at
  // configure time and loaded during buildFactory via
  // ExtensionModule::loadModule. The navigation section's value may be a single
  // string or a string array (one shared helper handles both spellings).
  tree_extension_so_names = cfg.paramStringOrList(k_sec, "behavior_tree_extension");
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
  if (!factory_built) {
    buildFactory(xml_path, tree_id);
  }
}

void BehaviorTreePilot::loadFromFile(const std::string& xml_path,
                                     const std::string& tree_id) {
  if (factory_built) {
    throw std::runtime_error(
        "BehaviorTreePilot: loadFromFile must be called only once");
  }
  this->xml_path = xml_path;
  this->tree_id = tree_id;
  buildFactory(xml_path, tree_id);
}

void BehaviorTreePilot::beginNavigation(
    const geometry_msgs::PoseStamped& goal) {
  if (!factory_built) {
    throw std::runtime_error(
        "BehaviorTreePilot: beginNavigation before loadFromFile");
  }
  {
    std::lock_guard<std::mutex> lk(goal_mutex);
    // latest wins: overwrite pending_goal (a new goal replaces the old one, no queuing).
    pending_goal = goal;
  }
  // Wake the tick loop. Two waiting places: its mailbox wait (idle, no instance)
  // and the tree's own sleep (attempt in flight). The tree one is guarded --
  // there is no tree object to poke while idle.
  goal_cv.notify_all();
  if (instance_active.load()) {
    bt_tree.emitWakeUpSignal();
  }
}

void BehaviorTreePilot::run() {
  if (!factory_built) {
    spdlog::error("BehaviorTreePilot::run: factory not built");
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
  bool navigation_finished = true;  // idle until a goal arrives
  while (true) {
    // Preemption belongs to the tick thread: halt requests / pending goals
    // are consumed locally (never cross-thread haltTree).
    consumeHaltRequest();
    if (kill_switch.load()) break;
    // A failure while building the attempt's tree (an unparsable XML, an
    // unresolvable module reference, a violated blackboard protocol) is a
    // configuration error, and it happens on THIS thread: letting it escape
    // would call std::terminate and take the process down with no diagnosis.
    // Report it the way every other terminal outcome is reported -- a session
    // navigation result -- and stop ticking, because a pilot that cannot build
    // its tree cannot recover by trying again.
    try {
    if (applyPendingGoal()) {
      navigation_finished = false;  // new goal / goal revision: resume ticking
    }
    if (!instance_active.load()) {
      // Idle between attempts. There is no tree instance, so nothing may touch
      // bt_tree (tick/sleep/emitWakeUpSignal on a default-constructed Tree is a
      // null-root access): wait on the mailbox instead, which a new goal wakes.
      std::unique_lock<std::mutex> lk(goal_mutex);
      goal_cv.wait_for(lk, safe_period, [this] {
        return pending_goal.has_value() || kill_switch.load() ||
               halt_requested.load();
      });
      continue;
    }
    const BT::NodeStatus tick_status = bt_tree.tickOnce();
    if (tick_status != BT::NodeStatus::RUNNING) {
      emitNavigationResult(tick_status == BT::NodeStatus::SUCCESS
                               ? actions::GoalStatus::SUCCEEDED
                               : actions::GoalStatus::ABORTED);
      // The attempt is over: drop the instance. The next goal builds a fresh
      // one, so nothing -- RateController caches, retry budgets, rotation
      // cursors -- can carry over. This is the whole point of not keeping a
      // resident tree, and it replaces the former reset protocol.
      releaseTreeInstance();
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
    // Only sleep on a live instance: the terminal branch above may have just
    // released it, and sleeping a default-constructed Tree is a null-root access.
    // Without an instance the top of the loop waits on the mailbox instead.
    if (instance_active.load()) {
      bt_tree.sleep(safe_period);
    }
    } catch (const std::exception& e) {
      spdlog::critical("BehaviorTreePilot: navigation attempt failed to start: {}",
                       e.what());
      failure_context = e.what();
      emitNavigationResult(actions::GoalStatus::ABORTED);
      releaseTreeInstance();
      // A refused attempt has no record to keep: the blackboard only becomes
      // meaningful once the attempt actually starts. (A FINISHED attempt keeps
      // its blackboard -- see releaseTreeInstance.)
      bt_blackboard.reset();
      navigation_finished = true;
      break;  // a configuration error will not fix itself by retrying
    }
  }
  running.store(false);
}

void BehaviorTreePilot::halt() {
  if (!factory_built) {
    return;
  }
  // Request flag: the actual haltTree runs in the run thread's consumeHaltRequest (never cross-thread).
  halt_requested.store(true);
  // The loop may be waiting on either the mailbox or the tree sleep; wake both
  // (the tree one only when an instance exists).
  goal_cv.notify_all();
  if (instance_active.load()) {
    bt_tree.emitWakeUpSignal();
  }
}

void BehaviorTreePilot::consumeHaltRequest() {
  if (!halt_requested.exchange(false)) {
    return;
  }
  // This thread (the tick thread) owns bt_tree access, so halt is safe. The
  // instance is halted (RUNNING children get onHalted) and then released: a
  // halted attempt is over, and keeping the object would resurrect its state if
  // a goal arrived without an intervening createTreeInstance.
  if (instance_active.load()) {
    bt_tree.haltTree();
  }
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
    // slow module call.
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
    if (!failure_context.empty()) {
      // The attempt never started (tree build / validation refused it), so the
      // blackboard has nothing to say; the recorded reason is the diagnosis.
      ctx = FailureContext{};
      ctx.message = failure_context;
      ctx.code = ErrorCode::INVALID_INPUT;
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
  goal_cv.notify_all();
  if (instance_active.load()) {
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

  // A goal arriving with no live instance means the previous attempt is over
  // (or none has started): this is a NEW attempt, so build a fresh tree and
  // blackboard. A goal arriving while the instance is still active is a REVISION
  // of the running task: the instance stays, the running controller keeps
  // following its path, and only the blackboard entries below change.
  if (!instance_active.load()) {
    createTreeInstance();
  }

  bt_blackboard->set<geometry_msgs::PoseStamped>(k_goal, *goal);
  // Raw-path revision restarts with the round: the smoother branch's
  // reset_token must see a change when this round's first plan lands.
  bt_blackboard->set<uint64_t>(k_raw_path_revision, 0u);
  // Blackboard session reflects the shared session
  bt_blackboard->set<uint64_t>(k_session_id, session_state->currentSessionId());
  bt_blackboard->set<uint64_t>(k_goal_revision, goal_revision);
  // failure_context cleared each navigation round (failure context is only valid for this session).
  bt_blackboard->set<FailureContext>(k_failure_context, FailureContext{});  return true;
}

void BehaviorTreePilot::buildFactory(const std::string& xml_path,
                                     const std::string& tree_id) {
  // Node/tree registration happens once. Instances are created per navigation
  // attempt (createTreeInstance), so everything that does not depend on the goal
  // stays here.
  this->resolved_tree_id.clear();
  std::string fallback_tree_id;
  // Generic tree: load and hold the configured BehaviorTreeExtension(s);
  // the extensions are responsible for registering all custom nodes and the
  // tree XML; the core embeds no navigation XML/policy.
  for (const auto& so_name : tree_extension_so_names) {
    auto ext_module = ExtensionModule::loadModule(so_name);
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
    if (fallback_tree_id.empty()) {
      fallback_tree_id = ext->defaultTreeId();
    }
    tree_extensions.push_back(std::move(ext));
  }
  // The tree XML is required: an attempt cannot run without one. The extension
  // still registers its own trees as a SOURCE (configurable via
  // behavior_tree_xml), but a configured file that is missing is a deployment
  // error, not something to paper over with a fallback -- the stack would
  // otherwise run a different policy than the config says.
  const bool xml_exists =
      !xml_path.empty() && std::filesystem::exists(xml_path);
  if (!xml_path.empty() && !xml_exists) {
    throw std::runtime_error("BehaviorTreePilot: behavior_tree_xml '" + xml_path +
                             "' does not exist");
  }
  if (xml_exists) {
    factory.registerBehaviorTreeFromFile(xml_path);
  }
  resolved_tree_id =
      tree_id.empty() ? fallback_tree_id : tree_id;
  if (resolved_tree_id.empty()) {
    throw std::runtime_error(
        "BehaviorTreePilot: no bt_tree id configured and extension provides no "
        "default");
  }
  factory_built = true;
}

void BehaviorTreePilot::createTreeInstance() {
  // A fresh blackboard per attempt: the required entries, the extension-provided
  // ones (recovery role ids), and the injected runtime. Nothing is inherited
  // from the previous attempt.
  bt_blackboard = BT::Blackboard::create();
  writeRequiredBlackboardEntries();
  bt_blackboard->set<components::NavigationRuntime::Ptr>(
      k_navigation_runtime, navigation_runtime);
  for (const auto& ext : tree_extensions) {
    ext->provideBlackboardEntries(bt_blackboard);
  }
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
  bt_tree = factory.createTree(resolved_tree_id, bt_blackboard);
  validateTreePorts();
  instance_active.store(true);
}

void BehaviorTreePilot::validateTreePorts() const {
  if (!navigation_runtime) {
    return;  // nothing to resolve against (test doubles / map-free use)
  }
  const auto planner_modules =
      navigation_runtime->moduleNames(ayanami::components::NavigationRuntime::ModuleFamily::Planner);
  const auto smoother_modules =
      navigation_runtime->moduleNames(ayanami::components::NavigationRuntime::ModuleFamily::Smoother);
  const auto controller_modules = navigation_runtime->moduleNames(
      ayanami::components::NavigationRuntime::ModuleFamily::Controller);
  const auto recovery_modules =
      navigation_runtime->moduleNames(ayanami::components::NavigationRuntime::ModuleFamily::Recovery);

  auto check = [](const BT::TreeNode* node, const char* port,
                  const std::vector<std::string>& known, bool required) {
    std::string id;
    if (!node->getInput(port, id) || id.empty()) {
      if (required) {
        throw std::runtime_error(
            "BT: node '" + node->name() + "' has an empty '" + port +
            "' port; the reference resolves to nothing (a recovery role that no "
            "loaded module declares, or an unbound blackboard entry)");
      }
      return;  // empty is legal for *_id: it means "family default"
    }
    if (known.empty()) {
      return;  // runtime cannot enumerate this family: skip the membership check
    }
    if (std::find(known.begin(), known.end(), id) == known.end()) {
      throw std::runtime_error("BT: node '" + node->name() + "' references '" +
                               port + "' = '" + id +
                               "', which is not a loaded module");
    }
  };

  // applyVisitor is non-const, so take a mutable reference to the member.
  BT::Tree& tree = const_cast<BT::Tree&>(bt_tree);
  tree.applyVisitor([&](BT::TreeNode* node) {
    if (node == nullptr) {
      return;
    }
    const std::string& kind = node->registrationName();
    if (kind == "ComputePathToPose") {
      check(node, "planner_id", planner_modules, /*required=*/false);
    } else if (kind == "SmoothPath") {
      check(node, "smoother_id", smoother_modules, /*required=*/false);
    } else if (kind == "FollowPath") {
      check(node, "controller_id", controller_modules, /*required=*/false);
    } else if (kind == "Recovery") {
      check(node, "recovery", recovery_modules, /*required=*/true);
    }
  });
}

void BehaviorTreePilot::releaseTreeInstance() {
  // The tree goes (it is the live machinery, and its whole point is that nothing
  // survives an attempt); the blackboard STAYS as the finished attempt's record.
  // That keeps the outcome observable -- paths, session, plan version, failure
  // context -- without keeping any node state alive, which is what the resident
  // model used to conflate.
  bt_tree = BT::Tree{};
  instance_active.store(false);
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
  bt_blackboard->set<uint64_t>(k_raw_path_revision, 0u);
  bt_blackboard->set<uint64_t>(k_goal_revision, 0u);
  bt_blackboard->set<FailureContext>(k_failure_context, FailureContext{});
}

} // namespace behavior_tree
} // namespace ayanami
