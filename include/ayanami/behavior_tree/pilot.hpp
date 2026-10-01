#pragma once

#include "ayanami/behavior_tree/blackboard_protocol.hpp"
#include "ayanami/behavior_tree/extension.hpp"
#include "ayanami/abstract/components/navigation_server_base.hpp"
#include "ayanami/messages/geometry_msgs.hpp"
#include "ayanami/messages/actions.hpp"

#include <behaviortree_cpp/bt_factory.h>

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace ayanami {
namespace behavior_tree {

/**
 * @brief Resident-tree BT executor (nav2 BtActionServer model; no map
 * dependency).
 *
 * Model: the factory (node registration + tree XML) is built ONCE, and the
 * TREE + BLACKBOARD are built once with it (on the first goal, so the
 * injected NavigationRuntime is already in place -- nodes capture the
 * runtime at construction). The tree OUTLIVES navigation attempts: when an
 * attempt reaches a terminal status the pilot runs bt_tree.haltTree() (nav2
 * haltAllActions) and goes idle; the next goal re-arms the same tree.
 *
 * Cross-attempt state is cleaned by two cooperating mechanisms:
 * - haltTree returns every node to IDLE and runs its halt() override; every
 *   shipped node either clears its cached state there or re-baselines at
 *   the IDLE edge on its first tick of the next attempt (nav2 discipline).
 * - applyPendingGoal resets the per-attempt blackboard keys (goal,
 *   revisions, failure context, and the finished attempt's path artifacts)
 *   so no node can read the previous route.
 * Tokens ({goal_revision}, {raw_path_revision}) stay monotonic across
 * attempts, so token caches kept by nodes still detect every change.
 *
 * - New goals reach the tick thread via a latest-wins mailbox. The first
 *   goal of the process builds the tree; a goal REVISION while an attempt
 *   is in flight keeps the tree untouched and only rewrites the blackboard
 *   (goal / session / goal_revision / failure_context), so the running
 *   controller keeps following its path until the revised one lands.
 *   Goal revisions never halt the tree nor publish zero velocity.
 * - `halt()` only sets a request flag (atomic halt_requested); the run
 *   thread consumes it and halts locally (haltTree + CANCELED result),
 *   providing a safe tree stop for the stop/shutdown path (join before
 *   destruction already guarantees no concurrent tick).
 * - `run()` is a blocking tick loop (tick_frequency Hz, default 100Hz),
 *   called by the host on a separate thread; `bt_tree.sleep` can be woken
 *   by `emitWakeUpSignal`.
 *
 * Generic tree: **the core contains no default navigation XML/policy**.
 * configure caches the extension .so name(s) specified by
 * navigation.behavior_tree_extension; buildTree loads and holds the
 * BehaviorTreeExtension(s) via ExtensionModule::loadModule, calling
 * registerNodes + registerTrees in order, then createTree with the
 * configured tree_id (defaulting to the extension's defaultTreeId). The
 * blackboard protocol (blackboard_protocol.hpp) and the runtime core stay
 * in the core.
 */
class BehaviorTreePilot {
public:
  BehaviorTreePilot();
  ~BehaviorTreePilot();

  BehaviorTreePilot(const BehaviorTreePilot&) = delete;
  BehaviorTreePilot& operator=(const BehaviorTreePilot&) = delete;

  /// @brief Read the config's `navigation` section (behavior_tree_xml/id/
  /// frequencies/ids, spec §9.3), caching for loadFromFile to write the
  /// blackboard. setNavigationRuntime is called explicitly by the caller.
  /// When xml is absent the production path is empty -> loadFromFile("")
  /// uses the BehaviorTreeExtension's embedded default tree (production
  /// single source); tests may pass a custom behavior_tree_xml explicitly.
  void configure(const std::string& config_path);

  /// @brief Inject the NavigationRuntime ("construction-time injection").
  /// May be called before or after loadFromFile: the resident tree is built
  /// on the FIRST goal (not in loadFromFile), so the runtime is in the
  /// blackboard before any node is constructed. The blackboard only
  /// receives NavigationRuntime::Ptr (shared_ptr), no raw-pointer
  /// compatibility layer.
  void setNavigationRuntime(components::NavigationRuntime::Ptr runtime);

  /// @brief Build the FACTORY once: load the configured BehaviorTreeExtension(s),
  /// register their nodes and trees, and remember which tree id to instantiate.
  ///
  /// No tree instance is created here: the resident tree is built on the
  /// first goal (applyPendingGoal), once the NavigationRuntime is injected.
  /// When xml_path is empty the extension-registered tree is used; when
  /// tree_id is empty the extension's defaultTreeId applies.
  void loadFromFile(const std::string& xml_path, const std::string& tree_id);

  /// @brief Mailbox delivery of a new goal (latest-wins): overwrite
  /// pending_goal + emitWakeUpSignal to wake run. **No cross-thread
  /// haltTree**: the first goal builds the tree on the tick thread;
  /// subsequent goals while an attempt is in flight are IN-PLACE revisions
  /// (goal_revision advance + goal blackboard rewrite) -- no tree halt, no
  /// controller cancel (the running controller keeps following its path
  /// until the revised one lands via in-place updatePlan). Halting stays on
  /// the halt() path.
  void beginNavigation(const geometry_msgs::PoseStamped& goal);

  /// @brief Tick loop (tick_frequency Hz, default 100Hz): before each
  /// tick, first consume halt_requested / pending_goal (applyPendingGoal:
  /// tree build once / revision advance + per-attempt blackboard rewrite),
  /// then bt_tree.tickOnce + bt_tree.sleep(period). Blocking; the host
  /// calls it on its own thread.
  void run();

  /// @brief Request stopping all RUNNING actions (bt_tree.haltTree runs on
  /// the tick thread; wakes run if sleeping). Mainly for requests before
  /// stop/shutdown.
  void halt();

  /// @brief Terminate: kill_switch + wake + join the internal tick_thread (if any).
  void stop();

  // ---- Test/host support accessors ----
  /// @brief True once the factory is built (node/tree registration done): the
  /// stack is wired and the first goal can be dispatched. Independent of any
  /// attempt, so the host can validate configuration before navigating.
  bool ready() const { return factory_built; }

  /// @brief True while an attempt is in flight (a goal was consumed and the
  /// tree has not reached a terminal status / halt yet). An idle pilot --
  /// no goal consumed yet, or the previous attempt finished -- still owns
  /// the resident tree and blackboard, but they hold the FINISHED (or
  /// default, pre-first-goal) state.
  bool attemptActive() const { return attempt_active.load(); }

  /// @brief The resident tree (valid once the first goal built it; the same
  /// instance across attempts, all nodes IDLE between attempts).
  const BT::Tree& tree() const { return bt_tree; }
  /// @brief The RESIDENT blackboard -- one instance for the process life,
  /// shared by every attempt (null only before the first goal built the
  /// tree). Between attempts it holds the last finished attempt's record
  /// (paths, session, plan version, failure context) until the next goal's
  /// applyPendingGoal rewrites the per-attempt keys.
  const BT::Blackboard::Ptr& blackboard() const { return bt_blackboard; }

  /// @brief Shared session/version state (the same object as NavigationServerBase).
  std::shared_ptr<components::NavigationSessionState> sessionState() const { return session_state; }

private:
  /// @brief run-thread only: if halt_requested is set, locally halt the tree
  /// + controller cancelAll (stop RUNNING actions, never cross-thread). The
  /// attempt ends CANCELED; the resident tree stays (all nodes IDLE).
  void consumeHaltRequest();

  /// @brief run-thread only: the first goal builds the resident tree; later
  /// goals either start a NEW attempt on it (fresh session -- stale results
  /// of the finished attempt are rejected -- plus defensive haltTree +
  /// per-attempt blackboard reset) or advance the revision and rewrite the
  /// goal / session / revision / failure context entries (no tree halt, no
  /// controller cancel: the running controller keeps following its path
  /// until the new one is available, and preemption of in-flight tasks
  /// happens through the planner RateController token + in-place plan
  /// update). Returns whether a goal was consumed.
  bool applyPendingGoal();

  /// @brief Emit the session-level navigation terminal outcome (once per
  /// attempt). status: SUCCEEDED / ABORTED / CANCELED.
  void emitNavigationResult(actions::GoalStatus status);

  /// @brief run-thread only: whether the pending_goal mailbox is non-empty
  /// (removes the race where tickOnce's inner layer consumed the wake signal
  /// with a stale blackboard, see the run comment).
  bool hasPendingGoal();

  /// @brief Build the FACTORY (extensions -> nodes -> trees) and validate.
  /// Runs once, from loadFromFile.
  void buildFactory(const std::string& xml_path, const std::string& tree_id);

  /// @brief Build the RESIDENT tree + blackboard: required entries,
  /// extension-provided entries (recovery role ids), the injected runtime,
  /// then createTree + port validation. Runs ONCE, on the tick thread, from
  /// the first applyPendingGoal (so the runtime is guaranteed injected).
  /// Throws when the tree cannot be built or the blackboard protocol is
  /// violated (a configuration error -- the pilot stops ticking).
  void buildResidentTree();

  /// @brief Resolve every module id the tree references, while the tree is being
  /// built. Throws when a reference resolves to nothing.
  ///
  /// Two failures this catches early, both of which otherwise surface only when
  /// the corresponding branch actually runs:
  /// - a Recovery node whose role is not declared by any loaded recovery module
  ///   resolves to an EMPTY id -- the error appears the moment a recovery is
  ///   needed, i.e. exactly when the robot is already in trouble;
  /// - an action node naming a module id that is not loaded.
  void validateTreePorts() const;

  /// @brief Write the core-protocol blackboard entries (values from the
  /// configure cache).
  void writeRequiredBlackboardEntries();

  /// @brief The RESIDENT tree instance: built once (first goal), kept across
  /// navigation attempts; every attempt ends with haltTree (all nodes IDLE).
  BT::Tree bt_tree;
  /// @brief Held for the process life: node/tree registration is the
  /// expensive part and does not depend on the goal.
  BT::BehaviorTreeFactory factory;
  /// @brief Tree id to instantiate (resolved once from the config or the
  /// extension's defaultTreeId).
  std::string resolved_tree_id;
  /// @brief Set once the factory is ready; gates tree building.
  bool factory_built = false;
  /// @brief Set once the resident tree exists (first goal); never reset
  /// (a build failure stops the pilot for good -- it is a config error).
  bool tree_built = false;
  BT::Blackboard::Ptr bt_blackboard;
  components::NavigationRuntime::Ptr navigation_runtime;
  // Generic tree: loaded and held BehaviorTreeExtension(s) (requirement 3).
  // Nodes/trees are registered before createTree. Destruction order:
  // extensions are released before bt_tree, no circular dependency.
  std::vector<ayanami::behavior_tree::BehaviorTreeExtension::Ptr> tree_extensions;
  std::atomic<bool> running{false};
  std::atomic<bool> kill_switch{false};
  /// @brief Consecutive tick-time exceptions (reset by any clean tick). A
  /// streak reaching kMaxTickErrorStreak stops the pilot for good: a
  /// repeating runtime failure is not transient. Single occurrences abort
  /// only the attempt (supervised abort in run()); the tick loop survives.
  static constexpr std::uint32_t kMaxTickErrorStreak = 5;
  std::uint32_t tick_error_streak{0};
  // Halt request flag: halt only sets it + wakes; the run thread consumes it (haltTree on the tick thread).
  std::atomic<bool> halt_requested{false};
  // Session/version single source of truth: shares the same object with
  // NavigationServerBase (injected via setNavigationRuntime), replacing the
  // previously independent session counters (spec single-source discipline).
  std::shared_ptr<components::NavigationSessionState> session_state;
  std::thread tick_thread;
  double tick_frequency = 100.0;
  std::mutex goal_mutex;
  /// @brief Idle wait (no attempt in flight): a new goal must wake the loop
  /// without the loop ticking the (idle, all-IDLE) resident tree.
  std::condition_variable goal_cv;
  std::optional<geometry_msgs::PoseStamped> pending_goal;
  /// @brief Reason an attempt could not be built (tree XML / module reference /
  /// blackboard protocol); reported with the terminal navigation result.
  std::string failure_context;
  uint64_t goal_revision{0};
  bool navigation_started{false};
  /// @brief One navigation-result emission per attempt (reset by every
  /// consumed goal). Session-level observability exit via
  /// NavigationCallbacks::on_navigation_result.
  bool navigation_result_emitted{false};

  // configure cache (needed by loadFromFile to write the blackboard);
  // recovery role ids are extension-owned, not cached here.
  double planner_frequency = 10.0;
  double smoother_frequency = -1.0;
  double controller_frequency = 20.0;
  /// @brief Outer RecoveryNode retry budget (navigation.max_nav_retries),
  /// written to the blackboard as {nav_retries} for the tree XML to bind.
  uint32_t max_nav_retries = 6;
  /// @brief Navigation-level oscillation trigger parameters
  /// (navigation.oscillation_{distance,timeout}); written to the
  /// blackboard for the IsRobotProgressing gate in the tree XML.
  double oscillation_distance = 0.1;
  double oscillation_timeout = 5.0;
  /// @brief BT deadline budgets for the planner/smoother branches
  /// (navigation.planner_timeout/smoother_timeout, seconds; <= 0 =
  /// unlimited -> written as 0 ms) and the outer RecoveryNode same-cause
  /// pruning budget (navigation.max_same_cause, 0 = off).
  double planner_timeout = 2.0;
  double smoother_timeout = 2.0;
  uint32_t max_same_cause = 3;

  // Generic tree: extension .so specified by navigation.behavior_tree_extension.
  std::vector<std::string> tree_extension_so_names;

  std::string xml_path;
  std::string tree_id;
  /// @brief Set while an attempt is in flight (goal consumed, tree not yet
  /// terminal/halted). Atomic: written by the tick thread, read by
  /// attemptActive() from any thread.
  std::atomic<bool> attempt_active{false};
};

}  // namespace behavior_tree
}  // namespace ayanami
