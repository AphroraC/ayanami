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
 * @brief One-tree-per-navigation-attempt BT executor (no map dependency).
 *
 * Model: the factory (node registration + tree XML) is built ONCE and held;
 * the TREE INSTANCE is created per navigation attempt -- a new goal arriving
 * after the previous attempt ended builds a fresh tree and a fresh blackboard,
 * and the instance is released when the attempt reaches a terminal status.
 *
 * A tree life is therefore a tree instance. That is what makes the whole
 * cross-life state problem disappear: RateController single-shot caches,
 * RecoveryNode retry budgets and the RecoverySequence cursor cannot leak into
 * the next attempt because there is no next attempt on that object. The former
 * resident-tree design had to bolt on a reset protocol (a mandated node
 * interface, a build-time validation pass that threw on violating nodes, and a
 * full logical reset on every terminal tick) purely to emulate this; all of it
 * is gone, and a third-party control/decorator node no longer has to implement
 * anything for the framework.
 *
 * - New goals reach the tick thread via a latest-wins mailbox. The first goal
 *   of an attempt opens a session and creates the tree; a goal REVISION while
 *   the tree is still running keeps the current instance and only rewrites the
 *   blackboard (goal / session / goal_revision / failure_context), so the
 *   running controller keeps following its path until the revised one lands.
 *   Goal revisions never halt the whole tree nor publish zero velocity.
 * - `halt()` only sets a request flag (atomic halt_requested); the run
 *   thread consumes it and halts locally, providing a safe tree stop for
 *   the stop/shutdown path (join before destruction already guarantees no
 *   concurrent tick).
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
  /// May be called before or after loadFromFile: if loadFromFile already
  /// requested tree building and this call arrives later, the tree is built
  /// now. The blackboard only receives NavigationRuntime::Ptr (shared_ptr),
  /// no raw-pointer compatibility layer.
  void setNavigationRuntime(components::NavigationRuntime::Ptr runtime);

  /// @brief Build the FACTORY once: load the configured BehaviorTreeExtension(s),
  /// register their nodes and trees, and remember which tree id to instantiate.
  ///
  /// No tree instance is created here: instances are per navigation attempt.
  /// When xml_path is empty the extension-registered tree is used; when tree_id
  /// is empty the extension's defaultTreeId applies.
  void loadFromFile(const std::string& xml_path, const std::string& tree_id);

  /// @brief Mailbox delivery of a new goal (latest-wins): overwrite
  /// pending_goal + emitWakeUpSignal to wake run. **No cross-thread
  /// haltTree**: the first goal opens a session on the tick thread;
  /// subsequent goals are IN-PLACE revisions (goal_revision advance + goal
  /// blackboard rewrite) -- no tree halt, no controller cancel (the running
  /// controller keeps following its path until the revised one lands via
  /// in-place updatePlan). Halting stays on the halt() path.
  void beginNavigation(const geometry_msgs::PoseStamped& goal);

  /// @brief Tick loop (tick_frequency Hz, default 100Hz): before each
  /// tick, first consume halt_requested / pending_goal (applyPendingGoal:
  /// session/revision advance + goal/session/revision/plan_version/
  /// failure_context blackboard rewrite), then bt_tree.tickOnce +
  /// bt_tree.sleep(period). Blocking; the host calls it on its own thread.
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

  /// @brief True while this attempt has a live tree instance, i.e. while the
  /// blackboard below is meaningful. An Idle pilot (no attempt in flight, or the
  /// previous one finished) holds no instance: tree() is a default-constructed
  /// Tree and blackboard() is null. Poll this before inspecting either.
  bool attemptActive() const { return instance_active.load(); }

  /// @brief The current attempt's tree instance (only valid while attemptActive).
  const BT::Tree& tree() const { return bt_tree; }
  /// @brief The current attempt's blackboard -- or, once it finished, the
  /// record of that finished attempt (paths, session, plan version, failure
  /// context). Null only before the first goal of the process.
  ///
  /// One blackboard per attempt: a new attempt replaces it wholesale, so nothing
  /// is inherited from the previous one. Observing a finished attempt is why the
  /// record outlives the tree; no NODE state does.
  const BT::Blackboard::Ptr& blackboard() const { return bt_blackboard; }

  /// @brief Shared session/version state (the same object as NavigationServerBase).
  std::shared_ptr<components::NavigationSessionState> sessionState() const {
    return session_state;
  }

 private:
  /// @brief run-thread only: if halt_requested is set, locally halt the old
  /// tree + controller cancelAll (stop RUNNING actions, never cross-thread).
  void consumeHaltRequest();

  /// @brief run-thread only: the first goal opens a session; later goals
  /// advance the revision and rewrite the goal / session / revision /
  /// plan_version / failure context blackboard entries (no tree halt, no
  /// controller cancel: the running controller keeps following its path
  /// until the new one is available, and preemption of in-flight tasks
  /// happens through the planner RateController token + in-place plan
  /// update). Returns whether a goal was consumed.
  bool applyPendingGoal();

  /// @brief Emit the session-level navigation terminal outcome (once per
  /// tree life). status: SUCCEEDED / ABORTED / CANCELED.
  void emitNavigationResult(actions::GoalStatus status);

  /// @brief run-thread only: whether the pending_goal mailbox is non-empty
  /// (removes the race where tickOnce's inner layer consumed the wake signal
  /// with a stale blackboard, see the run comment).
  bool hasPendingGoal();

  /// @brief Build the FACTORY (extensions -> nodes -> trees) and validate.
  /// Runs once, from loadFromFile.
  void buildFactory(const std::string& xml_path, const std::string& tree_id);

  /// @brief Create a fresh tree instance + blackboard for one navigation
  /// attempt. Tick thread only. Throws when the tree cannot be built or the
  /// blackboard protocol is violated.
  void createTreeInstance();

  /// @brief Drop the current instance (terminal status reached). The factory and
  /// the extensions stay; only the per-attempt object goes away.
  void releaseTreeInstance();

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

  /// @brief The tree instance for the CURRENT navigation attempt (default when
  /// the pilot is idle: an attempt is over or has not started).
  BT::Tree bt_tree;
  /// @brief Held across attempts: node/tree registration is the expensive part
  /// and does not depend on the goal.
  BT::BehaviorTreeFactory factory;
  /// @brief Tree id to instantiate per attempt (resolved once from the config
  /// or the extension's defaultTreeId).
  std::string resolved_tree_id;
  /// @brief Set once the factory is ready; gates tree instance creation.
  bool factory_built = false;
  BT::Blackboard::Ptr bt_blackboard;
  components::NavigationRuntime::Ptr navigation_runtime;
  // Generic tree: loaded and held BehaviorTreeExtension(s) (requirement 3).
  // Nodes/trees are registered before createTree. Destruction order:
  // extensions are released before bt_tree, no circular dependency.
  std::vector<ayanami::behavior_tree::BehaviorTreeExtension::Ptr>
      tree_extensions;
  std::atomic<bool> running{false};
  std::atomic<bool> kill_switch{false};
  // Halt request flag: halt only sets it + wakes; the run thread consumes it (haltTree on the tick thread).
  std::atomic<bool> halt_requested{false};
  // Session/version single source of truth: shares the same object with
  // NavigationServerBase (injected via setNavigationRuntime), replacing the
  // previously independent session counters (spec single-source discipline).
  std::shared_ptr<components::NavigationSessionState> session_state;
  std::thread tick_thread;
  double tick_frequency = 100.0;
  std::mutex goal_mutex;
  /// @brief Idle wait (no tree instance): a new goal must wake the loop without
  /// the loop having to poke a tree object that does not exist yet.
  std::condition_variable goal_cv;
  std::optional<geometry_msgs::PoseStamped> pending_goal;
  /// @brief Reason an attempt could not be built (tree XML / module reference /
  /// blackboard protocol); reported with the terminal navigation result.
  std::string failure_context;
  uint64_t goal_revision{0};
  bool navigation_started{false};
  /// @brief One navigation-result emission per tree life (reset by every
  /// consumed goal). Session-level observability exit via
  /// NavigationCallbacks::on_navigation_result.
  bool navigation_result_emitted{false};

  // configure cache (needed by loadFromFile to write the blackboard);
  // recovery role ids are extension-owned, not cached here.
  double planner_frequency = 10.0;
  double smoother_frequency = -1.0;
  double controller_frequency = 20.0;

  // Generic tree: extension .so specified by navigation.behavior_tree_extension.
  std::vector<std::string> tree_extension_so_names;

  std::string xml_path;
  std::string tree_id;
  /// @brief Set while an attempt's instance exists (guards beginNavigation and
  /// the create/release transitions). Atomic: written by the tick thread when it
  /// creates/releases the instance, read by attemptActive() from any thread.
  std::atomic<bool> instance_active{false};
};

} // namespace behavior_tree
} // namespace ayanami
