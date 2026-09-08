#pragma once

#include "ayanami/behavior_tree/blackboard_protocol.hpp"
#include "ayanami/behavior_tree/extension.hpp"
#include "ayanami/abstract/components/navigation_server_base.hpp"
#include "ayanami/messages/geometry_msgs.hpp"
#include "ayanami/messages/actions.hpp"

#include <behaviortree_cpp/bt_factory.h>

#include <atomic>
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
 * @brief Resident-tree BT executor (no map dependency).
 *
 * Key disciplines:
 * - **Tree is resident, never rebuilt** `loadFromFile` builds the tree
 *   once; a new goal goes through `beginNavigation` which writes
 *   `pending_goal`, and the run loop's `applyPendingGoal` only rewrites the
 *   blackboard (goal / session / plan_version / reset_stuck_token),
 *   avoiding the heavyweight recreateTree and the IsRobotStuck static state
 *   leaking across tasks.
 * - New goals reach the tick thread via a latest-wins mailbox. The first
 *   goal opens a session; subsequent goals only advance goal_revision, the
 *   planner updates in place, and the controller keeps following the old
 *   path until the new path is available. Ordinary goal revisions never
 *   halt the whole tree nor publish zero velocity.
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
 * BehaviorTreeExtension(s) via ExtensionModule::load_module, calling
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

  /// @brief Load the BT XML and build the resident tree (called once).
  /// When xml_path is empty, use the extension-registered default tree;
  /// when tree_id is empty, use the extension's defaultTreeId. The tree is
  /// built only after the nav_server is ready (action nodes read the
  /// runtime from the blackboard at construction), so this may be called
  /// before setNavigationRuntime. Node registration and the tree XML all
  /// come from the configured BehaviorTreeExtension.
  void loadFromFile(const std::string& xml_path, const std::string& tree_id);

  /// @brief Mailbox delivery of a new goal (latest-wins): overwrite
  /// pending_goal + emitWakeUpSignal to wake run. **No more cross-thread
  /// haltTree** -- the actual preemption (halting the old tree / controller
  /// cancelAll / zero velocity / session advance) all happens in the tick
  /// thread's applyPendingGoal (preemption discipline).
  void beginNavigation(const geometry_msgs::PoseStamped& goal);

  /// @brief Tick loop (tick_frequency Hz, default 100Hz): before each
  /// tick, first consume halt_requested / pending_goal (applyPendingGoal:
  /// turn+halt the old tree + controller cancelAll + one zero velocity +
  /// startNewSession + rewrite the blackboard), then bt_tree.tickOnce +
  /// bt_tree.sleep(period). Blocking; the host calls it on its own thread.
  void run();

  /// @brief Request stopping all RUNNING actions (bt_tree.haltTree runs on
  /// the tick thread; wakes run if sleeping). Mainly for requests before
  /// stop/shutdown.
  void halt();

  /// @brief Terminate: kill_switch + wake + join the internal tick_thread (if any).
  void stop();

  // ---- Test/host support accessors ----
  const BT::Tree& tree() const { return bt_tree; }
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
  /// advance the revision and rewrite the goal / revision / failure context
  /// blackboard entries. Returns whether a goal was consumed.
  bool applyPendingGoal();

  /// @brief Emit the session-level navigation terminal outcome (once per
  /// tree life). status: SUCCEEDED / ABORTED / CANCELED.
  void emitNavigationResult(actions::GoalStatus status);

  /// @brief run-thread only: whether the pending_goal mailbox is non-empty
  /// (removes the race where tickOnce's inner layer consumed the wake signal
  /// with a stale blackboard, see the run comment).
  bool hasPendingGoal();

  /// @brief Build the tree: load/hold BehaviorTreeExtension(s) -> register
  /// nodes -> register trees (file or extension default) -> createTree ->
  /// validate the 17 blackboard entries.
  void buildTree(const std::string& xml_path, const std::string& tree_id);

  /// @brief Write the 17 required blackboard entries (values from the configure cache).
  void writeRequiredBlackboardEntries();

  BT::Tree bt_tree;
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
  std::optional<geometry_msgs::PoseStamped> pending_goal;
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
  bool load_requested = false;
  bool tree_built = false;
};

} // namespace behavior_tree
} // namespace ayanami
