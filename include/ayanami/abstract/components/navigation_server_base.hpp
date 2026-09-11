#pragma once

#include "ayanami/abstract/components/controller_action.hpp"
#include "ayanami/abstract/components/controller_execution.hpp"
#include "ayanami/abstract/components/navigation_session_state.hpp"
#include "ayanami/abstract/components/planner_action.hpp"
#include "ayanami/abstract/components/planner_execution.hpp"
#include "ayanami/abstract/components/module_manager.hpp"
#include "ayanami/abstract/components/recovery_action.hpp"
#include "ayanami/abstract/components/recovery_execution.hpp"
#include "ayanami/abstract/components/smoother_action.hpp"
#include "ayanami/abstract/components/smoother_execution.hpp"
#include "ayanami/abstract/core/abstract_controller.hpp"
#include "ayanami/abstract/core/abstract_goal_checker.hpp"
#include "ayanami/abstract/core/abstract_planner.hpp"
#include "ayanami/abstract/core/abstract_progress_checker.hpp"
#include "ayanami/abstract/core/abstract_recovery.hpp"
#include "ayanami/abstract/core/abstract_smoother.hpp"
#include "ayanami/interfaces/callbacks.hpp"
#include "ayanami/messages/actions.hpp"

#include <atomic>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace ayanami {
namespace components {

class NavigationRuntime {
public:
  using Ptr = std::shared_ptr<NavigationRuntime>;
  virtual ~NavigationRuntime() = default;

  virtual TaskHandle<ayanami::actions::GetPathResult> requestPath(
      const ayanami::actions::GetPathGoal& goal) = 0;
  virtual TaskHandle<ayanami::actions::SmoothPathResult> requestSmoothing(
      const ayanami::actions::SmoothPathGoal& goal) = 0;
  virtual TaskHandle<ayanami::actions::ExePathResult> requestExecution(
      const ayanami::actions::ExePathGoal& goal) = 0;
  virtual TaskHandle<ayanami::actions::RecoveryResult> requestRecovery(
      const ayanami::actions::RecoveryGoal& goal) = 0;
  virtual uint64_t currentSessionId() const = 0;
  virtual uint64_t currentPlanVersion() const = 0;
  virtual bool sessionIsCurrent(uint64_t session_id) const = 0;
  virtual bool planVersionIsCurrent(uint64_t plan_version) const = 0;
  virtual void incrementPlanVersion() = 0;
  virtual bool requestPlanUpdate(
      uint64_t goal_id, const ayanami::actions::ExePathGoal& goal) = 0;
  /// @brief Planner in-place goal update (the requestPlanUpdate analogue):
  /// while a planning task is still running, its start/goal/tolerance are
  /// replaced wholesale (no cancel/join/new Execution); the same TaskHandle
  /// keeps running and the run loop replans for the latest goal. Called by
  /// ComputePathToPose when the goal port changes mid-flight. Default:
  /// unsupported (test doubles / map-free servers).
  virtual bool requestPathUpdate(uint64_t goal_id,
                                 const ayanami::actions::GetPathGoal& goal) {
    (void)goal_id;
    (void)goal;
    return false;
  }
  virtual void cancelAllControllers() = 0;
  virtual void cancelAllPlanners() {}
  virtual void cancelAllSmoothers() {}
  /// @brief Non-blocking preemption: mark the running controller canceled
  /// (a zeroed command is published synchronously by cancel()) and let the
  /// task wind down on its own thread. Used by runtime preemption paths
  /// (halt / recovery start) that run on the BT tick thread and must not
  /// block on a slow module call.
  virtual void preemptAllControllers() {}
  /// @brief Module family, for the name queries below.
  enum class ModuleFamily { Planner, Smoother, Controller, Recovery };

  /// @brief Names of the loaded modules of one family.
  ///
  /// The BT-visible name surface: the tree references modules by id (either
  /// directly or through a recovery role), and building the tree is the last
  /// moment where an id that resolves to nothing can be reported as a
  /// configuration error instead of a mid-navigation failure. Empty means "this
  /// runtime cannot enumerate that family" (test doubles), in which case callers
  /// skip the membership check.
  virtual std::vector<std::string> moduleNames(ModuleFamily) const {
    return {};
  }

  /// @brief Path validity query (consumed by the BT IsPathValid condition
  /// node; the BT-readable surface of the responsibility table's "is the
  /// path stale"). Default = valid iff non-empty; map-family hosts override
  /// with real validation (CostmapNavigationServer ->
  /// AbstractMapContext::isPathValid).
  virtual bool isPathValid(const ayanami::nav_msgs::Path& path) const {
    return !path.poses.empty();
  }
  virtual void setSessionState(
      std::shared_ptr<NavigationSessionState> session) = 0;
};

/**
 * @brief Navigation server base class (no map dependency).
 *
 * Holds the module managers, the four Action families and the shared
 * navigation session.
 * Map objects (costmap/gridmap) are injected by derived servers (e.g.
 * CostmapNavigationServer, §5.4) overriding the virtual initialize*Module
 * hooks; derived servers override createActions() to build map-aware
 * Actions (production path = costmap_action). The base createActions()
 * assembles abstract Actions via the virtual new*Execution factories (for
 * map-free servers and test overrides).
 *
 * Mainline control flow: requestPath/requestSmoothing/requestExecution/
 * requestRecovery return TaskHandles straight to the caller (BT nodes);
 * the side path is emitted by each Action on
 * NavigationCallbacks::on*_result (rviz extension/monitoring). A planning
 * failure does not automatically trigger recovery via the side path --
 * recovery is scheduled only by RecoveryNode/Recovery nodes inside the BT.
 */
class NavigationServerBase : public NavigationRuntime {
public:

  virtual ~NavigationServerBase();

  /// @brief Read the config.json global section -> dlopen each of the 6
  /// sub-configs + call the virtual initialize*Module hooks ->
  /// createActions.
  void loadModules(const std::string& config_path);

  /// @brief Fetch modules by their default*_ names and assemble the Actions
  /// (one per module; derived servers override to return map-aware Actions,
  /// e.g. CostmapNavigationServer -> Costmap*Action).
  virtual void createActions();

  /// @brief Start the server (idempotent; reserved lifecycle hook). Derived
  /// servers override to start their own resident machinery (e.g. the
  /// costmap map-writer thread) on top of the module activation.
  virtual void start();

  /// @brief Stop the server: cancel and join all RUNNING actions (idempotent).
  virtual void stop();

  /// @brief Lookup an action of the first three families by module id
  /// (nav2 semantics: every configured module gets an action; an empty id
  /// resolves to the family default = the first configured module).
  /// Recovery actions remain selectable by their configured recovery id.
  PlannerAction* plannerAction(const std::string& id = "") const;
  SmootherAction* smootherAction(const std::string& id = "") const;
  ControllerAction* controllerAction(const std::string& id = "") const;
  RecoveryAction* recoveryAction(const std::string& name) const;
  std::vector<std::string> recoveryActionNames() const;

  /// @brief Loaded module names per family (see NavigationRuntime::moduleNames).
  std::vector<std::string> moduleNames(ModuleFamily family) const override;

  /// @brief Mainline TaskHandle requests. Planner, smoother and controller each
  /// dispatch to their one configured action; recovery uses recovery_id.
  /// virtual so test mocks can override.
  virtual TaskHandle<ayanami::actions::GetPathResult> requestPath(
      const ayanami::actions::GetPathGoal& goal);
  virtual TaskHandle<ayanami::actions::SmoothPathResult> requestSmoothing(
      const ayanami::actions::SmoothPathGoal& goal);
  virtual TaskHandle<ayanami::actions::ExePathResult> requestExecution(
      const ayanami::actions::ExePathGoal& goal);
  virtual TaskHandle<ayanami::actions::RecoveryResult> requestRecovery(
      const ayanami::actions::RecoveryGoal& goal);

  /// @brief Controller in-place plan update (spec): while the goal is still
  /// running, only the path + version are updated, with no cancel/join/new
  /// Execution. Called by FollowPathNode when plan_version changes.
  virtual bool requestPlanUpdate(uint64_t goal_id,
                                 const ayanami::actions::ExePathGoal& goal);

  /// @brief Planner in-place goal update (see NavigationRuntime).
  bool requestPathUpdate(uint64_t goal_id,
                         const ayanami::actions::GetPathGoal& goal) override;

  /// @brief Cancel the entire controller action map (all logical ids, not
  /// just the default id): cleans up every possibly running controller task
  /// on preemption / new session / halt. Derived classes may override.
  virtual void cancelAllControllers();

  /// @brief Non-blocking variant of cancelAllControllers (see
  /// NavigationRuntime::preemptAllControllers).
  void preemptAllControllers() override;
  virtual void cancelAllPlanners();
  virtual void cancelAllSmoothers();

  // ---- Session/version (single source of truth = shared NavigationSessionState) ----
  uint64_t currentSessionId() const;
  uint64_t currentPlanVersion() const;
  void incrementPlanVersion();
  bool sessionIsCurrent(uint64_t session_id) const;
  bool planVersionIsCurrent(uint64_t plan_version) const;
  /// @brief Start a new session (session+1 and plan_version reset to zero).
  void startNewSession();

  /// @brief Inject the shared session state (passed in when
  /// BehaviorTreePilot calls setNavigationRuntime, so pilot and server share
  /// one session/plan_version count); falls back to the built-in instance
  /// when not injected.
  void setSessionState(std::shared_ptr<NavigationSessionState> session);
  std::shared_ptr<NavigationSessionState> sessionState() const;

protected:
  // Key virtual hooks: derived servers override these to inject map
  // objects. module_config is the module's own module json (already
  // parsed); derived servers use it to build the ModuleContext passed to
  // the family-level configure.
  virtual bool initializePlannerModule(
      const std::string& name, const ayanami::Config& module_config,
      ayanami::abstract::AbstractPlanner::Ptr);
  virtual bool initializeSmootherModule(
      const std::string& name, const ayanami::Config& module_config,
      ayanami::abstract::AbstractSmoother::Ptr);
  virtual bool initializeControllerModule(
      const std::string& name, const ayanami::Config& module_config,
      ayanami::abstract::AbstractController::Ptr);
  virtual bool initializeRecoveryModule(
      const std::string& name, const ayanami::Config& module_config,
      ayanami::abstract::AbstractRecovery::Ptr);
  // Checker virtual hooks: no map dependency, default return true (extension families).
  virtual bool initializeGoalCheckerModule(
      const std::string& name, const ayanami::Config& module_config,
      ayanami::abstract::AbstractGoalChecker::Ptr);
  virtual bool initializeProgressCheckerModule(
      const std::string& name, const ayanami::Config& module_config,
      ayanami::abstract::AbstractProgressChecker::Ptr);

  // Factories: derived servers override to return map-aware Executions.
  // The base createActions assembles abstract Actions through them (the
  // map-free server and test override surface). Defaults return the
  // abstract versions.
  // No longer used by CostmapNavigationServer (its createActions constructs
  // Costmap*Action directly).
  virtual std::unique_ptr<SmootherExecution> newSmootherExecution(
      const std::string& name, ayanami::abstract::AbstractSmoother::Ptr);
  virtual std::unique_ptr<PlannerExecution> newPlannerExecution(
      const std::string& name, ayanami::abstract::AbstractPlanner::Ptr);
  virtual std::unique_ptr<ControllerExecution> newControllerExecution(
      const std::string& name, ayanami::abstract::AbstractController::Ptr);
  virtual std::unique_ptr<RecoveryExecution> newRecoveryExecution(
      const std::string& name, ayanami::abstract::AbstractRecovery::Ptr);

  /// @brief Apply server-level assembly to a controller execution (goal/
  /// progress checker injection + velocity limits). Shared by the base and
  /// costmap derived createActions; virtual so derived/test code can inject
  /// custom checker assembly.
  virtual void configureControllerExecution(
      std::shared_ptr<ControllerExecution> exec) const;

  /// @brief Apply server-level assembly to a recovery execution (velocity limits). Same as above.
  virtual void configureRecoveryExecution(
      std::shared_ptr<RecoveryExecution> exec) const;

  /// @brief Shared module managers for the four families (+ checkers).
  ModuleManager<ayanami::abstract::AbstractPlanner> planner_modules;
  ModuleManager<ayanami::abstract::AbstractSmoother> smoother_modules;
  ModuleManager<ayanami::abstract::AbstractController> controller_modules;
  ModuleManager<ayanami::abstract::AbstractRecovery> recovery_modules;
  ModuleManager<ayanami::abstract::AbstractGoalChecker> goal_checker_modules;
  ModuleManager<ayanami::abstract::AbstractProgressChecker>
      progress_checker_modules;
  // Multi-module action maps (nav2 semantics): every configured module gets
  // an action; requests select by goal id, empty = default (first
  // configured).
  std::map<std::string, std::unique_ptr<PlannerAction>> planner_actions;
  std::map<std::string, std::unique_ptr<SmootherAction>> smoother_actions;
  std::map<std::string, std::unique_ptr<ControllerAction>> controller_actions;
  std::map<std::string, std::unique_ptr<RecoveryAction>> recovery_actions;
  std::string default_planner;
  std::string default_smoother;
  std::string default_controller;
  std::vector<std::string> recovery_sequence;
  std::string default_goal_checker;
  std::string default_progress_checker;
  // Velocity limit cache (navigation.max_vel_x/max_vel_y/max_vel_theta,
  // parameters non-negative; max_vel_y=0 forbids lateral motion).
  // loadModules reads the cache; the createActions controller execution
  // factory injects ControllerExecution::setMaximumVelocity. The default
  // Infinity = no clamping (consistent with ControllerExecution defaults).
  double max_vel_x{std::numeric_limits<double>::infinity()};
  double max_vel_y{std::numeric_limits<double>::infinity()};
  double max_vel_theta{std::numeric_limits<double>::infinity()};
  // Controller loop rate (navigation.controller_frequency, Hz): cached at
  // loadModules and injected into every controller execution via
  // configureControllerExecution -> setControllerFrequency. The execution
  // owns the computeVelocityCommand cadence; the BT FollowPath "frequency"
  // port is informational only (blackboard mirror for the tree, never a
  // control path).
  double controller_frequency{20.0};
  // Patience (navigation.{controller_patience,recovery_patience}, seconds):
  // controller = nav2 failure_tolerance analogue (<0 = tolerate forever
  // [default], 0 = fail immediately, >0 = tolerance window; covers both "no
  // valid command" and "stale map" -> MAP_STALE terminal), recovery = per-run
  // timeout (<=0 = disabled).
  double controller_patience{-1.0};
  double recovery_patience{0.0};
  // Robot-pose loss grace window (navigation.velocity_timeout, seconds):
  // injected into the controller execution (setPoseGrace).
  double velocity_timeout{0.25};

 public:
  /// @brief Cached patience values (config readback for tests/observability).
  double controllerPatience() const { return controller_patience; }
  double recoveryPatience() const { return recovery_patience; }
  // Session/version single source of truth: built-in by default;
  // BehaviorTreePilot injects the shared instance on setNavigationRuntime
  // (spec single-source discipline, removing divergent pilot-only counts).
  std::shared_ptr<NavigationSessionState> session_state{
      std::make_shared<NavigationSessionState>()};

 private:
  std::atomic<bool> running{false};
  std::atomic<bool> recovery_exclusive{false};
  mutable std::mutex planner_request_mutex;
  mutable std::mutex smoother_request_mutex;
  mutable std::mutex controller_request_mutex;
  mutable std::atomic<uint64_t> next_rejected_id{1};

  /// @brief Build an immediately-failed TaskHandle (invalid module id
  /// etc.): valid==true, ready==true, status=ABORTED, result.failure
  /// structured failure code.
  template <typename Result>
  TaskHandle<Result> rejectedHandle(ErrorCode code,
                                    const std::string& message,
                                    uint64_t session_id,
                                    uint64_t plan_version) const;
};

}  // namespace components
}  // namespace ayanami
