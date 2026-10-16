#pragma once

#include "ayanami/abstract/components/controller_execution.hpp"
#include "ayanami/abstract/components/navigation_session_state.hpp"
#include "ayanami/abstract/components/planner_execution.hpp"
#include "ayanami/abstract/components/module_manager.hpp"
#include "ayanami/abstract/components/recovery_execution.hpp"
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

/**
 * @brief Navigation server base class (no map dependency).
 *
 * Owns the module managers (planner / smoother / controller / recovery +
 * the two checker extension families), the shared navigation session, and
 * the server-level execution assembly caches (velocity limits, controller
 * frequency, patience, pose-loss grace).
 *
 * Map objects (costmap / esdfmap) are composed by the FSM hosts (e.g.
 * RoseNavigationFsm) which hold them as plain members; derived hosts
 * override the virtual initialize*Module hooks to inject family-typed map
 * handles into their modules, and createActions() is the loadModules tail
 * hook where a host assembles its family executions + the FsmDriver (the
 * Action layer is gone -- executions are driven inline by the FSM).
 */
class NavigationServerBase {
public:

  virtual ~NavigationServerBase();

  /// @brief Read the config.json global section -> dlopen each of the 6
  /// sub-configs + call the virtual initialize*Module hooks ->
  /// createActions (the host's assembly hook).
  void loadModules(const std::string& config_path);

  /// @brief Assembly hook (loadModules tail call): the FSM hosts build their
  /// family executions + FsmDriver here. The base implementation is a no-op.
  virtual void createActions();

  /// @brief Start the server (idempotent): activate every loaded module.
  /// Derived hosts override to also start their own resident machinery
  /// (map writers, the FsmDriver).
  virtual void start();

  /// @brief Stop the server (idempotent): deactivate every loaded module.
  virtual void stop();

  /// @brief Module family tags (observability / host queries).
  enum class ModuleFamily { Planner, Smoother, Controller, Recovery };

  /// @brief Loaded module names per family.
  std::vector<std::string> moduleNames(ModuleFamily family) const;

  // ---- Session/version (single source of truth = shared NavigationSessionState) ----
  uint64_t currentSessionId() const;
  uint64_t currentPlanVersion() const;
  void incrementPlanVersion();
  bool sessionIsCurrent(uint64_t session_id) const;
  bool planVersionIsCurrent(uint64_t plan_version) const;
  /// @brief Start a new session (session+1 and plan_version reset to zero).
  void startNewSession();

  std::shared_ptr<NavigationSessionState> sessionState() const;

protected:
  // Key virtual hooks: derived hosts override these to inject map handles.
  // module_config is the module's own module json (already parsed); derived
  // hosts use it to build the ModuleContext passed to the family-level
  // configure.
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

  /// @brief Apply server-level assembly to a controller execution (goal/
  /// progress checker injection + velocity limits + cadence + patience).
  virtual void configureControllerExecution(
      std::shared_ptr<ControllerExecution> exec) const;

  /// @brief Apply server-level assembly to a recovery execution (velocity
  /// limits + patience).
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
  std::string default_planner;
  std::string default_smoother;
  std::string default_controller;
  std::vector<std::string> recovery_sequence;
  std::string default_goal_checker;
  std::string default_progress_checker;
  // Velocity limit cache (navigation.max_vel_x/max_vel_y/max_vel_theta,
  // parameters non-negative; max_vel_y=0 forbids lateral motion).
  double max_vel_x{std::numeric_limits<double>::infinity()};
  double max_vel_y{std::numeric_limits<double>::infinity()};
  double max_vel_theta{std::numeric_limits<double>::infinity()};
  // Controller loop rate (navigation.controller_frequency, Hz).
  double controller_frequency{20.0};
  // Patience (navigation.{controller_patience,recovery_patience}, seconds):
  // controller = nav2 failure_tolerance analogue (<0 = tolerate forever
  // [default], 0 = fail immediately, >0 = tolerance window; covers both "no
  // valid command" and "stale map" -> MAP_STALE terminal), recovery = per-run
  // timeout (<=0 = disabled).
  double controller_patience{-1.0};
  double recovery_patience{0.0};
  // Robot-pose loss grace window (navigation.velocity_timeout, seconds).
  double velocity_timeout{0.25};
  /// @brief Tracking-deviation guard threshold (navigation.
  /// deviation_threshold, meters; 0 = disabled), injected into the
  /// controller execution (setDeviationThreshold).
  double deviation_threshold{0.0};

public:
  /// @brief Cached patience values (config readback for tests/observability).
  double controllerPatience() const { return controller_patience; }
  double recoveryPatience() const { return recovery_patience; }
  // Session/version single source of truth: hosts may inject one shared
  // instance; the built-in one is the default.
  std::shared_ptr<NavigationSessionState> session_state{
      std::make_shared<NavigationSessionState>()};

private:
  std::atomic<bool> running{false};
};

}  // namespace components
}  // namespace ayanami
