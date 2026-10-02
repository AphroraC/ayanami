// NavigationServerBase main implementation (FSM era).
//
// Lifecycle discipline: the destructor runs stop() (module deactivation),
// then module cleanup() hooks, then clearModules releases the module
// objects (destructed while the dlopen handles are still alive).
// Zero ROS includes (red line §5.7).
//
// The Action layer is gone: executions are assembled by the FSM hosts in
// createActions() (the loadModules tail hook) and driven inline by the
// FsmDriver.

#include "ayanami/abstract/components/navigation_server_base.hpp"
#include <ayanami/utility/bootstrap.hpp>

#include "ayanami/utility/config.hpp"

#include <spdlog/spdlog.h>
#include <ayanami/utility/logging.hpp>

#include <filesystem>
#include <utility>

namespace ayanami {
namespace components {

using ayanami::abstract::AbstractController;
using ayanami::abstract::AbstractGoalChecker;
using ayanami::abstract::AbstractPlanner;
using ayanami::abstract::AbstractProgressChecker;
using ayanami::abstract::AbstractRecovery;
using ayanami::abstract::AbstractSmoother;

namespace {

// Module file family enumeration: in config_navigation's navigation
// section, navigation[family_key] is a single string or a string array (an
// array means a multi-module file family; currently only config_recovery
// uses it). Returns full paths resolved relative to config_path (absolute
// paths kept as-is).
std::vector<std::string> moduleConfigFiles(const ayanami::Config& navigation,
                                           const std::string& family_key) {
  const ayanami::GlobalConfig* g = ayanami::GlobalConfig::instance();
  // One helper for the "single string or array of strings" spelling, which used
  // to be hand-written (try vector, fall back to string) at every call site.
  const std::vector<std::string> entries =
      navigation.paramStringOrList("navigation", family_key);
  const std::string directory =
      g->param<std::string>("global", "config_path", ".");
  std::vector<std::string> paths;
  paths.reserve(entries.size());
  for (const auto& entry : entries) {
    const std::filesystem::path p(entry);
    paths.push_back(p.is_absolute() ? entry : (directory + "/" + entry));
  }
  return paths;
}

/// @brief Fail-fast for a module the config explicitly names.
///
/// A navigation stack that silently starts without its configured planner or
/// controller is worse than one that refuses to start: the omission only
/// surfaces later, as an unexplained INVALID_INPUT when a goal finally arrives.
/// The manager records WHY it failed, so the message distinguishes "the .so is
/// not deployed" from "the .so was built against a different interface".
template <typename ManagerT>
void requireLoaded(const ManagerT& manager, const std::string& family,
                   const std::string& name) {
  if (manager.getModule(name)) {
    return;
  }
  const std::string detail = manager.lastFailureMessage();
  spdlog::critical("{} module '{}' failed to load: {}", family, name, detail);
  throw std::runtime_error(family + " module failed to load: " + detail);
}

}  // namespace

NavigationServerBase::~NavigationServerBase() {
  stop();
  for (const auto& name : planner_modules.getLoadedNames()) {
    if (auto module = planner_modules.getModule(name)) module->cleanup();
  }
  for (const auto& name : smoother_modules.getLoadedNames()) {
    if (auto module = smoother_modules.getModule(name)) module->cleanup();
  }
  for (const auto& name : controller_modules.getLoadedNames()) {
    if (auto module = controller_modules.getModule(name)) module->cleanup();
  }
  for (const auto& name : recovery_modules.getLoadedNames()) {
    if (auto module = recovery_modules.getModule(name)) module->cleanup();
  }
  planner_modules.clearModules();
  smoother_modules.clearModules();
  controller_modules.clearModules();
  recovery_modules.clearModules();
  goal_checker_modules.clearModules();
  progress_checker_modules.clearModules();
}

void NavigationServerBase::loadModules(const std::string& config_path) {
  ayanami::bootstrap(config_path);

  // Module id single source of truth = each module json's own name field:
  // the first three families each load exactly one module, with no separate
  // *_id config keys; recovery is an ordered file family (config_recovery
  // vector).
  ayanami::Config nav_cfg(
      ayanami::GlobalConfig::getConfigPath("config_navigation"));
  default_planner.clear();
  default_smoother.clear();
  default_controller.clear();
  recovery_sequence.clear();
  default_goal_checker.clear();
  default_progress_checker.clear();

  // Velocity limit single source of truth =
  // navigation.{max_vel_x,max_vel_y,max_vel_theta}:
  // cached as members and injected by the host's execution assembly via
  // configureControllerExecution. Parameters are non-negative
  // (max_vel_y=0 forbids lateral motion).
  max_vel_x = nav_cfg.param<double>("navigation", "max_vel_x", max_vel_x);
  max_vel_y = nav_cfg.param<double>("navigation", "max_vel_y", max_vel_y);
  max_vel_theta =
      nav_cfg.param<double>("navigation", "max_vel_theta", max_vel_theta);
  // Controller loop rate: the execution layer owns the computeVelocityCommand
  // cadence, so the config value must land here (the server), not only on
  // the host -- the previous gap left every controller execution running on
  // its built-in 20 Hz default regardless of config.
  controller_frequency =
      nav_cfg.param<double>("navigation", "controller_frequency",
                            controller_frequency);
  // Patience: controller = nav2 failure_tolerance semantics (<0 forever,
  // 0 immediate, >0 window; also covers the stale-map safety check);
  // recovery patience <=0 disables.
  controller_patience =
      nav_cfg.param<double>("navigation", "controller_patience", -1.0);
  recovery_patience =
      nav_cfg.param<double>("navigation", "recovery_patience", 0.0);
  // Pose-loss grace window for the controller loop (same source as the
  // RobotStateController velocity_timeout).
  velocity_timeout =
      nav_cfg.param<double>("navigation", "velocity_timeout", 0.25);

  // planner/smoother/controller each have exactly one configured module.
  for (const auto& path : moduleConfigFiles(nav_cfg, "config_planner")) {
    ayanami::Config cfg(path);
    const std::string name = cfg.param<std::string>("planner", "name", "");
    if (name.empty()) {
      continue;
    }
    if (default_planner.empty()) {
      default_planner = name;
    }
    planner_modules.loadSingleFromConfig(
        cfg, "planner", name,
        [this, cfg](const std::string& loaded_name,
                    AbstractPlanner::Ptr module) {
          return initializePlannerModule(loaded_name, cfg, std::move(module));
        });
    requireLoaded(planner_modules, "planner", name);
  }
  // smoother
  for (const auto& path : moduleConfigFiles(nav_cfg, "config_smoother")) {
    ayanami::Config cfg(path);
    const std::string name = cfg.param<std::string>("smoother", "name", "");
    if (name.empty()) {
      continue;
    }
    if (default_smoother.empty()) {
      default_smoother = name;
    }
    smoother_modules.loadSingleFromConfig(
        cfg, "smoother", name,
        [this, cfg](const std::string& loaded_name,
                    AbstractSmoother::Ptr module) {
          return initializeSmootherModule(loaded_name, cfg, std::move(module));
        });
    requireLoaded(smoother_modules, "smoother", name);
  }
  // controller
  for (const auto& path : moduleConfigFiles(nav_cfg, "config_controller")) {
    ayanami::Config cfg(path);
    const std::string name = cfg.param<std::string>("controller", "name", "");
    if (name.empty()) {
      continue;
    }
    if (default_controller.empty()) {
      default_controller = name;
    }
    controller_modules.loadSingleFromConfig(
        cfg, "controller", name,
        [this, cfg](const std::string& loaded_name,
                    AbstractController::Ptr module) {
          return initializeControllerModule(loaded_name, cfg,
                                           std::move(module));
        });
    requireLoaded(controller_modules, "controller", name);
  }
  // recovery (ordered config_recovery file family; the order is the
  // recovery rotation declaration order)
  for (const auto& path : moduleConfigFiles(nav_cfg, "config_recovery")) {
    ayanami::Config cfg(path);
    const std::string name = cfg.param<std::string>("recovery", "name", "");
    if (!name.empty()) {
      recovery_sequence.push_back(name);
    }
    recovery_modules.loadSingleFromConfig(
        cfg, "recovery", name,
        [this, cfg](const std::string& loaded_name,
                    AbstractRecovery::Ptr module) {
          return initializeRecoveryModule(loaded_name, cfg, std::move(module));
        });
    requireLoaded(recovery_modules, "recovery", name);
  }
  // goal checker (single module; params injected via ModuleContext)
  for (const auto& path : moduleConfigFiles(nav_cfg, "config_goal_checker")) {
    ayanami::Config cfg(path);
    const std::string name =
        cfg.param<std::string>("goal_checker", "name", "");
    if (name.empty()) {
      continue;
    }
    if (default_goal_checker.empty()) {
      default_goal_checker = name;
    }
    goal_checker_modules.loadSingleFromConfig(
        cfg, "goal_checker", name,
        [this, cfg](const std::string& loaded_name,
                    AbstractGoalChecker::Ptr module) {
          return initializeGoalCheckerModule(loaded_name, cfg,
                                            std::move(module));
        });
    requireLoaded(goal_checker_modules, "goal_checker", name);
  }
  // progress checker (single module; params injected via ModuleContext)
  for (const auto& path : moduleConfigFiles(nav_cfg, "config_progress_checker")) {
    ayanami::Config cfg(path);
    const std::string name =
        cfg.param<std::string>("progress_checker", "name", "");
    if (name.empty()) {
      continue;
    }
    if (default_progress_checker.empty()) {
      default_progress_checker = name;
    }
    progress_checker_modules.loadSingleFromConfig(
        cfg, "progress_checker", name,
        [this, cfg](const std::string& loaded_name,
                    AbstractProgressChecker::Ptr module) {
          return initializeProgressCheckerModule(loaded_name, cfg,
                                                std::move(module));
        });
    requireLoaded(progress_checker_modules, "progress_checker", name);
  }

  createActions();
}

std::vector<std::string> NavigationServerBase::moduleNames(
    ModuleFamily family) const {
  std::vector<std::string> names;
  switch (family) {
    case ModuleFamily::Planner:
      names = planner_modules.getLoadedNames();
      break;
    case ModuleFamily::Smoother:
      names = smoother_modules.getLoadedNames();
      break;
    case ModuleFamily::Controller:
      names = controller_modules.getLoadedNames();
      break;
    case ModuleFamily::Recovery:
      names = recovery_modules.getLoadedNames();
      break;
  }
  return names;
}

void NavigationServerBase::start() {
  if (running.exchange(true)) return;
  for (const auto& name : planner_modules.getLoadedNames()) {
    if (auto module = planner_modules.getModule(name)) module->activate();
  }
  for (const auto& name : smoother_modules.getLoadedNames()) {
    if (auto module = smoother_modules.getModule(name)) module->activate();
  }
  for (const auto& name : controller_modules.getLoadedNames()) {
    if (auto module = controller_modules.getModule(name)) module->activate();
  }
  for (const auto& name : recovery_modules.getLoadedNames()) {
    if (auto module = recovery_modules.getModule(name)) module->activate();
  }
}

void NavigationServerBase::stop() {
  if (!running.exchange(false)) return;
  for (const auto& name : recovery_modules.getLoadedNames()) {
    if (auto module = recovery_modules.getModule(name)) module->deactivate();
  }
  for (const auto& name : controller_modules.getLoadedNames()) {
    if (auto module = controller_modules.getModule(name)) module->deactivate();
  }
  for (const auto& name : smoother_modules.getLoadedNames()) {
    if (auto module = smoother_modules.getModule(name)) module->deactivate();
  }
  for (const auto& name : planner_modules.getLoadedNames()) {
    if (auto module = planner_modules.getModule(name)) module->deactivate();
  }
}

void NavigationServerBase::createActions() {
  // Assembly hook: the FSM hosts build their family executions + the
  // FsmDriver here. The map-free base has nothing to assemble.
}

void NavigationServerBase::configureControllerExecution(
    std::shared_ptr<ControllerExecution> exec) const {
  // Goal/progress checker module injection (consumed by the execution at runtime).
  if (auto checker = goal_checker_modules.getModule(default_goal_checker)) {
    exec->setGoalChecker(std::move(checker));
  } else if (!default_goal_checker.empty()) {
    SPDLOG_LOGGER_WARN(
      ayanami::controllerLogger(),
      "NavigationServerBase: goal checker '{}' not loaded",
      default_goal_checker);
  }
  if (auto checker = progress_checker_modules.getModule(default_progress_checker)) {
    exec->setProgressChecker(std::move(checker));
  } else if (!default_progress_checker.empty()) {
    SPDLOG_LOGGER_WARN(
      ayanami::controllerLogger(),
      "NavigationServerBase: progress checker '{}' not loaded",
      default_progress_checker);
  }
  // Velocity limit injection (navigation.max_vel_{x,y,theta} cache;
  // over-limit but finite commands are clamped by the execution, and
  // max_vel_y=0 also forbids lateral motion).
  exec->setMaximumVelocity(max_vel_x, max_vel_y, max_vel_theta);
  // Controller loop rate (navigation.controller_frequency): the execution
  // owns the module call cadence.
  exec->setControllerFrequency(controller_frequency);
  // Failure patience (navigation.controller_patience, nav2
  // failure_tolerance analogue): only after no valid command persists for
  // this long does the execution report a terminal state; retry decisions
  // still belong to the FSM.
  exec->setPatience(controller_patience);
  // Transient pose-loss grace window (navigation.velocity_timeout).
  exec->setPoseGrace(velocity_timeout);
}

void NavigationServerBase::configureRecoveryExecution(
    std::shared_ptr<RecoveryExecution> exec) const {
  // Velocity limit injection (same source as the controller:
  // navigation.max_vel_*);
  // RecoveryExecution validates finite + max_vel before its sole publish and
  // zeroes invalid commands.
  exec->setMaximumVelocity(max_vel_x, max_vel_y, max_vel_theta);
  // Recovery patience (navigation.recovery_patience): framework-level
  // per-run timeout so a recovery module without self-limiting cannot stay
  // RUNNING forever.
  exec->setRecoveryPatience(recovery_patience);
}

uint64_t NavigationServerBase::currentSessionId() const {
  return session_state->currentSessionId();
}

uint64_t NavigationServerBase::currentPlanVersion() const {
  return session_state->currentPlanVersion();
}

void NavigationServerBase::incrementPlanVersion() {
  session_state->incrementPlanVersion();
}

bool NavigationServerBase::sessionIsCurrent(uint64_t session_id) const {
  return session_state->sessionIsCurrent(session_id);
}

bool NavigationServerBase::planVersionIsCurrent(uint64_t plan_version) const {
  return session_state->planVersionIsCurrent(plan_version);
}

void NavigationServerBase::startNewSession() {
  session_state->startNewSession();
}

std::shared_ptr<NavigationSessionState> NavigationServerBase::sessionState()
    const {
  return session_state;
}

// ---- Virtual hook default implementations ----

bool NavigationServerBase::initializePlannerModule(
    const std::string& name, const ayanami::Config&, AbstractPlanner::Ptr) {
  (void)name;
  return true;
}

bool NavigationServerBase::initializeSmootherModule(
    const std::string& name, const ayanami::Config&, AbstractSmoother::Ptr) {
  (void)name;
  return true;
}

bool NavigationServerBase::initializeControllerModule(
    const std::string& name, const ayanami::Config&, AbstractController::Ptr) {
  (void)name;
  return true;
}

bool NavigationServerBase::initializeRecoveryModule(
    const std::string& name, const ayanami::Config&, AbstractRecovery::Ptr) {
  (void)name;
  return true;
}

bool NavigationServerBase::initializeGoalCheckerModule(
    const std::string& name, const ayanami::Config& module_config,
    AbstractGoalChecker::Ptr ptr) {
  // checkers have no map dependency: the default hook injects the ModuleContext directly (derived servers may override).
  if (ptr) {
    ptr->configure(ayanami::ModuleContext{name, module_config});
  }
  return true;
}

bool NavigationServerBase::initializeProgressCheckerModule(
    const std::string& name, const ayanami::Config& module_config,
    AbstractProgressChecker::Ptr ptr) {
  if (ptr) {
    ptr->configure(ayanami::ModuleContext{name, module_config});
  }
  return true;
}

}  // namespace components
}  // namespace ayanami
