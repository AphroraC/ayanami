// NavigationServerBase main implementation.
//
// Lifecycle discipline: the destructor first stops all actions (cancel +
// join slot threads), then resets the action maps, then clearModules
// releases the module objects (destructed while the dlopen handles are
// still alive).
// Zero ROS includes (red line §5.7).
//
// Dispatch discipline: planner/smoother/controller support multiple loaded
// module configs (in the main config.json's global section the config_*
// string-or-list is already array-capable),
// Planner, smoother and controller each use one configured module/action.
// Recovery remains an ordered set of independently addressable actions.

#include "ayanami/abstract/components/navigation_server_base.hpp"
#include <ayanami/utility/bootstrap.hpp>

#include "ayanami/utility/config.hpp"

#include <spdlog/spdlog.h>

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

template <typename ActionMap>
std::string loadedNames(const ActionMap& actions) {
  std::string out;
  for (const auto& kv : actions) {
    if (!out.empty()) {
      out += ", ";
    }
    out += kv.first;
  }
  return out;
}

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

}  // namespace

NavigationServerBase::~NavigationServerBase() {
  stop();
  planner_actions.clear();
  smoother_actions.clear();
  controller_actions.clear();
  recovery_actions.clear();
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

namespace {

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
  // cached as members and injected by the createActions controller execution
  // factory via ControllerExecution::setMaximumVelocity. Parameters are
  // non-negative (max_vel_y=0 forbids lateral motion).
  max_vel_x = nav_cfg.param<double>("navigation", "max_vel_x", max_vel_x);
  max_vel_y = nav_cfg.param<double>("navigation", "max_vel_y", max_vel_y);
  max_vel_theta =
      nav_cfg.param<double>("navigation", "max_vel_theta", max_vel_theta);
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
    if (!name.empty()) recovery_sequence.push_back(name);
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
  auto collect = [&names](const auto& action_map) {
    names.reserve(action_map.size());
    for (const auto& entry : action_map) {
      names.push_back(entry.first);
    }
  };
  switch (family) {
    case ModuleFamily::Planner:
      collect(planner_actions);
      break;
    case ModuleFamily::Smoother:
      collect(smoother_actions);
      break;
    case ModuleFamily::Controller:
      collect(controller_actions);
      break;
    case ModuleFamily::Recovery:
      collect(recovery_actions);
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
  for (auto& kv : planner_actions) {
    if (kv.second) kv.second->cancelAll();
  }
  for (auto& kv : smoother_actions) {
    if (kv.second) kv.second->cancelAll();
  }
  for (auto& kv : controller_actions) {
    if (kv.second) kv.second->cancelAll();
  }
  for (auto& kv : recovery_actions) {
    if (kv.second) kv.second->cancelAll();
  }
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
  planner_actions.clear();
  smoother_actions.clear();
  controller_actions.clear();
  recovery_actions.clear();

  // planner/smoother/controller/recovery: one action per loaded module
  // (nav2 multi-module semantics). default_* is the FIRST configured module;
  // requests select by goal id, empty id = default.
  for (const auto& name : planner_modules.getLoadedNames()) {
    auto planner = planner_modules.getModule(name);
    if (!planner) {
      continue;
    }
    auto action = std::make_unique<PlannerAction>(planner);
    action->setExecutionFactory([this, name]() {
      return std::shared_ptr<PlannerExecution>(
          newPlannerExecution(name, planner_modules.getModule(name)));
    });
    planner_actions[name] = std::move(action);
  }
  if (!planner_actions.empty() &&
      planner_actions.find(default_planner) == planner_actions.end()) {
    spdlog::error("NavigationServerBase::createActions: default planner '{}' "
                  "not loaded",
                  default_planner);
  }

  for (const auto& name : smoother_modules.getLoadedNames()) {
    auto smoother = smoother_modules.getModule(name);
    if (!smoother) {
      continue;
    }
    auto action = std::make_unique<SmootherAction>(smoother);
    action->setExecutionFactory([this, name]() {
      return std::shared_ptr<SmootherExecution>(
          newSmootherExecution(name, smoother_modules.getModule(name)));
    });
    smoother_actions[name] = std::move(action);
  }
  if (!smoother_actions.empty() &&
      smoother_actions.find(default_smoother) == smoother_actions.end()) {
    spdlog::error("NavigationServerBase::createActions: default smoother '{}' "
                  "not loaded",
                  default_smoother);
  }

  for (const auto& name : controller_modules.getLoadedNames()) {
    auto controller = controller_modules.getModule(name);
    if (!controller) {
      continue;
    }
    auto action = std::make_unique<ControllerAction>(controller);
    action->setExecutionFactory([this, name]() {
      auto exec = std::shared_ptr<ControllerExecution>(
          newControllerExecution(name, controller_modules.getModule(name)));
      configureControllerExecution(exec);
      return exec;
    });
    controller_actions[name] = std::move(action);
  }
  if (!controller_actions.empty() &&
      controller_actions.find(default_controller) == controller_actions.end()) {
    spdlog::error("NavigationServerBase::createActions: default controller "
                  "'{}' not loaded",
                  default_controller);
  }

  for (const auto& name : recovery_modules.getLoadedNames()) {
    auto recovery = recovery_modules.getModule(name);
    if (!recovery) {
      continue;
    }
    auto action = std::make_unique<RecoveryAction>(recovery);
    action->setExecutionFactory([this, name]() {
      auto exec = std::shared_ptr<RecoveryExecution>(newRecoveryExecution(
          name, recovery_modules.getModule(name)));
      configureRecoveryExecution(exec);
      return exec;
    });
    recovery_actions[name] = std::move(action);
  }
  if (recovery_actions.empty()) {
    spdlog::warn("NavigationServerBase::createActions: no recovery modules loaded");
  }
}

void NavigationServerBase::configureControllerExecution(
    std::shared_ptr<ControllerExecution> exec) const {
  // Goal/progress checker module injection (consumed by the execution at runtime).
  if (auto checker = goal_checker_modules.getModule(default_goal_checker)) {
    exec->setGoalChecker(std::move(checker));
  } else if (!default_goal_checker.empty()) {
    spdlog::warn("NavigationServerBase: goal checker '{}' not loaded",
                 default_goal_checker);
  }
  if (auto checker =
          progress_checker_modules.getModule(default_progress_checker)) {
    exec->setProgressChecker(std::move(checker));
  } else if (!default_progress_checker.empty()) {
    spdlog::warn("NavigationServerBase: progress checker '{}' not loaded",
                 default_progress_checker);
  }
  // Velocity limit injection (navigation.max_vel_{x,y,theta} cache;
  // validVelocityCommand rejects over-limit commands at the execution layer,
  // and max_vel_y=0 also forbids lateral motion).
  exec->setMaximumVelocity(max_vel_x, max_vel_y, max_vel_theta);
  // Failure patience (navigation.controller_patience, nav2
  // failure_tolerance analogue): only after no valid command persists for
  // this long does the execution report a terminal state; retry decisions
  // still belong to the BT.
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

namespace {

/// @brief Family map lookup: explicit id -> that action (nullptr when the id
/// is unknown); empty id -> the family default.
template <typename Action>
Action* selectAction(
    const std::map<std::string, std::unique_ptr<Action>>& actions,
    const std::string& default_id, const std::string& requested) {
  if (!requested.empty()) {
    auto iter = actions.find(requested);
    return iter == actions.end() ? nullptr : iter->second.get();
  }
  auto iter = actions.find(default_id);
  if (iter != actions.end()) {
    return iter->second.get();
  }
  return actions.empty() ? nullptr : actions.begin()->second.get();
}

}  // namespace

PlannerAction* NavigationServerBase::plannerAction(
    const std::string& id) const {
  return selectAction(planner_actions, default_planner, id);
}

SmootherAction* NavigationServerBase::smootherAction(
    const std::string& id) const {
  return selectAction(smoother_actions, default_smoother, id);
}

ControllerAction* NavigationServerBase::controllerAction(
    const std::string& id) const {
  return selectAction(controller_actions, default_controller, id);
}

RecoveryAction* NavigationServerBase::recoveryAction(
    const std::string& name) const {
  auto iter = recovery_actions.find(name);
  return iter == recovery_actions.end() ? nullptr : iter->second.get();
}

std::vector<std::string> NavigationServerBase::recoveryActionNames() const {
  std::vector<std::string> names;
  names.reserve(recovery_actions.size());
  for (const auto& kv : recovery_actions) {
    names.push_back(kv.first);
  }
  return names;
}

TaskHandle<ayanami::actions::GetPathResult> NavigationServerBase::requestPath(
    const ayanami::actions::GetPathGoal& goal) {
  std::lock_guard<std::mutex> lock(planner_request_mutex);
  PlannerAction* action =
      selectAction(planner_actions, default_planner, goal.planner_id);
  if (recovery_exclusive.load()) {
    return rejectedHandle<ayanami::actions::GetPathResult>(
        ErrorCode::RESOURCE_BUSY, "planner unavailable", currentSessionId(),
        currentPlanVersion());
  }
  if (!action) {
    // Explicit unknown id -> structured rejection (nav2/MBF semantics);
    // an empty-id miss means the family has no module at all.
    return rejectedHandle<ayanami::actions::GetPathResult>(
        goal.planner_id.empty() ? ErrorCode::RESOURCE_BUSY
                                : ErrorCode::INVALID_INPUT,
        goal.planner_id.empty()
            ? "planner unavailable"
            : "unknown planner id: " + goal.planner_id,
        currentSessionId(), currentPlanVersion());
  }
  auto g = goal;
  g.session_id = currentSessionId();
  g.plan_version = currentPlanVersion();
  return action->acceptGoal(g);
}

TaskHandle<ayanami::actions::SmoothPathResult>
NavigationServerBase::requestSmoothing(
    const ayanami::actions::SmoothPathGoal& goal) {
  std::lock_guard<std::mutex> lock(smoother_request_mutex);
  SmootherAction* action =
      selectAction(smoother_actions, default_smoother, goal.smoother_id);
  if (recovery_exclusive.load()) {
    return rejectedHandle<ayanami::actions::SmoothPathResult>(
        ErrorCode::RESOURCE_BUSY, "smoother unavailable", currentSessionId(),
        currentPlanVersion());
  }
  if (!action) {
    return rejectedHandle<ayanami::actions::SmoothPathResult>(
        goal.smoother_id.empty() ? ErrorCode::RESOURCE_BUSY
                                 : ErrorCode::INVALID_INPUT,
        goal.smoother_id.empty()
            ? "smoother unavailable"
            : "unknown smoother id: " + goal.smoother_id,
        currentSessionId(), currentPlanVersion());
  }
  auto g = goal;
  g.session_id = currentSessionId();
  g.plan_version = currentPlanVersion();
  return action->acceptGoal(g);
}

TaskHandle<ayanami::actions::ExePathResult>
NavigationServerBase::requestExecution(
    const ayanami::actions::ExePathGoal& goal) {
  std::lock_guard<std::mutex> lock(controller_request_mutex);
  ControllerAction* action =
      selectAction(controller_actions, default_controller, goal.controller_id);
  if (recovery_exclusive.load()) {
    return rejectedHandle<ayanami::actions::ExePathResult>(
        ErrorCode::RESOURCE_BUSY, "controller unavailable", currentSessionId(),
        currentPlanVersion());
  }
  if (!action) {
    return rejectedHandle<ayanami::actions::ExePathResult>(
        goal.controller_id.empty() ? ErrorCode::RESOURCE_BUSY
                                   : ErrorCode::INVALID_INPUT,
        goal.controller_id.empty()
            ? "controller unavailable"
            : "unknown controller id: " + goal.controller_id,
        currentSessionId(), currentPlanVersion());
  }
  auto g = goal;
  g.session_id = currentSessionId();
  g.plan_version = currentPlanVersion();
  return action->acceptGoal(g);
}

TaskHandle<ayanami::actions::RecoveryResult>
NavigationServerBase::requestRecovery(
    const ayanami::actions::RecoveryGoal& goal) {
  bool expected = false;
  if (!recovery_exclusive.compare_exchange_strong(expected, true)) {
    return rejectedHandle<ayanami::actions::RecoveryResult>(
        ErrorCode::RESOURCE_BUSY, "another recovery is active",
        currentSessionId(), currentPlanVersion());
  }
  auto iter = recovery_actions.find(goal.recovery_id);
  if (iter == recovery_actions.end() || !iter->second) {
    recovery_exclusive.store(false);
    spdlog::error("NavigationServerBase::requestRecovery: no recovery action "
                  "for '{}'",
                  goal.recovery_id);
    return rejectedHandle<ayanami::actions::RecoveryResult>(
        ErrorCode::INVALID_INPUT,
        "unknown recovery module id: " + goal.recovery_id, currentSessionId(),
        currentPlanVersion());
  }
  // Stop the controller after validating the recovery id. Non-blocking:
  // the preemption marks the controller canceled (cancel() already
  // published a zeroed command synchronously) and the task winds down on
  // its own thread, so the BT tick thread never blocks on a slow module
  // call.
  preemptAllControllers();
  ayanami::actions::RecoveryGoal g = goal;
  g.session_id = currentSessionId();
  g.plan_version = currentPlanVersion();
  try {
    auto task = iter->second->acceptGoal(g);
    // Resource protocol: the controller may resume accepting requests only
    // after the recovery finishes (including the exception-fallback path's
    // publishResult).
    task.onComplete([this] { recovery_exclusive.store(false); });
    return task;
  } catch (...) {
    // acceptGoal exception: no TaskHandle exists to attach the release callback, so return the mutex immediately and rethrow.
    recovery_exclusive.store(false);
    throw;
  }
}

bool NavigationServerBase::requestPlanUpdate(
    uint64_t goal_id, const ayanami::actions::ExePathGoal& goal) {
  // The in-place update addresses the goal by id: search every loaded
  // controller action (the goal may run on a non-default module).
  std::lock_guard<std::mutex> request_lock(controller_request_mutex);
  for (auto& kv : controller_actions) {
    if (kv.second && kv.second->updatePlan(goal_id, goal.path,
                                           goal.plan_version)) {
      return true;
    }
  }
  spdlog::error("NavigationServerBase::requestPlanUpdate: no running "
                "controller goal {} to update",
                goal_id);
  return false;
}

void NavigationServerBase::cancelAllControllers() {
  std::lock_guard<std::mutex> request_lock(controller_request_mutex);
  // Preemption/new-session/halt discipline: cancel the ENTIRE controller
  // action map -- with multiple controller modules, tasks beyond the default
  // id must stop too, otherwise stale velocity output remains.
  for (auto& kv : controller_actions) {
    if (kv.second) kv.second->cancelAll();
  }
}

void NavigationServerBase::preemptAllControllers() {
  std::lock_guard<std::mutex> request_lock(controller_request_mutex);
  for (auto& kv : controller_actions) {
    if (kv.second) kv.second->preemptAll();
  }
}

void NavigationServerBase::cancelAllPlanners() {
  std::lock_guard<std::mutex> request_lock(planner_request_mutex);
  for (auto& kv : planner_actions) {
    if (kv.second) kv.second->cancelAll();
  }
}

void NavigationServerBase::cancelAllSmoothers() {
  std::lock_guard<std::mutex> request_lock(smoother_request_mutex);
  for (auto& kv : smoother_actions) {
    if (kv.second) kv.second->cancelAll();
  }
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

void NavigationServerBase::setSessionState(
    std::shared_ptr<NavigationSessionState> session) {
  if (!session) {
    return;
  }
  session_state = std::move(session);
}

std::shared_ptr<NavigationSessionState> NavigationServerBase::sessionState()
    const {
  return session_state;
}

// ---- Semantics: a structured-failure TaskHandle for an invalid module id ----

template <typename Result>
TaskHandle<Result> NavigationServerBase::rejectedHandle(
    ErrorCode code, const std::string& message, uint64_t session_id,
    uint64_t plan_version) const {
  using Completion = ayanami::actions::GoalResult<Result>;
  auto state = std::make_shared<TaskState<Result>>();
  Completion completion;
  completion.status = ayanami::actions::GoalStatus::ABORTED;
  completion.result.error = code;
  completion.result.message = message;
  completion.result.session_id = session_id;
  completion.result.plan_version = plan_version;
  state->complete(std::move(completion));
  return TaskHandle<Result>(next_rejected_id.fetch_add(1), std::move(state),
                            []() { return false; });
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

std::unique_ptr<SmootherExecution> NavigationServerBase::newSmootherExecution(
    const std::string& name, AbstractSmoother::Ptr smoother) {
  (void)name;
  return std::make_unique<SmootherExecution>(std::move(smoother));
}

std::unique_ptr<PlannerExecution> NavigationServerBase::newPlannerExecution(
    const std::string& name, AbstractPlanner::Ptr planner) {
  (void)name;
  return std::make_unique<PlannerExecution>(std::move(planner));
}

std::unique_ptr<ControllerExecution>
NavigationServerBase::newControllerExecution(const std::string& name,
                                             AbstractController::Ptr controller) {
  (void)name;
  return std::make_unique<ControllerExecution>(std::move(controller));
}

std::unique_ptr<RecoveryExecution> NavigationServerBase::newRecoveryExecution(
    const std::string& name, AbstractRecovery::Ptr recovery) {
  (void)name;
  return std::make_unique<RecoveryExecution>(std::move(recovery));
}

}  // namespace components
}  // namespace ayanami
