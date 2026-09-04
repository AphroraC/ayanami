// NavigationServerBase 主实现 (spec §5.3)。
//
// 生命周期纪律 (spec §4.5): 析构先 stop() 停全部 action (取消 + join 槽线程),
// 再 reset 各 action map, 再 clearPlugins 释放插件对象 (dlopen 句柄仍存活时析构)。
// 零 ROS include (红线 §5.7)。
//
// 分派纪律: planner/smoother/controller 支持 multiple loaded plugin configs
// (main config.json 的 global 段里 config_* string-or-list 已是 array-capable),
// Planner, smoother and controller each use one configured plugin/action.
// Recovery remains an ordered set of independently addressable actions.

#include "ayanami/abstract/components/navigation_server_base.hpp"

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

// 严格单层索引的插件文件族枚举: 主 config.json 的 global 段里
// global[family_key] 为单字符串或字符串数组 (数组即多插件文件族)。
// 返回相对 config_path 解析后的完整路径 (绝对路径原样保留)。
std::vector<std::string> moduleConfigFiles(const ayanami::Config& navigation,
                                           const std::string& family_key) {
  const ayanami::GlobalConfig* g = ayanami::GlobalConfig::instance();
  std::vector<std::string> entries;
  if (navigation.hasParam("navigation", family_key)) {
    try {
      entries = navigation.param<std::vector<std::string>>(
          "navigation", family_key, {});
    } catch (const std::exception&) {
      // navigation[family_key] 是单字符串。
    }
    if (entries.empty()) {
      const std::string single =
          navigation.param<std::string>("navigation", family_key, "");
      if (!single.empty()) {
        entries.push_back(single);
      }
    }
  }
  // Keep old externally supplied configurations readable during migration.
  // The installed configuration no longer uses this branch.
  if (entries.empty() && g->hasParam("global", family_key)) {
    try {
      entries = g->param<std::vector<std::string>>("global", family_key, {});
    } catch (const std::exception&) {
      const std::string single = g->param<std::string>("global", family_key, "");
      if (!single.empty()) entries.push_back(single);
    }
  }
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

void registerModuleConfigAliases(const std::string& family_key,
                                 const std::string& path,
                                 const ayanami::Config& cfg) {
  ayanami::GlobalConfig::registerConfigPath(family_key, path);
  // Existing modules use their historical configuration stem internally. Keep
  // that implementation detail in the resolved-path registry, not in the
  // public root configuration schema.
  const std::string stem =
      std::filesystem::path(path).stem().string();
  if (!stem.empty()) {
    ayanami::GlobalConfig::registerConfigPath(stem, path);
  }
  const char* section = nullptr;
  if (family_key == "config_planner") section = "planner";
  if (family_key == "config_smoother") section = "smoother";
  if (family_key == "config_controller") section = "controller";
  if (family_key == "config_recovery") section = "recovery";
  if (family_key == "config_goal_checker") section = "goal_checker";
  if (family_key == "config_progress_checker") section = "progress_checker";
  if (section == nullptr) return;
  const std::string name = cfg.param<std::string>(section, "name", "");
  if (!name.empty()) ayanami::GlobalConfig::registerConfigPath(name, path);

  // Legacy module keys are derived from the individual file name. Do not
  // register one family-wide alias here: recovery contains several files and
  // such an alias would make every recovery read the last file in the list.
  if (stem.rfind("costmap_", 0) == 0) {
    ayanami::GlobalConfig::registerConfigPath("config_" + stem.substr(8), path);
  }
}

}  // namespace

NavigationServerBase::~NavigationServerBase() {
  stop();
  planner_action.reset();
  smoother_action.reset();
  controller_action.reset();
  recovery_actions.clear();
  for (const auto& name : planner_plugins.getLoadedNames()) {
    if (auto plugin = planner_plugins.getPlugin(name)) plugin->cleanup();
  }
  for (const auto& name : smoother_plugins.getLoadedNames()) {
    if (auto plugin = smoother_plugins.getPlugin(name)) plugin->cleanup();
  }
  for (const auto& name : controller_plugins.getLoadedNames()) {
    if (auto plugin = controller_plugins.getPlugin(name)) plugin->cleanup();
  }
  for (const auto& name : recovery_plugins.getLoadedNames()) {
    if (auto plugin = recovery_plugins.getPlugin(name)) plugin->cleanup();
  }
  planner_plugins.clearPlugins();
  smoother_plugins.clearPlugins();
  controller_plugins.clearPlugins();
  recovery_plugins.clearPlugins();
  goal_checker_plugins.clearPlugins();
  progress_checker_plugins.clearPlugins();
}

void NavigationServerBase::loadPlugins(const std::string& config_path) {
  ayanami::GlobalConfig::instance(config_path, /*override_path=*/true);

  // 默认插件 id 显式来自 navigation.{planner_id,smoother_id,controller_id,...}
  // (spec §9.3)。缺省回退到装载列表首插件名 (兼容无 *_id 键的历史配置)。
  ayanami::Config nav_cfg(
      ayanami::GlobalConfig::getConfigPath("config_navigation"));
  default_planner =
      nav_cfg.param<std::string>("navigation", "planner_id", "");
  default_smoother =
      nav_cfg.param<std::string>("navigation", "smoother_id", "");
  default_controller =
      nav_cfg.param<std::string>("navigation", "controller_id", "");
  recovery_sequence.clear();
  default_goal_checker =
      nav_cfg.param<std::string>("navigation", "goal_checker_id", "");
  default_progress_checker =
      nav_cfg.param<std::string>("navigation", "progress_checker_id", "");

  // Resolve every module path before constructing any plugin. Several legacy
  // plugins read their own file from GlobalConfig during construction; the
  // registry must therefore be populated before the first factory call.
  for (const auto& family : {"config_planner", "config_smoother",
                             "config_controller", "config_recovery",
                             "config_goal_checker", "config_progress_checker"}) {
    for (const auto& path : moduleConfigFiles(nav_cfg, family)) {
      registerModuleConfigAliases(family, path, ayanami::Config(path));
    }
  }

  // 速度上限单一来源 = navigation.{max_vel_x,max_vel_y,max_vel_theta}:
  // 缓存为成员, createActions controller execution factory 注入
  // ControllerExecution::setMaximumVelocity。参数非负 (max_vel_y=0 禁止横移)。
  max_vel_x = nav_cfg.param<double>("navigation", "max_vel_x", max_vel_x);
  max_vel_y = nav_cfg.param<double>("navigation", "max_vel_y", max_vel_y);
  max_vel_theta =
      nav_cfg.param<double>("navigation", "max_vel_theta", max_vel_theta);

  // planner/smoother/controller each have exactly one configured plugin.
  for (const auto& path : moduleConfigFiles(nav_cfg, "config_planner")) {
    ayanami::Config cfg(path);
    registerModuleConfigAliases("config_planner", path, cfg);
    const std::string name = cfg.param<std::string>("planner", "name", "");
    if (name.empty()) {
      continue;
    }
    if (default_planner.empty()) {
      default_planner = name;
    }
    planner_plugins.loadSingleFromConfig(
        cfg, "planner", name,
        [this, cfg](const std::string& loaded_name, AbstractPlanner::Ptr plugin) {
          return initializePlannerPlugin(loaded_name, cfg, std::move(plugin));
        });
  }
  // smoother
  for (const auto& path : moduleConfigFiles(nav_cfg, "config_smoother")) {
    ayanami::Config cfg(path);
    registerModuleConfigAliases("config_smoother", path, cfg);
    const std::string name = cfg.param<std::string>("smoother", "name", "");
    if (name.empty()) {
      continue;
    }
    if (default_smoother.empty()) {
      default_smoother = name;
    }
    smoother_plugins.loadSingleFromConfig(
        cfg, "smoother", name,
        [this, cfg](const std::string& loaded_name, AbstractSmoother::Ptr plugin) {
          return initializeSmootherPlugin(loaded_name, cfg, std::move(plugin));
        });
  }
  // controller
  for (const auto& path : moduleConfigFiles(nav_cfg, "config_controller")) {
    ayanami::Config cfg(path);
    registerModuleConfigAliases("config_controller", path, cfg);
    const std::string name = cfg.param<std::string>("controller", "name", "");
    if (name.empty()) {
      continue;
    }
    if (default_controller.empty()) {
      default_controller = name;
    }
    controller_plugins.loadSingleFromConfig(
        cfg, "controller", name,
        [this, cfg](const std::string& loaded_name, AbstractController::Ptr plugin) {
          return initializeControllerPlugin(loaded_name, cfg, std::move(plugin));
        });
  }
  // recovery (role 决定默认: clear_costmap 优先)
  for (const auto& path : moduleConfigFiles(nav_cfg, "config_recovery")) {
    ayanami::Config cfg(path);
    registerModuleConfigAliases("config_recovery", path, cfg);
    const std::string name = cfg.param<std::string>("recovery", "name", "");
    const std::string role = cfg.param<std::string>("recovery", "role", "");
    if (!name.empty()) recovery_sequence.push_back(name);
    recovery_plugins.loadSingleFromConfig(
        cfg, "recovery", name,
        [this, cfg](const std::string& loaded_name, AbstractRecovery::Ptr plugin) {
          return initializeRecoveryPlugin(loaded_name, cfg, std::move(plugin));
        });
  }
  {
      for (const auto& path : moduleConfigFiles(nav_cfg, "config_goal_checker")) {
      ayanami::Config cfg(path);
      registerModuleConfigAliases("config_goal_checker", path, cfg);
      const std::string name =
          cfg.param<std::string>("goal_checker", "name", "");
      if (name.empty()) {
        continue;
      }
      if (default_goal_checker.empty()) {
        default_goal_checker = name;
      }
      goal_checker_plugins.loadSingleFromConfig(
          cfg, "goal_checker", name,
          [this, cfg](const std::string& loaded_name,
                 AbstractGoalChecker::Ptr plugin) {
            return initializeGoalCheckerPlugin(loaded_name, cfg, std::move(plugin));
          });
    }
  }
  {
      for (const auto& path : moduleConfigFiles(nav_cfg, "config_progress_checker")) {
      ayanami::Config cfg(path);
      registerModuleConfigAliases("config_progress_checker", path, cfg);
      const std::string name =
          cfg.param<std::string>("progress_checker", "name", "");
      if (name.empty()) {
        continue;
      }
      if (default_progress_checker.empty()) {
        default_progress_checker = name;
      }
      progress_checker_plugins.loadSingleFromConfig(
          cfg, "progress_checker", name,
          [this, cfg](const std::string& loaded_name,
                 AbstractProgressChecker::Ptr plugin) {
            return initializeProgressCheckerPlugin(loaded_name, cfg,
                                                   std::move(plugin));
          });
    }
  }

  createActions();
}

void NavigationServerBase::start() {
  if (running.exchange(true)) return;
  for (const auto& name : planner_plugins.getLoadedNames()) {
    if (auto plugin = planner_plugins.getPlugin(name)) plugin->activate();
  }
  for (const auto& name : smoother_plugins.getLoadedNames()) {
    if (auto plugin = smoother_plugins.getPlugin(name)) plugin->activate();
  }
  for (const auto& name : controller_plugins.getLoadedNames()) {
    if (auto plugin = controller_plugins.getPlugin(name)) plugin->activate();
  }
  for (const auto& name : recovery_plugins.getLoadedNames()) {
    if (auto plugin = recovery_plugins.getPlugin(name)) plugin->activate();
  }
}

void NavigationServerBase::stop() {
  if (!running.exchange(false)) return;
  if (planner_action) planner_action->cancelAll();
  if (smoother_action) smoother_action->cancelAll();
  if (controller_action) controller_action->cancelAll();
  for (auto& kv : recovery_actions) {
    if (kv.second) kv.second->cancelAll();
  }
  for (const auto& name : recovery_plugins.getLoadedNames()) {
    if (auto plugin = recovery_plugins.getPlugin(name)) plugin->deactivate();
  }
  for (const auto& name : controller_plugins.getLoadedNames()) {
    if (auto plugin = controller_plugins.getPlugin(name)) plugin->deactivate();
  }
  for (const auto& name : smoother_plugins.getLoadedNames()) {
    if (auto plugin = smoother_plugins.getPlugin(name)) plugin->deactivate();
  }
  for (const auto& name : planner_plugins.getLoadedNames()) {
    if (auto plugin = planner_plugins.getPlugin(name)) plugin->deactivate();
  }
}

void NavigationServerBase::createActions() {
  planner_action.reset();
  smoother_action.reset();
  controller_action.reset();
  recovery_actions.clear();

  if (auto planner = planner_plugins.getPlugin(default_planner)) {
    auto action = std::make_unique<PlannerAction>(planner);
    action->setExecutionFactory([this]() {
      return std::shared_ptr<PlannerExecution>(
          newPlannerExecution(default_planner, planner_plugins.getPlugin(default_planner)));
    });
    planner_action = std::move(action);
  }
  if (!planner_plugins.hasPlugin(default_planner)) {
    spdlog::error("NavigationServerBase::createActions: default planner '{}' "
                  "not loaded",
                  default_planner);
  }

  if (auto smoother = smoother_plugins.getPlugin(default_smoother)) {
    auto action = std::make_unique<SmootherAction>(smoother);
    action->setExecutionFactory([this]() {
      return std::shared_ptr<SmootherExecution>(
          newSmootherExecution(default_smoother, smoother_plugins.getPlugin(default_smoother)));
    });
    smoother_action = std::move(action);
  }
  if (!smoother_plugins.hasPlugin(default_smoother)) {
    spdlog::error("NavigationServerBase::createActions: default smoother '{}' "
                  "not loaded",
                  default_smoother);
  }

  if (auto controller = controller_plugins.getPlugin(default_controller)) {
    auto action = std::make_unique<ControllerAction>(controller);
    action->setExecutionFactory([this]() {
      auto exec = std::shared_ptr<ControllerExecution>(newControllerExecution(
          default_controller, controller_plugins.getPlugin(default_controller)));
      configureControllerExecution(exec);
      return exec;
    });
    controller_action = std::move(action);
  }
  if (!controller_plugins.hasPlugin(default_controller)) {
    spdlog::error("NavigationServerBase::createActions: default controller "
                  "'{}' not loaded",
                  default_controller);
  }

  // recovery: 为所有已装载 recovery 插件建 action (requestRecovery 按 id 分派)
  for (const auto& name : recovery_plugins.getLoadedNames()) {
    auto recovery = recovery_plugins.getPlugin(name);
    if (!recovery) {
      continue;
    }
    auto action = std::make_unique<RecoveryAction>(recovery);
    action->setExecutionFactory([this, name]() {
      auto exec = std::shared_ptr<RecoveryExecution>(newRecoveryExecution(
          name, recovery_plugins.getPlugin(name)));
      configureRecoveryExecution(exec);
      return exec;
    });
    recovery_actions[name] = std::move(action);
  }
  if (recovery_actions.empty()) {
    spdlog::warn("NavigationServerBase::createActions: no recovery plugins loaded");
  }
}

void NavigationServerBase::configureControllerExecution(
    std::shared_ptr<ControllerExecution> exec) const {
  // 目标/进展检查插件注入 (execution 运行时消费)。
  if (auto checker = goal_checker_plugins.getPlugin(default_goal_checker)) {
    exec->setGoalChecker(std::move(checker));
  } else if (!default_goal_checker.empty()) {
    spdlog::warn("NavigationServerBase: goal checker '{}' not loaded",
                 default_goal_checker);
  }
  if (auto checker =
          progress_checker_plugins.getPlugin(default_progress_checker)) {
    exec->setProgressChecker(std::move(checker));
  } else if (!default_progress_checker.empty()) {
    spdlog::warn("NavigationServerBase: progress checker '{}' not loaded",
                 default_progress_checker);
  }
  // 速度上限注入 (navigation.max_vel_{x,y,theta} 缓存; validVelocityCommand
  // 在 execution 层拒绝超限命令, max_vel_y=0 同时禁止横移)。
  exec->setMaximumVelocity(max_vel_x, max_vel_y, max_vel_theta);
}

void NavigationServerBase::configureRecoveryExecution(
    std::shared_ptr<RecoveryExecution> exec) const {
  // 速度上限注入 (与 controller 同一来源 navigation.max_vel_*);
  // RecoveryExecution 在唯一发布前校验 finite + max_vel 并清零非法命令。
  exec->setMaximumVelocity(max_vel_x, max_vel_y, max_vel_theta);
}

PlannerAction* NavigationServerBase::plannerAction(
    const std::string& id) const {
  const std::string key = id.empty() ? default_planner : id;
  (void)key;
  return planner_action.get();
}

SmootherAction* NavigationServerBase::smootherAction(
    const std::string& id) const {
  const std::string key = id.empty() ? default_smoother : id;
  (void)key;
  return smoother_action.get();
}

ControllerAction* NavigationServerBase::controllerAction(
    const std::string& id) const {
  const std::string key = id.empty() ? default_controller : id;
  (void)key;
  return controller_action.get();
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
  if (recovery_exclusive.load() || !planner_action) {
    return rejectedHandle<ayanami::actions::GetPathResult>(
        ErrorCode::RESOURCE_BUSY, "planner unavailable", currentSessionId(),
        currentPlanVersion());
  }
  auto g = goal;
  g.session_id = currentSessionId();
  g.plan_version = currentPlanVersion();
  return planner_action->acceptGoal(g);
}

TaskHandle<ayanami::actions::SmoothPathResult>
NavigationServerBase::requestSmoothing(
    const ayanami::actions::SmoothPathGoal& goal) {
  std::lock_guard<std::mutex> lock(smoother_request_mutex);
  if (recovery_exclusive.load() || !smoother_action) {
    return rejectedHandle<ayanami::actions::SmoothPathResult>(
        ErrorCode::RESOURCE_BUSY, "smoother unavailable", currentSessionId(),
        currentPlanVersion());
  }
  auto g = goal;
  g.session_id = currentSessionId();
  g.plan_version = currentPlanVersion();
  return smoother_action->acceptGoal(g);
}

TaskHandle<ayanami::actions::ExePathResult>
NavigationServerBase::requestExecution(
    const ayanami::actions::ExePathGoal& goal) {
  std::lock_guard<std::mutex> lock(controller_request_mutex);
  if (recovery_exclusive.load() || !controller_action) {
    return rejectedHandle<ayanami::actions::ExePathResult>(
        ErrorCode::RESOURCE_BUSY, "controller unavailable", currentSessionId(),
        currentPlanVersion());
  }
  auto g = goal;
  g.session_id = currentSessionId();
  g.plan_version = currentPlanVersion();
  return controller_action->acceptGoal(g);
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
        "unknown recovery plugin id: " + goal.recovery_id, currentSessionId(),
        currentPlanVersion());
  }
  // Stop the controller only after validating the recovery id. This is
  // synchronous and guarantees no controller can publish during recovery.
  cancelAllControllers();
  ayanami::actions::RecoveryGoal g = goal;
  g.session_id = currentSessionId();
  g.plan_version = currentPlanVersion();
  try {
    auto task = iter->second->acceptGoal(g);
    // 资源协议: recovery 结束 (含异常兜底路径的 publishResult) 后才允许
    // controller 恢复接受请求。
    task.onComplete([this] { recovery_exclusive.store(false); });
    return task;
  } catch (...) {
    // acceptGoal 异常: 无 TaskHandle 可挂释放回调, 立即归还互斥再抛出。
    recovery_exclusive.store(false);
    throw;
  }
}

bool NavigationServerBase::requestPlanUpdate(
    uint64_t goal_id, const ayanami::actions::ExePathGoal& goal) {
  ControllerAction* action = controllerAction();
  if (!action) {
    spdlog::error("NavigationServerBase::requestPlanUpdate: no controller "
                  "action for '{}'",
                  default_controller);
    return false;
  }
  return action->updatePlan(goal_id, goal.path, goal.plan_version);
}

void NavigationServerBase::cancelAllControllers() {
  std::lock_guard<std::mutex> request_lock(controller_request_mutex);
  // 抢占/新会话/halt 纪律: 取消所有装载 controller 的 action map, 而不只是
  // 默认 id 的那一个 —— multiple controller configs 下默认 id 之外的控制器
  // 任务同样必须停止, 否则抢占后残留速度输出。
  if (controller_action) controller_action->cancelAll();
}

void NavigationServerBase::cancelAllPlanners() {
  std::lock_guard<std::mutex> request_lock(planner_request_mutex);
  if (planner_action) planner_action->cancelAll();
}

void NavigationServerBase::cancelAllSmoothers() {
  std::lock_guard<std::mutex> request_lock(smoother_request_mutex);
  if (smoother_action) smoother_action->cancelAll();
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

// ---- 语义: 无效插件 id 的结构化失败 TaskHandle ----

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

// ---- 虚钩子默认实现 (spec §5.3) ----

bool NavigationServerBase::initializePlannerPlugin(
    const std::string& name, const ayanami::Config&, AbstractPlanner::Ptr) {
  (void)name;
  return true;
}

bool NavigationServerBase::initializeSmootherPlugin(
    const std::string& name, const ayanami::Config&, AbstractSmoother::Ptr) {
  (void)name;
  return true;
}

bool NavigationServerBase::initializeControllerPlugin(
    const std::string& name, const ayanami::Config&, AbstractController::Ptr) {
  (void)name;
  return true;
}

bool NavigationServerBase::initializeRecoveryPlugin(
    const std::string& name, const ayanami::Config&, AbstractRecovery::Ptr) {
  (void)name;
  return true;
}

bool NavigationServerBase::initializeGoalCheckerPlugin(
    const std::string& name, const ayanami::Config&, AbstractGoalChecker::Ptr) {
  (void)name;
  return true;
}

bool NavigationServerBase::initializeProgressCheckerPlugin(
    const std::string& name, const ayanami::Config&,
    AbstractProgressChecker::Ptr) {
  (void)name;
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
