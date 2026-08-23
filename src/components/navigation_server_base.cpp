// NavigationServerBase 主实现 (spec §5.3)。
//
// 生命周期纪律 (spec §4.5): 析构先 stop() 停全部 action (取消 + join 槽线程),
// 再 reset 4 个 action, 再 clearPlugins 释放插件对象 (dlopen 句柄仍存活时析构),
// 最后移除旁路 fallback 回调。零 ROS include (红线 §5.7)。

#include "ayanami/abstract/components/navigation_server_base.hpp"

#include "ayanami/utility/config.hpp"

#include <spdlog/spdlog.h>

namespace ayanami {
namespace components {

using ayanami::abstract::AbstractController;
using ayanami::abstract::AbstractGoalChecker;
using ayanami::abstract::AbstractPlanner;
using ayanami::abstract::AbstractProgressChecker;
using ayanami::abstract::AbstractRecovery;
using ayanami::abstract::AbstractSmoother;

NavigationServerBase::~NavigationServerBase() {
  stop();
  if (fallback_callback_id_ >= 0) {
    ayanami::interfaces::NavigationCallbacks::on_get_path_result.remove(
        fallback_callback_id_);
  }
  planner_action_.reset();
  smoother_action_.reset();
  controller_action_.reset();
  recovery_actions_.clear();
  planner_plugins_.clearPlugins();
  smoother_plugins_.clearPlugins();
  controller_plugins_.clearPlugins();
  recovery_plugins_.clearPlugins();
  goal_checker_plugins_.clearPlugins();
  progress_checker_plugins_.clearPlugins();
}

void NavigationServerBase::loadPlugins(const std::string& config_path) {
  GlobalConfig::instance(config_path, /*override_path=*/true);

  {
    Config cfg(GlobalConfig::get_config_path("config_navigation", "config_planner"));
    default_planner_ = cfg.param<std::string>("planner", "name", "");
    planner_plugins_.loadSingleFromConfig(
        cfg, "planner", default_planner_,
        [this](const std::string& name, AbstractPlanner::Ptr plugin) {
          return initializePlannerPlugin(name, std::move(plugin));
        });
  }
  {
    Config cfg(GlobalConfig::get_config_path("config_navigation", "config_smoother"));
    default_smoother_ = cfg.param<std::string>("smoother", "name", "");
    smoother_plugins_.loadSingleFromConfig(
        cfg, "smoother", default_smoother_,
        [this](const std::string& name, AbstractSmoother::Ptr plugin) {
          return initializeSmootherPlugin(name, std::move(plugin));
        });
  }
  {
    Config cfg(GlobalConfig::get_config_path("config_navigation", "config_controller"));
    default_controller_ = cfg.param<std::string>("controller", "name", "");
    controller_plugins_.loadSingleFromConfig(
        cfg, "controller", default_controller_,
        [this](const std::string& name, AbstractController::Ptr plugin) {
          return initializeControllerPlugin(name, std::move(plugin));
        });
  }
  for (const auto& path : GlobalConfig::get_config_paths("config_navigation", "config_recovery")) {
    Config cfg(path);
    const std::string name = cfg.param<std::string>("recovery", "name", "");
    const std::string role = cfg.param<std::string>("recovery", "role", "");
    if (default_recovery_.empty() || role == "clear_costmap") {
      default_recovery_ = name;
    }
    recovery_plugins_.loadSingleFromConfig(
        cfg, "recovery", name,
        [this](const std::string& loaded_name, AbstractRecovery::Ptr plugin) {
          return initializeRecoveryPlugin(loaded_name, std::move(plugin));
        });
  }
  {
    Config cfg(GlobalConfig::get_config_path("config_navigation", "config_goal_checker"));
    default_goal_checker_ =
        cfg.param<std::string>("goal_checker", "name", "");
    goal_checker_plugins_.loadSingleFromConfig(
        cfg, "goal_checker", default_goal_checker_,
        [this](const std::string& name, AbstractGoalChecker::Ptr plugin) {
          return initializeGoalCheckerPlugin(name, std::move(plugin));
        });
  }
  {
    Config cfg(GlobalConfig::get_config_path("config_navigation", "config_progress_checker"));
    default_progress_checker_ =
        cfg.param<std::string>("progress_checker", "name", "");
    progress_checker_plugins_.loadSingleFromConfig(
        cfg, "progress_checker", default_progress_checker_,
        [this](const std::string& name, AbstractProgressChecker::Ptr plugin) {
          return initializeProgressCheckerPlugin(name, std::move(plugin));
        });
  }

  createActions();
}

void NavigationServerBase::start() { running_.store(true); }

void NavigationServerBase::stop() {
  running_.store(false);
  if (planner_action_) planner_action_->cancelAll();
  if (smoother_action_) smoother_action_->cancelAll();
  if (controller_action_) controller_action_->cancelAll();
  for (auto& kv : recovery_actions_) {
    if (kv.second) kv.second->cancelAll();
  }
}

PlannerAction* NavigationServerBase::plannerAction() const {
  return planner_action_.get();
}

SmootherAction* NavigationServerBase::smootherAction() const {
  return smoother_action_.get();
}

ControllerAction* NavigationServerBase::controllerAction() const {
  return controller_action_.get();
}

RecoveryAction* NavigationServerBase::recoveryAction(
    const std::string& name) const {
  auto iter = recovery_actions_.find(name);
  return iter == recovery_actions_.end() ? nullptr : iter->second.get();
}

std::vector<std::string> NavigationServerBase::recoveryActionNames() const {
  std::vector<std::string> names;
  names.reserve(recovery_actions_.size());
  for (const auto& kv : recovery_actions_) {
    names.push_back(kv.first);
  }
  return names;
}

TaskHandle<ayanami::actions::GetPathResult> NavigationServerBase::requestPath(
    const ayanami::actions::GetPathGoal& goal) {
  if (!planner_action_) {
    spdlog::error("NavigationServerBase::requestPath: planner action not "
                  "created (loadPlugins first)");
    return {};
  }
  ayanami::actions::GetPathGoal g = goal;
  g.session_id = currentSessionId();
  g.plan_version = currentPlanVersion();
  return planner_action_->acceptGoal(g);
}

TaskHandle<ayanami::actions::SmoothPathResult>
NavigationServerBase::requestSmoothing(
    const ayanami::actions::SmoothPathGoal& goal) {
  if (!smoother_action_) {
    spdlog::error("NavigationServerBase::requestSmoothing: smoother action not "
                  "created (loadPlugins first)");
    return {};
  }
  ayanami::actions::SmoothPathGoal g = goal;
  g.session_id = currentSessionId();
  g.plan_version = currentPlanVersion();
  return smoother_action_->acceptGoal(g);
}

TaskHandle<ayanami::actions::ExePathResult>
NavigationServerBase::requestExecution(
    const ayanami::actions::ExePathGoal& goal) {
  if (!controller_action_) {
    spdlog::error("NavigationServerBase::requestExecution: controller action "
                  "not created (loadPlugins first)");
    return {};
  }
  ayanami::actions::ExePathGoal g = goal;
  g.session_id = currentSessionId();
  g.plan_version = currentPlanVersion();
  return controller_action_->acceptGoal(g);
}

TaskHandle<ayanami::actions::RecoveryResult>
NavigationServerBase::requestRecovery(
    const ayanami::actions::RecoveryGoal& goal) {
  auto iter = recovery_actions_.find(goal.recovery_id);
  if (iter == recovery_actions_.end() || !iter->second) {
    spdlog::error("NavigationServerBase::requestRecovery: no recovery action "
                  "for '{}'",
                  goal.recovery_id);
    return {};
  }
  ayanami::actions::RecoveryGoal g = goal;
  g.session_id = currentSessionId();
  g.plan_version = currentPlanVersion();
  return iter->second->acceptGoal(g);
}

uint64_t NavigationServerBase::currentSessionId() const {
  return session_id_.load();
}

uint64_t NavigationServerBase::currentPlanVersion() const {
  return plan_version_.load();
}

void NavigationServerBase::incrementPlanVersion() {
  plan_version_.fetch_add(1);
}

bool NavigationServerBase::sessionIsCurrent(uint64_t session_id) const {
  return session_id == session_id_.load();
}

bool NavigationServerBase::planVersionIsCurrent(uint64_t plan_version) const {
  return plan_version == plan_version_.load();
}

void NavigationServerBase::registerPlannerFallback() {
  if (fallback_callback_id_ >= 0) {
    return;  // 只注册一次
  }
  fallback_callback_id_ =
      ayanami::interfaces::NavigationCallbacks::on_get_path_result.add(
          [this](const ayanami::actions::GoalResult<
                 ayanami::actions::GetPathResult>& result) {
            if (result.status != ayanami::actions::GoalStatus::ABORTED) {
              return;  // 仅规划失败触发, 取消/成功不触发
            }
            auto iter = recovery_actions_.find(default_recovery_);
            if (iter == recovery_actions_.end() || !iter->second) {
              spdlog::warn("NavigationServerBase: fallback recovery '{}' "
                           "action missing",
                           default_recovery_);
              return;
            }
            ayanami::actions::RecoveryGoal goal;
            goal.recovery_id = default_recovery_;
            goal.session_id = currentSessionId();
            goal.plan_version = currentPlanVersion();
            iter->second->acceptGoal(goal);
          });
}

RobotStateProvider& NavigationServerBase::robotStateProvider() {
  return robot_state_provider_;
}

// ---- 虚钩子默认实现 (spec §5.3) ----

bool NavigationServerBase::initializePlannerPlugin(const std::string& name,
                                                   AbstractPlanner::Ptr) {
  (void)name;
  return true;
}

bool NavigationServerBase::initializeSmootherPlugin(const std::string& name,
                                                    AbstractSmoother::Ptr) {
  (void)name;
  return true;
}

bool NavigationServerBase::initializeControllerPlugin(
    const std::string& name, AbstractController::Ptr) {
  (void)name;
  return true;
}

bool NavigationServerBase::initializeRecoveryPlugin(const std::string& name,
                                                    AbstractRecovery::Ptr) {
  (void)name;
  return true;
}

bool NavigationServerBase::initializeGoalCheckerPlugin(
    const std::string& name, AbstractGoalChecker::Ptr) {
  (void)name;
  return true;
}

bool NavigationServerBase::initializeProgressCheckerPlugin(
    const std::string& name, AbstractProgressChecker::Ptr) {
  (void)name;
  return true;
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
