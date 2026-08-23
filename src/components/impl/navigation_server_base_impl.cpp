// NavigationServerBase 核心 implementation: 内部派生 Action (经虚 new*Execution
// 工厂注入带地图的 Execution) + createActions。
//
// 背景: Task 9 的 *Action 基类内部 createExecution() 只构造 abstract Execution。
// 为使 spec §5.3 的 virtual new*Execution 工厂真正生效 (派生 CostmapNavigationServer
// 返回带地图的 Execution), 这里在 server 内部派生 3 个私有子类覆写 createExecution,
// 把每次 acceptGoal 的 Execution 创建转发给 server 的虚工厂。不改 Task 9 的 Action
// 公共接口。Smoother 无地图工厂 (spec §5.3 无 newSmootherExecution), 用默认执行。

#include "ayanami/abstract/components/navigation_server_base.hpp"

#include <spdlog/spdlog.h>

#include <functional>
#include <memory>
#include <string>
#include <utility>

namespace ayanami {
namespace components {

namespace {

using ayanami::abstract::AbstractController;
using ayanami::abstract::AbstractGoalChecker;
using ayanami::abstract::AbstractPlanner;
using ayanami::abstract::AbstractProgressChecker;
using ayanami::abstract::AbstractRecovery;

class ServerPlannerAction : public PlannerAction {
public:
  using PlannerAction::PlannerAction;

  void setExecutionFactory(std::function<PlannerExecution::Ptr()> factory) {
    factory_ = std::move(factory);
  }

protected:
  PlannerExecution::Ptr createExecution() override {
    return factory_ ? factory_() : PlannerAction::createExecution();
  }

private:
  std::function<PlannerExecution::Ptr()> factory_;
};

class ServerControllerAction : public ControllerAction {
public:
  using ControllerAction::ControllerAction;

  void setExecutionFactory(std::function<ControllerExecution::Ptr()> factory) {
    factory_ = std::move(factory);
  }

protected:
  ControllerExecution::Ptr createExecution() override {
    return factory_ ? factory_() : ControllerAction::createExecution();
  }

private:
  std::function<ControllerExecution::Ptr()> factory_;
};

class ServerRecoveryAction : public RecoveryAction {
public:
  using RecoveryAction::RecoveryAction;

  void setExecutionFactory(std::function<RecoveryExecution::Ptr()> factory) {
    factory_ = std::move(factory);
  }

protected:
  RecoveryExecution::Ptr createExecution() override {
    return factory_ ? factory_() : RecoveryAction::createExecution();
  }

private:
  std::function<RecoveryExecution::Ptr()> factory_;
};

}  // namespace

void NavigationServerBase::createActions() {
  if (auto planner = planner_plugins_.getPlugin(default_planner_)) {
    auto action = std::make_unique<ServerPlannerAction>(planner);
    action->setExecutionFactory([this]() {
      return std::shared_ptr<PlannerExecution>(newPlannerExecution(
          default_planner_, planner_plugins_.getPlugin(default_planner_)));
    });
    planner_action_ = std::move(action);
  } else {
    spdlog::error("NavigationServerBase::createActions: default planner '{}' "
                  "not loaded",
                  default_planner_);
  }

  if (auto smoother = smoother_plugins_.getPlugin(default_smoother_)) {
    smoother_action_ = std::make_unique<SmootherAction>(smoother);
  } else {
    spdlog::error("NavigationServerBase::createActions: default smoother '{}' "
                  "not loaded",
                  default_smoother_);
  }

  if (auto controller = controller_plugins_.getPlugin(default_controller_)) {
    auto action = std::make_unique<ServerControllerAction>(controller);
    action->setExecutionFactory([this]() {
      auto exec = std::shared_ptr<ControllerExecution>(newControllerExecution(
          default_controller_,
          controller_plugins_.getPlugin(default_controller_)));
      // 目标/进展检查插件注入 (预留集成; run 现未消费 checker)。
      if (auto checker = goal_checker_plugins_.getPlugin(default_goal_checker_)) {
        exec->setGoalChecker(std::move(checker));
      } else if (!default_goal_checker_.empty()) {
        spdlog::warn("NavigationServerBase::createActions: goal checker '{}' "
                     "not loaded",
                     default_goal_checker_);
      }
      if (auto checker =
              progress_checker_plugins_.getPlugin(default_progress_checker_)) {
        exec->setProgressChecker(std::move(checker));
      } else if (!default_progress_checker_.empty()) {
        spdlog::warn("NavigationServerBase::createActions: progress checker "
                     "'{}' not loaded",
                     default_progress_checker_);
      }
      return exec;
    });
    controller_action_ = std::move(action);
  } else {
    spdlog::error("NavigationServerBase::createActions: default controller '{}' "
                  "not loaded",
                  default_controller_);
  }

  // recovery: 为所有已装载 recovery 插件建 action (requestRecovery 按 id 分派)
  for (const auto& name : recovery_plugins_.getLoadedNames()) {
    auto recovery = recovery_plugins_.getPlugin(name);
    if (!recovery) {
      continue;
    }
    auto action = std::make_unique<ServerRecoveryAction>(recovery);
    action->setExecutionFactory([this, name]() {
      return std::shared_ptr<RecoveryExecution>(newRecoveryExecution(
          name, recovery_plugins_.getPlugin(name)));
    });
    recovery_actions_[name] = std::move(action);
  }
  if (!recovery_plugins_.hasPlugin(default_recovery_)) {
    spdlog::error("NavigationServerBase::createActions: default recovery '{}' "
                  "not loaded",
                  default_recovery_);
  }
}

}  // namespace components
}  // namespace ayanami
