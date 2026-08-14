#pragma once

#include "ayanami/components/controller_action.hpp"
#include "ayanami/components/controller_execution.hpp"
#include "ayanami/components/planner_action.hpp"
#include "ayanami/components/planner_execution.hpp"
#include "ayanami/components/plugin_manager.hpp"
#include "ayanami/components/recovery_action.hpp"
#include "ayanami/components/recovery_execution.hpp"
#include "ayanami/components/robot_state_provider.hpp"
#include "ayanami/components/smoother_action.hpp"
#include "ayanami/components/smoother_execution.hpp"
#include "ayanami/core/abstract_controller.hpp"
#include "ayanami/core/abstract_planner.hpp"
#include "ayanami/core/abstract_recovery.hpp"
#include "ayanami/core/abstract_smoother.hpp"
#include "ayanami/core/map_context.hpp"
#include "ayanami/interfaces/callbacks.hpp"
#include "ayanami/messages/actions.hpp"

#include <atomic>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace ayanami {
namespace components {

/**
 * @brief 导航服务基类 (spec §5.3, 无地图依赖)。
 *
 * 持 4 个 PluginManager + 4 个 Action + AbstractMapContext + RobotStateProvider。
 * 地图对象 (costmap/gridmap) 由派生 server (如 CostmapNavigationServer, §5.4)
 * 覆写 virtual initialize*Plugin 注入; virtual new*Execution 工厂让派生返回
 * 带地图的 Execution。
 *
 * 主流控制流: requestPath/requestSmoothing/requestExecution/requestRecovery
 * 返回 TaskHandle 直连调用者 (BT 节点, spec §3.4); 旁路由各 Action 在
 * NavigationCallbacks::on_*_result 上 emit (rviz 扩展/回退/监控)。
 * registerPlannerFallback 订阅 on_get_path_result, 规划失败时自动触发默认
 * recovery action (spec §3.4 旁路用途)。
 */
class NavigationServerBase {
public:
  using MapContextPtr = ayanami::abstract::AbstractMapContext::Ptr;

  virtual ~NavigationServerBase();

  /// @brief 读 config.json global 段 → 4 个子配置逐 dlopen + 调虚
  /// initialize*Plugin → createActions()。
  void loadPlugins(const std::string& config_path);

  /// @brief 用 default_*_ 插件名取插件, 经虚 new*Execution 工厂建 4 个 Action。
  void createActions();

  /// @brief 启动服务 (幂等; 预留生命周期钩子)。
  void start();

  /// @brief 停止服务: 取消并 join 所有 RUNNING action (幂等)。
  void stop();

  PlannerAction* plannerAction() const;
  SmootherAction* smootherAction() const;
  ControllerAction* controllerAction() const;
  RecoveryAction* recoveryAction(const std::string& name) const;
  std::vector<std::string> recoveryActionNames() const;

  /// @brief 主流 TaskHandle 直连 (spec §3.4): 以 server 当前 session/plan_version
  /// 填充 Goal 后 acceptGoal。
  TaskHandle<ayanami::actions::GetPathResult> requestPath(
      const ayanami::actions::GetPathGoal& goal);
  TaskHandle<ayanami::actions::SmoothPathResult> requestSmoothing(
      const ayanami::actions::SmoothPathGoal& goal);
  TaskHandle<ayanami::actions::ExePathResult> requestExecution(
      const ayanami::actions::ExePathGoal& goal);
  TaskHandle<ayanami::actions::RecoveryResult> requestRecovery(
      const ayanami::actions::RecoveryGoal& goal);

  uint64_t currentSessionId() const;
  uint64_t currentPlanVersion() const;
  void incrementPlanVersion();
  bool sessionIsCurrent(uint64_t session_id) const;
  bool planVersionIsCurrent(uint64_t plan_version) const;

  /// @brief 订阅 on_get_path_result; 失败 (ABORTED) 时自动触发默认 recovery
  /// action 的 acceptGoal。只注册一次。
  void registerPlannerFallback();

  RobotStateProvider& robotStateProvider();

protected:
  // ★ 虚钩子: 派生覆写注入地图对象 (spec §5.3 关键)。默认 return true。
  virtual bool initializePlannerPlugin(
      const std::string& name, ayanami::abstract::AbstractPlanner::Ptr);
  virtual bool initializeSmootherPlugin(
      const std::string& name, ayanami::abstract::AbstractSmoother::Ptr);
  virtual bool initializeControllerPlugin(
      const std::string& name, ayanami::abstract::AbstractController::Ptr);
  virtual bool initializeRecoveryPlugin(
      const std::string& name, ayanami::abstract::AbstractRecovery::Ptr);

  // ★ 工厂: 派生覆写返回带地图的 Execution。默认返回 abstract 版。
  virtual std::unique_ptr<PlannerExecution> newPlannerExecution(
      const std::string& name, ayanami::abstract::AbstractPlanner::Ptr);
  virtual std::unique_ptr<ControllerExecution> newControllerExecution(
      const std::string& name, ayanami::abstract::AbstractController::Ptr);
  virtual std::unique_ptr<RecoveryExecution> newRecoveryExecution(
      const std::string& name, ayanami::abstract::AbstractRecovery::Ptr);

  PluginManager<ayanami::abstract::AbstractPlanner> planner_plugins_;
  PluginManager<ayanami::abstract::AbstractSmoother> smoother_plugins_;
  PluginManager<ayanami::abstract::AbstractController> controller_plugins_;
  PluginManager<ayanami::abstract::AbstractRecovery> recovery_plugins_;
  std::unique_ptr<PlannerAction> planner_action_;
  std::unique_ptr<SmootherAction> smoother_action_;
  std::unique_ptr<ControllerAction> controller_action_;
  std::map<std::string, std::unique_ptr<RecoveryAction>> recovery_actions_;
  RobotStateProvider robot_state_provider_;
  MapContextPtr map_context_;  // 通用, 派生层注入 (spec §5.2)
  std::string default_planner_;
  std::string default_smoother_;
  std::string default_controller_;
  std::string default_recovery_;
  std::atomic<uint64_t> session_id_{0};
  std::atomic<uint64_t> plan_version_{0};

private:
  std::atomic<bool> running_{false};
  int fallback_callback_id_{-1};  // -1 = 未注册
};

}  // namespace components
}  // namespace ayanami
