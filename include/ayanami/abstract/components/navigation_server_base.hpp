#pragma once

#include "ayanami/abstract/components/controller_action.hpp"
#include "ayanami/abstract/components/controller_execution.hpp"
#include "ayanami/abstract/components/navigation_session_state.hpp"
#include "ayanami/abstract/components/planner_action.hpp"
#include "ayanami/abstract/components/planner_execution.hpp"
#include "ayanami/abstract/components/plugin_manager.hpp"
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
  virtual void cancelAllControllers() = 0;
  virtual void cancelAllPlanners() {}
  virtual void cancelAllSmoothers() {}
  /// @brief 路径有效性查询 (BT IsPathValid 条件节点消费; 职责表"路径是否
  /// 过期"的 BT 可读面)。默认 = 非空即有效; 地图族宿主覆写为真实校验
  /// (CostmapNavigationServer → AbstractMapContext::isPathValid)。
  virtual bool isPathValid(const ayanami::nav_msgs::Path& path) const {
    return !path.poses.empty();
  }
  virtual void setSessionState(
      std::shared_ptr<NavigationSessionState> session) = 0;
};

/**
 * @brief 导航服务基类 (spec §5.3, 无地图依赖)。
 *
 * 持插件管理器、四类 Action 和共享导航会话。
 * 地图对象 (costmap/gridmap) 由派生 server (如 CostmapNavigationServer, §5.4)
 * 覆写 virtual initialize*Plugin 注入; 派生覆写 createActions() 构造带地图的
 * Action (生产路径 = costmap_action)。基类 createActions() 经虚 new*Execution
 * 工厂装配 abstract Action (供无地图 server 与测试覆写)。
 *
 * 主流控制流: requestPath/requestSmoothing/requestExecution/requestRecovery
 * 返回 TaskHandle 直连调用者 (BT 节点, spec §3.4); 旁路由各 Action 在
 * NavigationCallbacks::on*_result 上 emit (rviz 扩展/监控)。规划失败不经旁路
 * 自动触发 recovery —— recovery 只由 BT 内 RecoveryNode/Recovery 节点调度。
 */
class NavigationServerBase : public NavigationRuntime {
public:

  virtual ~NavigationServerBase();

  /// @brief 读 config.json global 段 → 6 个子配置逐 dlopen + 调虚
  /// initialize*Plugin → createActions()。
  void loadPlugins(const std::string& config_path);

  /// @brief 用 default*_ 插件名取插件, 装配各 Action (每插件一个; 派生覆写以
  /// 返回带地图的 Action, 如 CostmapNavigationServer → Costmap*Action)。
  virtual void createActions();

  /// @brief 启动服务 (幂等; 预留生命周期钩子)。
  void start();

  /// @brief 停止服务: 取消并 join 所有 RUNNING action (幂等)。
  void stop();

  /// @brief Return the single configured action for the first three families.
  /// Recovery actions remain selectable by their configured recovery id.
  PlannerAction* plannerAction(const std::string& id = "") const;
  SmootherAction* smootherAction(const std::string& id = "") const;
  ControllerAction* controllerAction(const std::string& id = "") const;
  RecoveryAction* recoveryAction(const std::string& name) const;
  std::vector<std::string> recoveryActionNames() const;

  /// @brief Mainline TaskHandle requests. Planner, smoother and controller each
  /// dispatch to their one configured action; recovery uses recovery_id.
  /// virtual 供测试 Mock 覆写 (Task 13)。
  virtual TaskHandle<ayanami::actions::GetPathResult> requestPath(
      const ayanami::actions::GetPathGoal& goal);
  virtual TaskHandle<ayanami::actions::SmoothPathResult> requestSmoothing(
      const ayanami::actions::SmoothPathGoal& goal);
  virtual TaskHandle<ayanami::actions::ExePathResult> requestExecution(
      const ayanami::actions::ExePathGoal& goal);
  virtual TaskHandle<ayanami::actions::RecoveryResult> requestRecovery(
      const ayanami::actions::RecoveryGoal& goal);

  /// @brief 控制器 in-place 规划更新 (spec): 目标仍在跑时仅更新路径 + 版本,
  /// 无 cancel/join/new Execution。FollowPathNode 在 plan_version 变化时调用。
  virtual bool requestPlanUpdate(uint64_t goal_id,
                                 const ayanami::actions::ExePathGoal& goal);

  /// @brief 取消全部 controller action map (所有逻辑 id, 不止默认 id): 抢占 /
  /// 新会话 / halt 时清理所有可能在跑的 controller 任务。派生可覆写。
  virtual void cancelAllControllers();
  virtual void cancelAllPlanners();
  virtual void cancelAllSmoothers();

  // ---- 会话/版本 (单一来源 = 共享 NavigationSessionState) ----
  uint64_t currentSessionId() const;
  uint64_t currentPlanVersion() const;
  void incrementPlanVersion();
  bool sessionIsCurrent(uint64_t session_id) const;
  bool planVersionIsCurrent(uint64_t plan_version) const;
  /// @brief 开启新会话 (session+1 且 plan_version 归零)。
  void startNewSession();

  /// @brief 注入共享会话状态 (BehaviorTreePilot 调用 setNavigationRuntime 时传入,
  /// 使 pilot 与 server 使用同一 session/plan_version 计数); 未注入用内置实例。
  void setSessionState(std::shared_ptr<NavigationSessionState> session);
  std::shared_ptr<NavigationSessionState> sessionState() const;

protected:
  // ★ 虚钩子: 派生覆写注入地图对象 (spec §5.3 关键)。module_config 是该插件
  // 自身的模块 json (已解析), 派生用它构造 PluginContext 传入族级 configure。
  virtual bool initializePlannerPlugin(
      const std::string& name, const ayanami::Config& module_config,
      ayanami::abstract::AbstractPlanner::Ptr);
  virtual bool initializeSmootherPlugin(
      const std::string& name, const ayanami::Config& module_config,
      ayanami::abstract::AbstractSmoother::Ptr);
  virtual bool initializeControllerPlugin(
      const std::string& name, const ayanami::Config& module_config,
      ayanami::abstract::AbstractController::Ptr);
  virtual bool initializeRecoveryPlugin(
      const std::string& name, const ayanami::Config& module_config,
      ayanami::abstract::AbstractRecovery::Ptr);
  // ★ checker 虚钩子: 无地图依赖, 默认 return true (spec §5.3 扩展族)。
  virtual bool initializeGoalCheckerPlugin(
      const std::string& name, const ayanami::Config& module_config,
      ayanami::abstract::AbstractGoalChecker::Ptr);
  virtual bool initializeProgressCheckerPlugin(
      const std::string& name, const ayanami::Config& module_config,
      ayanami::abstract::AbstractProgressChecker::Ptr);

  // ★ 工厂: 派生覆写返回带地图的 Execution。基类 createActions() 经此装配
  // abstract Action (无地图 server 与测试覆写面)。默认返回 abstract 版。
  // CostmapNavigationServer 不再使用 (其 createActions 直接构造 Costmap*Action)。
  virtual std::unique_ptr<SmootherExecution> newSmootherExecution(
      const std::string& name, ayanami::abstract::AbstractSmoother::Ptr);
  virtual std::unique_ptr<PlannerExecution> newPlannerExecution(
      const std::string& name, ayanami::abstract::AbstractPlanner::Ptr);
  virtual std::unique_ptr<ControllerExecution> newControllerExecution(
      const std::string& name, ayanami::abstract::AbstractController::Ptr);
  virtual std::unique_ptr<RecoveryExecution> newRecoveryExecution(
      const std::string& name, ayanami::abstract::AbstractRecovery::Ptr);

  /// @brief 对 controller execution 施加 server 级装配 (goal/progress checker
  /// 注入 + 速度上限)。基类与 costmap 派生 createActions 共用; virtual 供
  /// 派生/测试注入自定义 checker 装配。
  virtual void configureControllerExecution(
      std::shared_ptr<ControllerExecution> exec) const;

  /// @brief 对 recovery execution 施加 server 级装配 (速度上限)。同上。
  virtual void configureRecoveryExecution(
      std::shared_ptr<RecoveryExecution> exec) const;

  /// @brief Shared dispatch for the three single-plugin families.
  PluginManager<ayanami::abstract::AbstractPlanner> planner_plugins;
  PluginManager<ayanami::abstract::AbstractSmoother> smoother_plugins;
  PluginManager<ayanami::abstract::AbstractController> controller_plugins;
  PluginManager<ayanami::abstract::AbstractRecovery> recovery_plugins;
  PluginManager<ayanami::abstract::AbstractGoalChecker> goal_checker_plugins;
  PluginManager<ayanami::abstract::AbstractProgressChecker>
      progress_checker_plugins;
  // The first three families each contain one configured plugin/action.
  std::unique_ptr<PlannerAction> planner_action;
  std::unique_ptr<SmootherAction> smoother_action;
  std::unique_ptr<ControllerAction> controller_action;
  std::map<std::string, std::unique_ptr<RecoveryAction>> recovery_actions;
  std::string default_planner;
  std::string default_smoother;
  std::string default_controller;
  std::vector<std::string> recovery_sequence;
  std::string default_goal_checker;
  std::string default_progress_checker;
  // 速度上限缓存 (navigation.max_vel_x/max_vel_y/max_vel_theta, 参数非负;
  // max_vel_y=0 禁止横移)。loadPlugins 读取缓存, createActions controller
  // execution factory 注入 ControllerExecution::setMaximumVelocity。缺省
  // Infinity = 不限幅 (与 ControllerExecution 默认一致)。
  double max_vel_x{std::numeric_limits<double>::infinity()};
  double max_vel_y{std::numeric_limits<double>::infinity()};
  double max_vel_theta{std::numeric_limits<double>::infinity()};
  // 会话/版本单一来源: 默认内置; BehaviorTreePilot setNavigationRuntime 时注入共享
  // 实例 (spec 单一来源纪律, 移除发散 pilot-only 计数)。
  std::shared_ptr<NavigationSessionState> session_state{
      std::make_shared<NavigationSessionState>()};

 private:
  std::atomic<bool> running{false};
  std::atomic<bool> recovery_exclusive{false};
  mutable std::mutex planner_request_mutex;
  mutable std::mutex smoother_request_mutex;
  mutable std::mutex controller_request_mutex;
  mutable std::atomic<uint64_t> next_rejected_id{1};

  /// @brief 构造立即失败的 TaskHandle (无效插件 id 等): valid()==true,
  /// ready()==true, status=ABORTED, result.failure 结构化失败码。
  template <typename Result>
  TaskHandle<Result> rejectedHandle(ErrorCode code,
                                    const std::string& message,
                                    uint64_t session_id,
                                    uint64_t plan_version) const;
};

}  // namespace components
}  // namespace ayanami
