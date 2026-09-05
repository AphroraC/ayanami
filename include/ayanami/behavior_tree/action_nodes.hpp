#pragma once

#include "ayanami/behavior_tree/blackboard_protocol.hpp"
#include "ayanami/behavior_tree/runtime_lookup.hpp"
#include "ayanami/abstract/components/navigation_server_base.hpp"
#include "ayanami/messages/actions.hpp"
#include "ayanami/messages/failure.hpp"
#include "ayanami/messages/geometry_msgs.hpp"
#include "ayanami/messages/nav_msgs.hpp"

#include <behaviortree_cpp/action_node.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>

namespace ayanami {
namespace behavior_tree {

/// @brief 把结构化失败上下文写入黑板 (spec §6.2 关键纪律扩展)。节点在返回
/// FAILURE 前调用, 使 Recovery 节点能读取到精确 cause。
inline void writeFailureContext(const BT::TreeNode& node,
                                const FailureContext& context) {
  if (node.config().blackboard) {
    node.config().blackboard->set<FailureContext>(k_failure_context, context);
  }
}

/// @brief NaN = "端口未提供" 哨兵 (BT InputPort double 缺省值)。
inline constexpr double k_behavior_param_unspecified =
    std::numeric_limits<double>::quiet_NaN();

/// @brief 读取端口 double; 非 NaN 才写入 optional (NaN 保持未提供)。
inline void applyBehaviorPort(const BT::TreeNode& node, const char* port,
                              std::optional<double>& out) {
  double value = k_behavior_param_unspecified;
  if (node.getInput(port, value) && !std::isnan(value)) {
    out = value;
  }
}

/**
 * @brief ComputePathToPose 异步 action (spec §6.2): TaskHandle 直连主流。
 *
 * onStart: 读 goal 端口, 组 GetPathGoal (填 server 当前 session/plan_version +
  * requestPath 持 TaskHandle, 注册 onComplete ->
 * emitWakeUpSignal。onRunning: 等 task ready; sessionIsCurrent +
 * planVersionIsCurrent 双校验防 stale 竞态 (spec §6.2 关键纪律), 通过后写 path
 * 端口 (XML 绑定 {raw_path}) 并 incrementPlanVersion。onHalted: 取消在飞 task。
 * 任何 FAILURE 前把精确 FailureContext 写入黑板 (使 Recovery 可获得 cause)。
 *
 * 端口: goal={goal} (输入), path={raw_path} (输出),
 * path_id={path_id} (输出, 每次成功规划单调递增; smoother 分支以此为准重新平滑)。
 */
class ComputePathToPoseNode : public BT::StatefulActionNode {
 public:
  ComputePathToPoseNode(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config),
       nav_server(lookupNavigationRuntime(config)) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<geometry_msgs::PoseStamped>(k_goal, "navigation goal"),
      BT::OutputPort<nav_msgs::Path>("path", "planned path"),
      BT::OutputPort<uint64_t>(k_path_id,
                               "monotonic id bumped on every fresh plan"),
    };
  }

  BT::NodeStatus onStart() override {
    task = {};
    if (!nav_server) {
      FailureContext ctx;
      ctx.stage = FailureStage::Planner;
       ctx.code = ErrorCode::INTERNAL_ERROR;
      ctx.message = "navigation server not injected";
      writeFailureContext(*this, ctx);
      return BT::NodeStatus::FAILURE;
    }
    geometry_msgs::PoseStamped goal;
    if (!getInput(k_goal, goal)) {
      FailureContext ctx;
      ctx.stage = FailureStage::Planner;
       ctx.code = ErrorCode::INVALID_INPUT;
      ctx.message = "goal port missing";
      ctx.session_id = nav_server->currentSessionId();
      ctx.plan_version = nav_server->currentPlanVersion();
      writeFailureContext(*this, ctx);
      return BT::NodeStatus::FAILURE;
    }
    submitted_goal = goal;
    actions::GetPathGoal g;
    g.goal = goal;
    g.session_id = nav_server->currentSessionId();
    g.plan_version = nav_server->currentPlanVersion();
    task = nav_server->requestPath(g);
    if (!task.valid()) {
      FailureContext ctx;
      ctx.stage = FailureStage::Planner;
       ctx.code = ErrorCode::INTERNAL_ERROR;
      ctx.message = "requestPath returned invalid task";
      ctx.plugin_id = "planner";
      ctx.session_id = nav_server->currentSessionId();
      ctx.plan_version = nav_server->currentPlanVersion();
      writeFailureContext(*this, ctx);
      return BT::NodeStatus::FAILURE;
    }
    task.onComplete([this] { emitWakeUpSignal(); });
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override {
    geometry_msgs::PoseStamped current_goal;
    if (getInput(k_goal, current_goal) && !goalEquals(current_goal, submitted_goal)) {
      actions::GetPathGoal update;
      update.goal = current_goal;
      update.session_id = nav_server->currentSessionId();
      update.plan_version = nav_server->currentPlanVersion();
      task = nav_server->requestPath(update);
      if (!task.valid()) return BT::NodeStatus::FAILURE;
      submitted_goal = current_goal;
      task.onComplete([this] { emitWakeUpSignal(); });
      return BT::NodeStatus::RUNNING;
    }
    if (!task.ready()) {
      return BT::NodeStatus::RUNNING;
    }
    const auto& r = task.result();
    if (!nav_server->sessionIsCurrent(r.session_id) ||
        !nav_server->planVersionIsCurrent(r.plan_version)) {
      FailureContext ctx;
      ctx.stage = FailureStage::Planner;
       ctx.code = ErrorCode::INTERNAL_ERROR;
      ctx.message = "stale session/plan_version result rejected";
       ctx.plugin_id = "planner";
      ctx.session_id = r.session_id;
      ctx.plan_version = r.plan_version;
      writeFailureContext(*this, ctx);
      return BT::NodeStatus::FAILURE;
    }
     if (r.error != ErrorCode::NONE) {
      FailureContext ctx;
      ctx.stage = FailureStage::Planner;
      ctx.code = r.error;
      ctx.message = r.message;
       ctx.plugin_id = "planner";
      ctx.session_id = r.session_id;
      ctx.plan_version = r.plan_version;
      writeFailureContext(*this, ctx);
      return BT::NodeStatus::FAILURE;  // 只依据 failure 决策, 不解析 message
    }
    setOutput("path", r.path);
    // path_id 单调递增 (本次规划 ≠ 上一次): 下游 smoother 分支的 RateController
    // 以 {path_id} 为 reset_token, 新路径落地即重新平滑并 bump plan_version,
    // controller 经 in-place updatePlan 跟上; goal 未变时的重规划同样触发。
    setOutput(k_path_id, ++planned_path_id);
    return BT::NodeStatus::SUCCESS;
  }

  void onHalted() override { task.cancel(); }

 private:
  components::NavigationRuntime::Ptr nav_server;
  components::TaskHandle<actions::GetPathResult> task;
  geometry_msgs::PoseStamped submitted_goal;
  uint64_t planned_path_id{0};
};

/**
 * @brief SmoothPath 一次性异步 action (spec §6.2 同构)。
 *
 * 端口: input_path={raw_path} (输入), path={smoothed_path} (输出),
  * 使用配置期唯一 smoother。
 */
class SmoothPathNode : public BT::StatefulActionNode {
 public:
  SmoothPathNode(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config),
       nav_server(lookupNavigationRuntime(config)) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<nav_msgs::Path>("input_path", "path to smooth"),
      BT::OutputPort<nav_msgs::Path>("path", "smoothed path"),
    };
  }

  BT::NodeStatus onStart() override {
    task = {};
    if (!nav_server) {
      FailureContext ctx;
      ctx.stage = FailureStage::Smoother;
       ctx.code = ErrorCode::INTERNAL_ERROR;
      ctx.message = "navigation server not injected";
      writeFailureContext(*this, ctx);
      return BT::NodeStatus::FAILURE;
    }
    nav_msgs::Path path;
    if (!getInput("input_path", path)) {
      FailureContext ctx;
      ctx.stage = FailureStage::Smoother;
       ctx.code = ErrorCode::INVALID_INPUT;
      ctx.message = "input_path port missing";
      ctx.session_id = nav_server->currentSessionId();
      ctx.plan_version = nav_server->currentPlanVersion();
      writeFailureContext(*this, ctx);
      return BT::NodeStatus::FAILURE;
    }
    actions::SmoothPathGoal g;
    g.path = path;
    g.session_id = nav_server->currentSessionId();
    g.plan_version = nav_server->currentPlanVersion();
    task = nav_server->requestSmoothing(g);
    if (!task.valid()) {
      FailureContext ctx;
      ctx.stage = FailureStage::Smoother;
       ctx.code = ErrorCode::INTERNAL_ERROR;
      ctx.message = "requestSmoothing returned invalid task";
      ctx.plugin_id = "smoother";
      ctx.session_id = nav_server->currentSessionId();
      ctx.plan_version = nav_server->currentPlanVersion();
      writeFailureContext(*this, ctx);
      return BT::NodeStatus::FAILURE;
    }
    task.onComplete([this] { emitWakeUpSignal(); });
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override {
    if (!task.ready()) {
      return BT::NodeStatus::RUNNING;
    }
    const auto& r = task.result();
    if (!nav_server->sessionIsCurrent(r.session_id) ||
        !nav_server->planVersionIsCurrent(r.plan_version)) {
      FailureContext ctx;
      ctx.stage = FailureStage::Smoother;
       ctx.code = ErrorCode::INTERNAL_ERROR;
      ctx.message = "stale session/plan_version result rejected";
      ctx.plugin_id = "smoother";
      ctx.session_id = r.session_id;
      ctx.plan_version = r.plan_version;
      writeFailureContext(*this, ctx);
      return BT::NodeStatus::FAILURE;
    }
     if (r.error != ErrorCode::NONE) {
      // 平滑失败显式化: 写入黑板 failure_context 并返回 FAILURE, 由上层
      // RecoveryNode/恢复分支处理; 不做 best-effort 降级 (不掩盖算法故障)。
      FailureContext ctx;
      ctx.stage = FailureStage::Smoother;
      ctx.code = r.error;
      ctx.message = r.message;
      ctx.plugin_id = "smoother";
      ctx.session_id = r.session_id;
      ctx.plan_version = r.plan_version;
      writeFailureContext(*this, ctx);
      return BT::NodeStatus::FAILURE;
    }
    setOutput("path", r.smoothed_path);
    nav_server->incrementPlanVersion();
    return BT::NodeStatus::SUCCESS;
  }

  void onHalted() override { task.cancel(); }

 private:
  components::NavigationRuntime::Ptr nav_server;
  components::TaskHandle<actions::SmoothPathResult> task;
};

/**
 * @brief FollowPath 异步 action (spec §6.2 同构), 维持至 controller 到达目标。
 *
 * plan_version 变化时走 in-place 控制器更新 (requestPlanUpdate): 不 cancel /
 * 不 join / 不新建 Execution, 运行中的 ControllerAction 线程安全更新路径+版本,
 * execution 下一 cycle 消费 setNewPlan, 最终结果携带最新 plan_version。全程无
 * 中间停止 (无 cancel → 无中间零速发布)。
 *
 * 端口: path={smoothed_path} (输入), frequency={controller_frequency} (输入),
  * 使用配置期唯一 controller。
 */
class FollowPathNode : public BT::StatefulActionNode {
 public:
  FollowPathNode(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config),
       nav_server(lookupNavigationRuntime(config)) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<nav_msgs::Path>("path", "path to follow"),
      BT::InputPort<double>("frequency", 20.0, "controller frequency in Hz"),
    };
  }

  BT::NodeStatus onStart() override {
    task = {};
    if (!nav_server) {
      FailureContext ctx;
      ctx.stage = FailureStage::Controller;
       ctx.code = ErrorCode::INTERNAL_ERROR;
      ctx.message = "navigation server not injected";
      writeFailureContext(*this, ctx);
      return BT::NodeStatus::FAILURE;
    }
    nav_msgs::Path path;
    if (!getInput("path", path)) {
      FailureContext ctx;
      ctx.stage = FailureStage::Controller;
       ctx.code = ErrorCode::INVALID_INPUT;
      ctx.message = "path port missing";
      ctx.session_id = nav_server->currentSessionId();
      ctx.plan_version = nav_server->currentPlanVersion();
      writeFailureContext(*this, ctx);
      return BT::NodeStatus::FAILURE;
    }
    return startExecution(path) ? BT::NodeStatus::RUNNING
                                : BT::NodeStatus::FAILURE;
  }

  BT::NodeStatus onRunning() override {
    // 先查任务是否已完成: 已完成的 goal 不再尝试 in-place 更新 (execution 已
    // 释放), 直接消费结果 —— 避免对已完成的 goal 调 updatePlan 误报失败。
    if (task.ready()) {
      return evaluateCompletedResult();
    }
    // 任务仍在跑且 plan_version 变化: 原位更新正在执行的控制器 (不 cancel /
    // 不 join / 不新建 Execution)。
    if (nav_server->currentPlanVersion() != submitted_plan_version) {
      nav_msgs::Path path;
      if (!getInput("path", path)) {
        FailureContext ctx;
        ctx.stage = FailureStage::Controller;
         ctx.code = ErrorCode::INVALID_INPUT;
        ctx.message = "path port missing";
        ctx.plugin_id = "controller";
        ctx.session_id = nav_server->currentSessionId();
        ctx.plan_version = nav_server->currentPlanVersion();
        writeFailureContext(*this, ctx);
        return BT::NodeStatus::FAILURE;
      }
      if (!updateExecutionPlan(path)) {
        // update miss: 任务可能恰在提交期间完成/终态 (execution 已释放或 terminal,
        // requestPlanUpdate 拒绝更新)。重查已完成任务并消费其结果 —— 成功到达的
        // 结果直接 SUCCESS, 不把"更新未命中"误报成失败。
        if (task.ready()) {
          return evaluateCompletedResult();
        }
        return BT::NodeStatus::RUNNING;  // 尚未完成, 下一 tick 重试更新/消费
      }
      return BT::NodeStatus::RUNNING;
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override { task.cancel(); }

 private:
  /// @brief 已完成任务的结果评估 (onRunning 顶部 + update miss 重查共用)。
  ///
  /// 竞态纪律 (review 修复): "已到达目标" 的 SUCCESS 结果不因 pending 版本差异被
  /// 拒 —— controller 可能以旧 applied 版本完成 (更新在途未消费即到达), 这是合法
  /// 完成竞态; 版本陈旧拒绝只适用于失败/未达标结果 (真 stale 产物)。
  BT::NodeStatus evaluateCompletedResult() {
    const auto& r = task.result();
    if (!nav_server->sessionIsCurrent(r.session_id)) {
      FailureContext ctx;
      ctx.stage = FailureStage::Controller;
         ctx.code = ErrorCode::INTERNAL_ERROR;
      ctx.message = "stale session result rejected";
         ctx.plugin_id = "controller";
      ctx.session_id = r.session_id;
      ctx.plan_version = r.plan_version;
      writeFailureContext(*this, ctx);
      return BT::NodeStatus::FAILURE;
    }
    // controller 成功且到达目标: 会话内即终点 (goal 已达), pending 版本差异不拒。
     if (r.error == ErrorCode::NONE && r.reached) {
      return BT::NodeStatus::SUCCESS;
    }
    if (!nav_server->planVersionIsCurrent(r.plan_version)) {
      FailureContext ctx;
      ctx.stage = FailureStage::Controller;
       ctx.code = ErrorCode::INTERNAL_ERROR;
      ctx.message = "stale session/plan_version result rejected";
       ctx.plugin_id = "controller";
      ctx.session_id = r.session_id;
      ctx.plan_version = r.plan_version;
      writeFailureContext(*this, ctx);
      return BT::NodeStatus::FAILURE;
    }
    // 失败或未达标: 只依据 failure 决策, 不解析 message。
    FailureContext ctx;
    ctx.stage = FailureStage::Controller;
     ctx.code = r.error != ErrorCode::NONE ? r.error
                                              : ErrorCode::GOAL_NOT_REACHED;
    ctx.message = r.message;
       ctx.plugin_id = "controller";
    ctx.session_id = r.session_id;
    ctx.plan_version = r.plan_version;
    writeFailureContext(*this, ctx);
    return BT::NodeStatus::FAILURE;
  }

  bool startExecution(const nav_msgs::Path& path) {
    actions::ExePathGoal goal;
    goal.path = path;
    goal.session_id = nav_server->currentSessionId();
    goal.plan_version = nav_server->currentPlanVersion();
    submitted_plan_version = goal.plan_version;
    task = nav_server->requestExecution(goal);
    if (!task.valid()) {
      FailureContext ctx;
      ctx.stage = FailureStage::Controller;
       ctx.code = ErrorCode::INTERNAL_ERROR;
      ctx.message = "requestExecution returned invalid task";
       ctx.plugin_id = "controller";
      ctx.session_id = goal.session_id;
      ctx.plan_version = goal.plan_version;
      writeFailureContext(*this, ctx);
      return false;
    }
    task.onComplete([this] { emitWakeUpSignal(); });
    return true;
  }

  /// @brief plan_version 变化: 对仍在跑的目标原位更新路径+版本 (无 cancel/新
  /// Execution)。返回 false 表示更新失败 (节点据此 FAILURE)。
  bool updateExecutionPlan(const nav_msgs::Path& path) {
    actions::ExePathGoal goal;
    goal.path = path;
    goal.session_id = nav_server->currentSessionId();
    goal.plan_version = nav_server->currentPlanVersion();
    if (!nav_server->requestPlanUpdate(task.goal_id(), goal)) {
      return false;
    }
    submitted_plan_version = goal.plan_version;
    return true;
  }

  components::NavigationRuntime::Ptr nav_server;
  components::TaskHandle<actions::ExePathResult> task;
  uint64_t submitted_plan_version{0};
};

/**
 * @brief Recovery 一次性异步 action (spec §6.2 同构)。
 *
 * 端口: recovery={recovery_id}/{clear_costmap_id}/{spin_id}/{wait_id}/{backup_id}
 * (输入, XML `<Recovery recovery="{...}"/>`; 黑板键由 pilot 配置决定)。
 * requestRecovery 的 Goal.cause = 当前黑板 failure_context (结构化触发原因),
 * 使恢复插件可诊断是谁触发了恢复。
 */
class RecoveryActionNode : public BT::StatefulActionNode {
 public:
  RecoveryActionNode(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config),
      nav_server(lookupNavigationRuntime(config)) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<std::string>("recovery", "recovery plugin id"),
      // 按次覆盖参数 (spec: RecoveryGoal 常用字段)。NaN 默认 = 未提供, 保持
      // 插件 configure 期默认值。
      BT::InputPort<double>("target_yaw",
                            std::numeric_limits<double>::quiet_NaN(),
                            "override spin target angle [rad]"),
      BT::InputPort<double>("backup_distance",
                            std::numeric_limits<double>::quiet_NaN(),
                            "override back_up distance [m]"),
      BT::InputPort<double>("backup_speed",
                            std::numeric_limits<double>::quiet_NaN(),
                            "override back_up linear speed [m/s]"),
      BT::InputPort<double>("wait_duration",
                            std::numeric_limits<double>::quiet_NaN(),
                            "override wait duration [s]"),
    };
  }

  BT::NodeStatus onStart() override {
    task = {};
    if (!nav_server) {
      FailureContext ctx;
      ctx.stage = FailureStage::Recovery;
       ctx.code = ErrorCode::INTERNAL_ERROR;
      ctx.message = "navigation server not injected";
      writeFailureContext(*this, ctx);
      return BT::NodeStatus::FAILURE;
    }
    std::string recovery_id;
    if (!getInput("recovery", recovery_id)) {
      FailureContext ctx;
      ctx.stage = FailureStage::Recovery;
       ctx.code = ErrorCode::INVALID_INPUT;
      ctx.message = "recovery port missing";
      ctx.session_id = nav_server->currentSessionId();
      ctx.plan_version = nav_server->currentPlanVersion();
      writeFailureContext(*this, ctx);
      return BT::NodeStatus::FAILURE;
    }
    this->recovery_id = recovery_id;
    actions::RecoveryGoal g;
    g.recovery_id = recovery_id;
    readBehaviorParams(g);
    // cause: 携带当前黑板上的失败上下文 (触发链: 失败节点已写入)。
    if (config().blackboard) {
      (void)config().blackboard->get<FailureContext>(k_failure_context, g.cause);
    }
    g.session_id = nav_server->currentSessionId();
    g.plan_version = nav_server->currentPlanVersion();
    task = nav_server->requestRecovery(g);
    if (!task.valid()) {
      FailureContext ctx;
      ctx.stage = FailureStage::Recovery;
       ctx.code = ErrorCode::INTERNAL_ERROR;
      ctx.message = "requestRecovery returned invalid task";
      ctx.plugin_id = recovery_id;
      ctx.session_id = nav_server->currentSessionId();
      ctx.plan_version = nav_server->currentPlanVersion();
      writeFailureContext(*this, ctx);
      return BT::NodeStatus::FAILURE;
    }
    task.onComplete([this] { emitWakeUpSignal(); });
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override {
    if (!task.ready()) {
      return BT::NodeStatus::RUNNING;
    }
    const auto& r = task.result();
    if (!nav_server->sessionIsCurrent(r.session_id) ||
        !nav_server->planVersionIsCurrent(r.plan_version)) {
      FailureContext ctx;
      ctx.stage = FailureStage::Recovery;
       ctx.code = ErrorCode::INTERNAL_ERROR;
      ctx.message = "stale session/plan_version result rejected";
      ctx.plugin_id = recovery_id;
      ctx.session_id = r.session_id;
      ctx.plan_version = r.plan_version;
      writeFailureContext(*this, ctx);
      return BT::NodeStatus::FAILURE;
    }
     if (r.error != ErrorCode::NONE) {
      FailureContext ctx;
      ctx.stage = FailureStage::Recovery;
      ctx.code = r.error;
      ctx.message = r.message;
      ctx.plugin_id = recovery_id;
      ctx.session_id = r.session_id;
      ctx.plan_version = r.plan_version;
      writeFailureContext(*this, ctx);
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
  }

  void onHalted() override { task.cancel(); }

 private:
  /// @brief 读取按次覆盖端口 (NaN = 未提供) 填入 goal.behavior。
  void readBehaviorParams(actions::RecoveryGoal& g) const {
    applyBehaviorPort(*this, "target_yaw", g.behavior.target_yaw);
    applyBehaviorPort(*this, "backup_distance", g.behavior.backup_distance);
    applyBehaviorPort(*this, "backup_speed", g.behavior.backup_speed);
    applyBehaviorPort(*this, "wait_duration", g.behavior.wait_duration);
  }

  components::NavigationRuntime::Ptr nav_server;
  components::TaskHandle<actions::RecoveryResult> task;
  std::string recovery_id;
};

} // namespace behavior_tree
} // namespace ayanami
