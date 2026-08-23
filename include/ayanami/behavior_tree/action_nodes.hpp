#pragma once

#include "ayanami/behavior_tree/blackboard_protocol.hpp"
#include "ayanami/abstract/components/navigation_server_base.hpp"
#include "ayanami/messages/actions.hpp"
#include "ayanami/messages/failure.hpp"
#include "ayanami/messages/geometry_msgs.hpp"
#include "ayanami/messages/nav_msgs.hpp"

#include <behaviortree_cpp/action_node.h>

#include <cstdint>
#include <string>

namespace ayanami {
namespace behavior_tree {

// 黑板注入键: BT action 节点构造期从 blackboard 取 NavigationServerBase* (spec
// §6.2 "构造期注入"; registerNodeType 无参 + createTree 时黑板已就绪)。
// 不属于 16 个必需板项 (requiredBlackboardEntries 由 pilot/测试显式 set)。
inline constexpr const char* kNavigationServer = "navigation_server";

/// @brief 构造期从 blackboard 注入 NavigationServerBase*; 缺失返回 nullptr
/// (节点 onStart 判空 FAILURE, 不 abort)。
inline components::NavigationServerBase* lookupNavigationServer(
    const BT::NodeConfig& config) {
  if (!config.blackboard) {
    return nullptr;
  }
  components::NavigationServerBase* nav_server = nullptr;
  const bool found =
      config.blackboard->get<components::NavigationServerBase*>(
          kNavigationServer, nav_server);
  (void)found;
  return nav_server;
}

/**
 * @brief ComputePathToPose 异步 action (spec §6.2): TaskHandle 直连主流。
 *
 * onStart: 读 goal 端口, 组 GetPathGoal (填 server 当前 session/plan_version),
 * requestPath 持 TaskHandle, 注册 onComplete -> emitWakeUpSignal。
 * onRunning: 等 task ready; sessionIsCurrent + planVersionIsCurrent 双校验防
 * stale 竞态 (spec §6.2 关键纪律), 通过后写 path 端口 (XML 绑定 {raw_path}) 并
 * incrementPlanVersion。
 * onHalted: 取消在飞 task。
 *
 * 端口: goal={goal} (输入), path={raw_path} (输出)。
 */
class ComputePathToPoseNode : public BT::StatefulActionNode {
 public:
  ComputePathToPoseNode(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config),
      nav_server_(lookupNavigationServer(config)) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<geometry_msgs::PoseStamped>(kGoal, "navigation goal"),
      BT::OutputPort<nav_msgs::Path>("path", "planned path"),
    };
  }

  BT::NodeStatus onStart() override {
    task_ = {};
    if (!nav_server_) {
      return BT::NodeStatus::FAILURE;
    }
    geometry_msgs::PoseStamped goal;
    if (!getInput(kGoal, goal)) {
      return BT::NodeStatus::FAILURE;
    }
    actions::GetPathGoal g;
    g.goal = goal;
    g.session_id = nav_server_->currentSessionId();
    g.plan_version = nav_server_->currentPlanVersion();
    task_ = nav_server_->requestPath(g);
    if (!task_.valid()) {
      return BT::NodeStatus::FAILURE;
    }
    task_.onComplete([this] { emitWakeUpSignal(); });
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override {
    if (!task_.ready()) {
      return BT::NodeStatus::RUNNING;
    }
    const auto& r = task_.result();
    if (!nav_server_->sessionIsCurrent(r.session_id) ||
        !nav_server_->planVersionIsCurrent(r.plan_version)) {
      return BT::NodeStatus::FAILURE;
    }
    if (r.failure != FailureCode::None) {
      return BT::NodeStatus::FAILURE;  // 只依据 failure 决策, 不解析 message
    }
    setOutput("path", r.path);
    return BT::NodeStatus::SUCCESS;
  }

  void onHalted() override { task_.cancel(); }

 private:
  components::NavigationServerBase* nav_server_;
  components::TaskHandle<actions::GetPathResult> task_;
};

/**
 * @brief SmoothPath 一次性异步 action (spec §6.2 同构)。
 *
 * 端口: input_path={raw_path} (输入), path={smoothed_path} (输出)。
 */
class SmoothPathNode : public BT::StatefulActionNode {
 public:
  SmoothPathNode(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config),
      nav_server_(lookupNavigationServer(config)) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<nav_msgs::Path>("input_path", "path to smooth"),
      BT::OutputPort<nav_msgs::Path>("path", "smoothed path"),
    };
  }

  BT::NodeStatus onStart() override {
    task_ = {};
    if (!nav_server_) {
      return BT::NodeStatus::FAILURE;
    }
    nav_msgs::Path path;
    if (!getInput("input_path", path)) {
      return BT::NodeStatus::FAILURE;
    }
    actions::SmoothPathGoal g;
    g.path = path;
    g.session_id = nav_server_->currentSessionId();
    g.plan_version = nav_server_->currentPlanVersion();
    task_ = nav_server_->requestSmoothing(g);
    if (!task_.valid()) {
      return BT::NodeStatus::FAILURE;
    }
    task_.onComplete([this] { emitWakeUpSignal(); });
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override {
    if (!task_.ready()) {
      return BT::NodeStatus::RUNNING;
    }
    const auto& r = task_.result();
    if (!nav_server_->sessionIsCurrent(r.session_id) ||
        !nav_server_->planVersionIsCurrent(r.plan_version)) {
      return BT::NodeStatus::FAILURE;
    }
    if (r.failure != FailureCode::None) {
      return BT::NodeStatus::FAILURE;  // 只依据 failure 决策, 不解析 message
    }
    const bool changed = !has_last_path_ ||
                         !pathsEquivalent(last_path_, r.smoothed_path);
    setOutput("path", r.smoothed_path);
    if (changed) {
      last_path_ = r.smoothed_path;
      has_last_path_ = true;
      nav_server_->incrementPlanVersion();
    }
    return BT::NodeStatus::SUCCESS;
  }

  void onHalted() override { task_.cancel(); }

 private:
  static bool pathsEquivalent(const nav_msgs::Path& lhs,
                              const nav_msgs::Path& rhs) {
    if (lhs.header.frame_id != rhs.header.frame_id ||
        lhs.poses.size() != rhs.poses.size()) {
      return false;
    }
    constexpr double kPositionTolerance = 1e-3;
    constexpr double kRotationTolerance = 1e-6;
    for (std::size_t index = 0; index < lhs.poses.size(); ++index) {
      const auto& left = lhs.poses[index].pose;
      const auto& right = rhs.poses[index].pose;
      if ((left.translation() - right.translation()).norm() >
              kPositionTolerance ||
          (left.linear() - right.linear()).norm() > kRotationTolerance) {
        return false;
      }
    }
    return true;
  }

  components::NavigationServerBase* nav_server_;
  components::TaskHandle<actions::SmoothPathResult> task_;
  nav_msgs::Path last_path_;
  bool has_last_path_{false};
};

/**
 * @brief FollowPath 异步 action (spec §6.2 同构), 维持至 controller 到达目标。
 *
 * 端口: path={smoothed_path} (输入), frequency={controller_frequency} (输入,
 * 仅配置透传; cmd_vel 循环频率由 controller_server 自己控制 spec §6.4)。
 */
class FollowPathNode : public BT::StatefulActionNode {
 public:
  FollowPathNode(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config),
      nav_server_(lookupNavigationServer(config)) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<nav_msgs::Path>("path", "path to follow"),
      BT::InputPort<double>("frequency", 20.0, "controller frequency in Hz"),
    };
  }

  BT::NodeStatus onStart() override {
    task_ = {};
    if (!nav_server_) {
      return BT::NodeStatus::FAILURE;
    }
    nav_msgs::Path path;
    if (!getInput("path", path)) {
      return BT::NodeStatus::FAILURE;
    }
    return startExecution(path) ? BT::NodeStatus::RUNNING
                                : BT::NodeStatus::FAILURE;
  }

  BT::NodeStatus onRunning() override {
    if (nav_server_->currentPlanVersion() != submitted_plan_version_) {
      nav_msgs::Path path;
      if (!getInput("path", path)) {
        return BT::NodeStatus::FAILURE;
      }
      task_.cancel();
      if (!startExecution(path)) {
        return BT::NodeStatus::FAILURE;
      }
      return BT::NodeStatus::RUNNING;
    }
    if (!task_.ready()) {
      return BT::NodeStatus::RUNNING;
    }
    const auto& r = task_.result();
    if (!nav_server_->sessionIsCurrent(r.session_id) ||
        !nav_server_->planVersionIsCurrent(r.plan_version)) {
      return BT::NodeStatus::FAILURE;
    }
    // controller 成功且到达目标才算完成 (ARRIVED_GOAL 语义); 只依据 failure, 不解析 message
    if (r.failure != FailureCode::None || !r.reached) {
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
  }

  void onHalted() override { task_.cancel(); }

 private:
  bool startExecution(const nav_msgs::Path& path) {
    actions::ExePathGoal goal;
    goal.path = path;
    goal.session_id = nav_server_->currentSessionId();
    goal.plan_version = nav_server_->currentPlanVersion();
    submitted_plan_version_ = goal.plan_version;
    task_ = nav_server_->requestExecution(goal);
    if (!task_.valid()) {
      return false;
    }
    task_.onComplete([this] { emitWakeUpSignal(); });
    return true;
  }

  components::NavigationServerBase* nav_server_;
  components::TaskHandle<actions::ExePathResult> task_;
  uint64_t submitted_plan_version_{0};
};

/**
 * @brief Recovery 一次性异步 action (spec §6.2 同构)。
 *
 * 端口: recovery={recovery_id}/{clear_costmap_id}/{spin_id}/{wait_id}/{backup_id}
 * (输入, XML `<Recovery recovery="{...}"/>`; 黑板键由 pilot 配置决定)。
 */
class RecoveryActionNode : public BT::StatefulActionNode {
 public:
  RecoveryActionNode(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config),
      nav_server_(lookupNavigationServer(config)) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<std::string>("recovery", "recovery plugin id"),
    };
  }

  BT::NodeStatus onStart() override {
    task_ = {};
    if (!nav_server_) {
      return BT::NodeStatus::FAILURE;
    }
    std::string recovery_id;
    if (!getInput("recovery", recovery_id)) {
      return BT::NodeStatus::FAILURE;
    }
    actions::RecoveryGoal g;
    g.recovery_id = recovery_id;
    g.session_id = nav_server_->currentSessionId();
    g.plan_version = nav_server_->currentPlanVersion();
    task_ = nav_server_->requestRecovery(g);
    if (!task_.valid()) {
      return BT::NodeStatus::FAILURE;
    }
    task_.onComplete([this] { emitWakeUpSignal(); });
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override {
    if (!task_.ready()) {
      return BT::NodeStatus::RUNNING;
    }
    const auto& r = task_.result();
    if (!nav_server_->sessionIsCurrent(r.session_id) ||
        !nav_server_->planVersionIsCurrent(r.plan_version)) {
      return BT::NodeStatus::FAILURE;
    }
    return r.failure == FailureCode::None ? BT::NodeStatus::SUCCESS
                                          : BT::NodeStatus::FAILURE;
  }

  void onHalted() override { task_.cancel(); }

 private:
  components::NavigationServerBase* nav_server_;
  components::TaskHandle<actions::RecoveryResult> task_;
};

} // namespace behavior_tree
} // namespace ayanami
