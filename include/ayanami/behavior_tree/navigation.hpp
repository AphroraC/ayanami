#pragma once

#include "ayanami/behavior_tree/action_nodes.hpp"
#include "ayanami/behavior_tree/blackboard_protocol.hpp"
#include "ayanami/behavior_tree/condition_nodes.hpp"
#include "ayanami/behavior_tree/pipeline_sequence.hpp"
#include "ayanami/behavior_tree/rate_controller.hpp"
#include "ayanami/behavior_tree/recovery_node.hpp"

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/control_node.h>

#include <cstddef>
#include <string>

namespace ayanami {
namespace behavior_tree {

/**
 * @brief RoundRobin (保留节点): 按记忆游标轮询 children。
 *
 * - 当前 child RUNNING -> 整体 RUNNING, 游标停住 (下次 tick 仍 tick 它);
 * - 当前 child SUCCESS -> 游标移到下一个 (mod 数量) 并整体 SUCCESS;
 * - 当前 child FAILURE -> 移到下一个, 全部试过仍 FAILURE -> 整体 FAILURE。
 *
 * reset_token (常驻树复位协议): 令牌变化 (新导航轮 goal_revision 推进) 时
 * 复位游标并 halt 在飞 child —— 跨 goal 不继承上一轮的恢复轮询位置
 * (nav2 靠整树重建天然规避; 常驻树必须显式复位)。
 */
class RoundRobin : public BT::ControlNode {
 public:
  RoundRobin(const std::string& name, const BT::NodeConfig& config)
    : BT::ControlNode(name, config), current_child_idx(0) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<uint64_t>("reset_token", 0,
                              "revision that resets the round-robin cursor"),
    };
  }

  BT::NodeStatus tick() override {
    uint64_t token = 0;
    (void)getInput("reset_token", token);
    if (token != reset_token) {
      reset_token = token;
      haltChildren();  // 取消在飞 recovery (IDLE child 为 no-op)
      current_child_idx = 0;
    }
    if (childrenCount() == 0) {
      return BT::NodeStatus::FAILURE;
    }
    while (current_child_idx < childrenCount()) {
      const BT::NodeStatus status =
          children_nodes_[current_child_idx]->executeTick();
      if (status == BT::NodeStatus::RUNNING) {
        return status;
      }
      if (status == BT::NodeStatus::SUCCESS) {
        current_child_idx = (current_child_idx + 1) % childrenCount();
        resetChildren();
        return BT::NodeStatus::SUCCESS;
      }
      // FAILURE -> 下一 child
      ++current_child_idx;
    }
    current_child_idx = 0;
    return BT::NodeStatus::FAILURE;
  }

  void halt() override {
    BT::ControlNode::halt();
    current_child_idx = 0;
  }

  /// @brief 树生命终端后的逻辑重置 (v4 halt 只传播到 RUNNING child)。
  void resetState() { current_child_idx = 0; }

 private:
  size_t current_child_idx;
  uint64_t reset_token{0};
};

/// Tries recovery children in declaration order. A successful recovery returns
/// SUCCESS to its parent so the surrounding RecoveryNode retries navigation;
/// a failed recovery advances to the next child.
///
/// reset_token (常驻树复位协议): 令牌变化时复位游标并 halt 在飞 child,
/// 新导航轮从第一个 recovery 重新开始 (不继承上轮游标)。
class RecoverySequence : public BT::ControlNode {
 public:
  RecoverySequence(const std::string& name, const BT::NodeConfig& config)
      : BT::ControlNode(name, config), current_child_idx(0) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<uint64_t>("reset_token", 0,
                              "revision that resets the recovery cursor"),
    };
  }

  BT::NodeStatus tick() override {
    uint64_t token = 0;
    (void)getInput("reset_token", token);
    if (token != reset_token) {
      reset_token = token;
      haltChildren();
      current_child_idx = 0;
    }
    while (current_child_idx < childrenCount()) {
      const auto status = children_nodes_[current_child_idx]->executeTick();
      if (status == BT::NodeStatus::RUNNING) return status;
      if (status == BT::NodeStatus::SUCCESS) {
        haltChildren();
        current_child_idx = 0;
        return BT::NodeStatus::SUCCESS;
      }
      if (status != BT::NodeStatus::FAILURE) return status;
      children_nodes_[current_child_idx]->haltNode();
      ++current_child_idx;
    }
    haltChildren();
    current_child_idx = 0;
    return BT::NodeStatus::FAILURE;
  }

  void halt() override {
    BT::ControlNode::halt();
    current_child_idx = 0;
  }

  /// @brief 树生命终端后的逻辑重置 (v4 halt 只传播到 RUNNING child)。
  void resetState() { current_child_idx = 0; }

 private:
  std::size_t current_child_idx;
  uint64_t reset_token{0};
};

/// @brief 树生命终端后的全量逻辑重置 (pilot 在 tick 返回非 RUNNING 时调用)。
///
/// BT.CPP v4 的 halt 语义变更: ControlNode::haltChildren 只对 status==RUNNING
/// 的 child 调 haltNode, 终态 (SUCCESS/FAILURE) 缓存的子树不会被重置 —— 常驻
/// 树模型下 planner/smoother 的 RateController 缓存、RecoveryNode 重试计数、
/// RecoverySequence/RoundRobin 游标会跨树生命残留, 造成 "planner 永不重规划 +
/// 恢复风暴" (rviz 实测 rosbag 取证)。nav2 (BT.CPP v3) 的强 halt 语义等价物
/// 即本函数: 逐节点重置内部状态, 不触碰 BT 状态机 (executeTick 已 auto-reset)。
inline void resetNavigationTreeState(BT::Tree& tree) {
  tree.applyVisitor([](BT::TreeNode* node) {
    if (auto* n = dynamic_cast<RateController*>(node)) {
      n->resetState();
    } else if (auto* n = dynamic_cast<PipelineSequence*>(node)) {
      n->resetState();
    } else if (auto* n = dynamic_cast<RecoveryNode*>(node)) {
      n->resetState();
    } else if (auto* n = dynamic_cast<RecoverySequence*>(node)) {
      n->resetState();
    } else if (auto* n = dynamic_cast<RoundRobin*>(node)) {
      n->resetState();
    }
  });
}

/// @brief 注册导航 BT 节点类 (保留节点 + 控制/条件节点)。
/// action 节点经 blackboard 注入 NavigationRuntime::Ptr (action_nodes.hpp)。
///
/// 注意: 默认导航树的 XML 与整棵树的注册已下沉到 navigation_behavior_tree
/// 扩展模块 (BehaviorTreeExtension); 本函数只注册节点类型, 供扩展模块
/// registerNodes 与测试复用。核心 (BehaviorTreePilot) 不含默认导航 XML 或策略。
inline void registerNavigationBehaviorTreeNodes(BT::BehaviorTreeFactory& factory) {
  // 4 个 action 节点 (TaskHandle 直连)
  factory.registerNodeType<ComputePathToPoseNode>("ComputePathToPose");
  factory.registerNodeType<SmoothPathNode>("SmoothPath");
  factory.registerNodeType<FollowPathNode>("FollowPath");
  factory.registerNodeType<RecoveryActionNode>("Recovery");
  // 5 个条件节点
  factory.registerNodeType<IsPathValid>("IsPathValid");
  factory.registerNodeType<IsGoalReached>("IsGoalReached");
  factory.registerNodeType<IsFarFromGoal>("IsFarFromGoal");
  factory.registerNodeType<IsRobotStuck>("IsRobotStuck");
  factory.registerNodeType<GoalUpdated>("GoalUpdated");
  // 控制节点 (+ RoundRobin)
  factory.registerNodeType<PipelineSequence>("PipelineSequence");
  factory.registerNodeType<RateController>("RateController");
  factory.registerNodeType<RecoveryNode>("RecoveryNode");
  factory.registerNodeType<RoundRobin>("RoundRobin");
  factory.registerNodeType<RecoverySequence>("RecoverySequence");
}

} // namespace behavior_tree
} // namespace ayanami
