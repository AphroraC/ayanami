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
 * @brief RoundRobin (spec §6.3 保留节点): 按记忆游标轮询 children。
 *
 * - 当前 child RUNNING -> 整体 RUNNING, 游标停住 (下次 tick 仍 tick 它);
 * - 当前 child SUCCESS -> 游标移到下一个 (mod 数量) 并整体 SUCCESS;
 * - 当前 child FAILURE -> 移到下一个, 全部试过仍 FAILURE -> 整体 FAILURE。
 */
class RoundRobin : public BT::ControlNode {
 public:
  RoundRobin(const std::string& name, const BT::NodeConfig& config)
    : BT::ControlNode(name, config), current_child_idx(0) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override {
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

 private:
  size_t current_child_idx;
};

/// @brief 注册导航 BT 节点类 (spec §6.3 保留节点 + Task 12 控制/条件节点)。
/// action 节点经 blackboard 注入 NavigationRuntime::Ptr (action_nodes.hpp)。
///
/// 注意: 默认导航树的 XML 与整棵树的注册已下沉到 navigation_behavior_tree
/// 扩展模块 (BehaviorTreeExtension); 本函数只注册节点类型, 供扩展模块
/// registerNodes 与测试复用。核心 (BehaviorTreePilot) 不含默认导航 XML 或策略。
inline void registerNavigationBehaviorTreeNodes(BT::BehaviorTreeFactory& factory) {
  // 4 个 action 节点 (TaskHandle 直连, spec §6.2)
  factory.registerNodeType<ComputePathToPoseNode>("ComputePathToPose");
  factory.registerNodeType<SmoothPathNode>("SmoothPath");
  factory.registerNodeType<FollowPathNode>("FollowPath");
  factory.registerNodeType<RecoveryActionNode>("Recovery");
  // recovery 前置暂停节点 (spec: recovery 暂停 controller 由 BT 实现)
  factory.registerNodeType<HaltControllersNode>("HaltControllers");
  // 5 个条件节点 (Task 12)
  factory.registerNodeType<IsPathValid>("IsPathValid");
  factory.registerNodeType<IsGoalReached>("IsGoalReached");
  factory.registerNodeType<IsFarFromGoal>("IsFarFromGoal");
  factory.registerNodeType<IsRobotStuck>("IsRobotStuck");
  factory.registerNodeType<GoalUpdated>("GoalUpdated");
  // 控制节点 (Task 12 + RoundRobin)
  factory.registerNodeType<PipelineSequence>("PipelineSequence");
  factory.registerNodeType<RateController>("RateController");
  factory.registerNodeType<RecoveryNode>("RecoveryNode");
  factory.registerNodeType<RoundRobin>("RoundRobin");
}

} // namespace behavior_tree
} // namespace ayanami
