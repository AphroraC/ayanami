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
 * @brief nav2 风格 RoundRobin (spec §6.3 保留节点): 按记忆游标轮询 children。
 *
 * - 当前 child RUNNING -> 整体 RUNNING, 游标停住 (下次 tick 仍 tick 它);
 * - 当前 child SUCCESS -> 游标移到下一个 (mod 数量) 并整体 SUCCESS;
 * - 当前 child FAILURE -> 移到下一个, 全部试过仍 FAILURE -> 整体 FAILURE。
 */
class RoundRobin : public BT::ControlNode {
 public:
  RoundRobin(const std::string& name, const BT::NodeConfig& config)
    : BT::ControlNode(name, config), current_child_idx_(0) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override {
    if (childrenCount() == 0) {
      return BT::NodeStatus::FAILURE;
    }
    while (current_child_idx_ < childrenCount()) {
      const BT::NodeStatus status =
          children_nodes_[current_child_idx_]->executeTick();
      if (status == BT::NodeStatus::RUNNING) {
        return status;
      }
      if (status == BT::NodeStatus::SUCCESS) {
        current_child_idx_ = (current_child_idx_ + 1) % childrenCount();
        resetChildren();
        return BT::NodeStatus::SUCCESS;
      }
      // FAILURE -> 下一 child
      ++current_child_idx_;
    }
    current_child_idx_ = 0;
    return BT::NodeStatus::FAILURE;
  }

  void halt() override {
    BT::ControlNode::halt();
    current_child_idx_ = 0;
  }

 private:
  size_t current_child_idx_;
};

// 内嵌默认导航树 (spec §6.3 verbatim), 与 config/BT/navigation.xml 同构。
inline constexpr const char* kDefaultNavigationXml = R"(
<root BTCPP_format="4" main_tree_to_execute="NavigateWithReplanningAndSmoothing">
  <BehaviorTree ID="NavigateWithReplanningAndSmoothing">
    <RecoveryNode name="NavigateRecovery" number_of_retries="6">
      <PipelineSequence name="NavigateWithReplanning">
        <RateController hz="{planner_frequency}">
          <RecoveryNode name="ComputePath" number_of_retries="1">
            <Sequence>
              <ComputePathToPose goal="{goal}" path="{raw_path}"/>
              <IsPathValid path="{raw_path}"/>
              <SmoothPath  input_path="{raw_path}" path="{smoothed_path}"/>
              <IsPathValid path="{smoothed_path}"/>
            </Sequence>
            <Sequence>
              <Recovery recovery="{clear_costmap_id}"/>
              <AlwaysFailure/>
            </Sequence>
          </RecoveryNode>
        </RateController>
        <RecoveryNode name="FollowPath" number_of_retries="1">
          <FollowPath path="{smoothed_path}" frequency="{controller_frequency}"/>
          <Sequence>
            <Recovery recovery="{clear_costmap_id}"/>
            <AlwaysFailure/>
          </Sequence>
        </RecoveryNode>
      </PipelineSequence>
      <ReactiveFallback name="RecoveryActions">
        <GoalUpdated goal="{goal}"/>
        <RoundRobin name="RecoveryActions">
          <Sequence><Recovery recovery="{spin_id}"/>    <AlwaysFailure/></Sequence>
          <Sequence><Recovery recovery="{wait_id}"/>    <AlwaysFailure/></Sequence>
          <Sequence><Recovery recovery="{backup_id}"/>  <AlwaysFailure/></Sequence>
        </RoundRobin>
      </ReactiveFallback>
    </RecoveryNode>
  </BehaviorTree>
</root>)";

/// @brief 注册全部导航 BT 节点 (spec §6.3 保留节点 + Task 12 控制/条件节点)。
/// action 节点经 blackboard 注入 NavigationServerBase* (action_nodes.hpp)。
inline void registerNavigationBehaviorTreeNodes(BT::BehaviorTreeFactory& factory) {
  // 4 个 action 节点 (TaskHandle 直连, spec §6.2)
  factory.registerNodeType<ComputePathToPoseNode>("ComputePathToPose");
  factory.registerNodeType<SmoothPathNode>("SmoothPath");
  factory.registerNodeType<FollowPathNode>("FollowPath");
  factory.registerNodeType<RecoveryActionNode>("Recovery");
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

/// @brief 用内嵌默认 XML 建默认导航树。blackboard 需先注入 NavigationServerBase*
/// (kNavigationServer) 与 16 个必需板项; tree_id 为空时用默认树名。
inline BT::Tree createDefaultNavigationTree(
    BT::Blackboard::Ptr blackboard,
    const std::string& tree_id = "NavigateWithReplanningAndSmoothing") {
  BT::BehaviorTreeFactory factory;
  registerNavigationBehaviorTreeNodes(factory);
  factory.registerBehaviorTreeFromText(kDefaultNavigationXml);
  return factory.createTree(tree_id.empty()
                                ? "NavigateWithReplanningAndSmoothing"
                                : tree_id,
                            blackboard);
}

} // namespace behavior_tree
} // namespace ayanami
