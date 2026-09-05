#pragma once

#include <behaviortree_cpp/control_node.h>

#include <cstddef>
#include <stdexcept>

namespace ayanami {
namespace behavior_tree {

/**
 * @brief nav2 风格 PipelineSequence。
 *
 * - 每次从 child_0 开始，使规划分支在 controller RUNNING 时继续被 tick;
 * - 前序 child RUNNING 且以前已推进到更后 child 时，继续 tick 后续流水线;
 * - 任一 child FAILURE -> halt 全部 children 并整体 FAILURE;
 * - 全部 SUCCESS -> 复位回开头并返回 SUCCESS。
 */
class PipelineSequence : public BT::ControlNode {
 public:
  PipelineSequence(const std::string& name, const BT::NodeConfig& config)
    : BT::ControlNode(name, config), current_child_idx(0) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override {
    for (std::size_t index = 0; index < childrenCount(); ++index) {
      const BT::NodeStatus status = children_nodes_[index]->executeTick();
      if (status == BT::NodeStatus::FAILURE) {
        haltChildren();
        current_child_idx = 0;
        return status;
      }
      if (status == BT::NodeStatus::RUNNING) {
        if (index >= current_child_idx) {
          current_child_idx = index;
          return status;
        }
        continue;
      }
      if (status != BT::NodeStatus::SUCCESS) {
        throw std::runtime_error("PipelineSequence child returned invalid status");
      }
    }
    haltChildren();
    current_child_idx = 0;
    return BT::NodeStatus::SUCCESS;
  }

  void halt() override {
    BT::ControlNode::halt();
    current_child_idx = 0;
  }

  /// @brief 树生命终端后的逻辑重置 (v4 halt 只传播到 RUNNING child, 见
  /// RateController::resetState 注释)。
  void resetState() { current_child_idx = 0; }

 private:
  size_t current_child_idx;
};

} // namespace behavior_tree
} // namespace ayanami
