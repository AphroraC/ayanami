#pragma once

#include <behaviortree_cpp/control_node.h>

#include <cstddef>

namespace ayanami {
namespace behavior_tree {

/**
 * @brief nav2 风格 PipelineSequence: 带记忆的 Sequence。
 *
 * - child_i RUNNING 时下次 tick 仍 tick child_i;
 * - child_i SUCCESS 后在同一 tick 内继续向后推进 (child_{i+1} ...);
 * - 任一 child FAILURE -> halt 全部 children 并整体 FAILURE;
 * - 全部 SUCCESS -> 复位回开头并返回 SUCCESS。
 */
class PipelineSequence : public BT::ControlNode {
 public:
  PipelineSequence(const std::string& name, const BT::NodeConfig& config)
    : BT::ControlNode(name, config), current_child_idx_(0) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override {
    while (current_child_idx_ < childrenCount()) {
      const BT::NodeStatus status = children_nodes_[current_child_idx_]->executeTick();
      if (status == BT::NodeStatus::SUCCESS) {
        ++current_child_idx_;
        if (current_child_idx_ == childrenCount()) {
          resetChildren();
          current_child_idx_ = 0;
          return BT::NodeStatus::SUCCESS;
        }
      } else if (status == BT::NodeStatus::FAILURE) {
        haltChildren();
        current_child_idx_ = 0;
        return BT::NodeStatus::FAILURE;
      } else {
        // RUNNING (或 SKIPPED): 停在当前 child, 下次 tick 继续
        return status;
      }
    }
    resetChildren();
    current_child_idx_ = 0;
    return BT::NodeStatus::SUCCESS;
  }

  void halt() override {
    BT::ControlNode::halt();
    current_child_idx_ = 0;
  }

 private:
  size_t current_child_idx_;
};

} // namespace behavior_tree
} // namespace ayanami
