#pragma once

#include <behaviortree_cpp/control_node.h>

#include <cstdint>

namespace ayanami {
namespace behavior_tree {

/**
 * @brief nav2 风格 RecoveryNode: 2 个 child, child0 为主任务, child1 为恢复。
 *
 * - child0 SUCCESS -> 复位 retry 计数并整体 SUCCESS;
 * - child0 FAILURE 且仍有 retries -> halt child0, tick child1;
 * - child1 SUCCESS -> retries++, 复位 child0 回主任务, 返回 RUNNING 让主任务重试;
 * - child1 FAILURE -> 整体 FAILURE;
 * - child0 FAILURE 且 retries 耗尽 -> 整体 FAILURE。
 */
class RecoveryNode : public BT::ControlNode {
 public:
  RecoveryNode(const std::string& name, const BT::NodeConfig& config)
    : BT::ControlNode(name, config),
      max_retries_(1),
      retry_count_(0),
      recovering_(false) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<uint32_t>("number_of_retries", 1u, "number of retries"),
    };
  }

  BT::NodeStatus tick() override {
    if (!getInput("number_of_retries", max_retries_)) {
      return BT::NodeStatus::FAILURE;
    }

    if (!recovering_) {
      const BT::NodeStatus status = children_nodes_[0]->executeTick();
      if (status == BT::NodeStatus::SUCCESS) {
        retry_count_ = 0;
        resetChildren();
        return BT::NodeStatus::SUCCESS;
      }
      if (status == BT::NodeStatus::RUNNING) {
        return BT::NodeStatus::RUNNING;
      }
      // child0 FAILURE
      if (retry_count_ >= max_retries_) {
        haltChildren();
        retry_count_ = 0;
        return BT::NodeStatus::FAILURE;
      }
      haltChild(0);
      recovering_ = true;
    }

    const BT::NodeStatus status = children_nodes_[1]->executeTick();
    if (status == BT::NodeStatus::SUCCESS) {
      ++retry_count_;
      recovering_ = false;
      resetChildren();
      return BT::NodeStatus::RUNNING;
    }
    if (status == BT::NodeStatus::RUNNING) {
      return BT::NodeStatus::RUNNING;
    }
    // child1 FAILURE
    haltChildren();
    retry_count_ = 0;
    recovering_ = false;
    return BT::NodeStatus::FAILURE;
  }

  void halt() override {
    BT::ControlNode::halt();
    retry_count_ = 0;
    recovering_ = false;
  }

 private:
  uint32_t max_retries_;
  uint32_t retry_count_;
  bool recovering_;
};

} // namespace behavior_tree
} // namespace ayanami
