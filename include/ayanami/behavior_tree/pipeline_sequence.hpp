#pragma once

#include <behaviortree_cpp/control_node.h>

#include "ayanami/behavior_tree/navigation_node_state.hpp"

#include <cstddef>
#include <stdexcept>

namespace ayanami {
namespace behavior_tree {

/**
 * @brief nav2-style PipelineSequence.
 *
 * - Restarts from child_0 each tick so the planning branch keeps being
 *   ticked while the controller is RUNNING;
 * - When an earlier child is RUNNING and progress previously reached a later
 *   child, keeps ticking the later pipeline stages;
 * - Any child FAILURE -> halt all children and return FAILURE;
 * - All SUCCESS -> reset back to the beginning and return SUCCESS.
 */
class PipelineSequence : public BT::ControlNode,
                         public NavigationTreeNodeState {
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

  /// @brief Logical reset after a tree-life terminal state (v4 halt only
  /// propagates to RUNNING children, see the
  /// RateController::resetState comment).
  void resetState() { current_child_idx = 0; }

 private:
  size_t current_child_idx;
};

} // namespace behavior_tree
} // namespace ayanami
