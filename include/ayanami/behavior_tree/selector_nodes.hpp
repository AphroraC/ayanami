#pragma once

#include <behaviortree_cpp/action_node.h>

#include <spdlog/spdlog.h>

#include <string>

namespace ayanami {
namespace behavior_tree {

/**
 * @brief nav2-style plugin selector (PlannerSelector/ControllerSelector/
 * SmootherSelector are the same class registered under three names).
 *
 * Mirrors nav2's selector semantics minus the ROS topic: the selected plugin
 * id is published to the output port each tick. The value comes from
 * (in priority):
 * 1. the input port `override_id` when it is bound to a blackboard entry
 *    that exists and is non-empty (a host/extension/test can hot-switch the
 *    plugin by writing that blackboard key externally, which is the
 *    ROS-free analogue of nav2's selector topic);
 * 2. the `default_id` input port (typically a literal in the tree XML).
 * A change of the effective selection is logged once.
 *
 * Usage (nav2 default-tree shape):
 *   <PlannerSelector default_id="costmap_smac" override_id="{planner_override}"
 *                    selected_id="{planner_id}"/>
 *   ...
 *   <ComputePathToPose goal="{goal}" planner_id="{planner_id}" .../>
 */
class PluginSelectorNode : public BT::SyncActionNode {
 public:
  PluginSelectorNode(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<std::string>("default_id", "", "fallback plugin id"),
      BT::InputPort<std::string>("override_id", "", "optional blackboard key an external writer updates to hot-switch"),
      BT::OutputPort<std::string>("selected_id", "the effective plugin id"),
    };
  }

  BT::NodeStatus tick() override {
    std::string selected;
    std::string override_id;
    if (getInput("override_id", override_id) && !override_id.empty()) {
      selected = override_id;
    } else {
      (void)getInput("default_id", selected);
    }
    if (selected != last_selected) {
      last_selected = selected;
      spdlog::info("[bt] {} selected plugin id '{}'", name(), selected);
    }
    setOutput("selected_id", selected);
    return BT::NodeStatus::SUCCESS;
  }

 private:
  std::string last_selected;
};

}  // namespace behavior_tree
}  // namespace ayanami
