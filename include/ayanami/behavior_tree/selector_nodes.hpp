#pragma once

#include <behaviortree_cpp/action_node.h>

#include <spdlog/spdlog.h>

#include <string>

namespace ayanami {
namespace behavior_tree {

/**
 * @brief nav2-style module selector (PlannerSelector/ControllerSelector/
 * SmootherSelector are the same class registered under three names).
 *
 * Mirrors nav2's selector semantics minus the ROS topic: the selected module
 * id is published to the output port each tick. The value comes from
 * (in priority):
 * 1. the input port `override_id` when it is bound to a blackboard entry
 *    that exists and is non-empty (a host/extension/test can hot-switch the
 *    module by writing that blackboard key externally, which is the
 *    ROS-free analogue of nav2's selector topic);
 * 2. the `default_id` input port (typically a literal in the tree XML).
 * A change of the effective selection is logged once.
 *
 * Usage (nav2 default-tree shape):
 *   <PlannerSelector default_id="costmap_astar" override_id="{planner_override}"
 *                    selected_id="{planner_id}"/>
 *   ...
 *   <ComputePathToPose goal="{goal}" planner_id="{planner_id}" .../>
 *
 * Deployment note (documented by design, not a wiring bug): the production
 * tree (config/BT/navigation.xml) does NOT instantiate selector nodes.
 * Module selection there happens at the REQUEST level instead -- the
 * action nodes' *_id ports (planner_id/controller_id/smoother_id) and the
 * recovery role ids ({clear_costmap_id} etc., provided by the BT extension
 * from config_recovery). Selectors are the tree-level alternative for
 * deployments that want the choice to live in the XML/blackboard; request
 * -level ids are the lighter default (empty id = family default = first
 * configured module).
 */
class ModuleSelectorNode : public BT::SyncActionNode {
 public:
  ModuleSelectorNode(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<std::string>("default_id", "", "fallback module id"),
      BT::InputPort<std::string>("override_id", "", "optional blackboard key an external writer updates to hot-switch"),
      BT::OutputPort<std::string>("selected_id", "the effective module id"),
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
      spdlog::info("[bt] {} selected module id '{}'", name(), selected);
    }
    setOutput("selected_id", selected);
    return BT::NodeStatus::SUCCESS;
  }

 private:
  std::string last_selected;
};

}  // namespace behavior_tree
}  // namespace ayanami
