// navigation_behavior_tree extension module header.
//
// The default navigation tree (XML) + all custom navigation BT node registrations sink to this module: the core
// BehaviorTreePilot contains no default navigation XML/policies; it only loads and holds this extension.

#pragma once

#include "ayanami/behavior_tree/extension.hpp"
#include "ayanami/behavior_tree/navigation.hpp"

#include <string>
#include <vector>

namespace ayanami {
namespace modules {

/**
 * @brief Default navigation BT extension (derives BehaviorTreeExtension).
 *
 * - registerNodes: registers all custom navigation nodes (ComputePathToPose / SmoothPath /
 *   FollowPath / Recovery / conditions / control nodes, via the core registerNavigationBehaviorTreeNodes).
 * - registerTrees: registers the navigation tree XML. The XML is a config
 *   asset (navigation.behavior_tree_xml in the navigation config, resolved
 *   relative to the config directory); the module keeps the same tree
 *   embedded as the fallback when the asset is absent, so the extension
 *   alone is enough to run.
 * - defaultTreeId: "NavigateWithReplanningAndSmoothing".
 */
class NavigationBehaviorTreeExtension
    final : public ayanami::behavior_tree::BehaviorTreeExtension {
 public:
  std::string name() const override { return "navigation"; }

  void registerNodes(BT::BehaviorTreeFactory& factory) const override;

  std::vector<std::string> registerTrees(
      BT::BehaviorTreeFactory& factory) const override;

  std::string defaultTreeId() const override { return k_default_tree_id; }

  // Recovery role ids are a tree-level contract (this tree references them
  // by role), declared and provided here -- the core pilot stays agnostic
  // of what the recoveries do.
  std::vector<const char*> requiredBlackboardEntries() const override;

  void provideBlackboardEntries(BT::Blackboard::Ptr blackboard) const override;

 private:
  static constexpr const char* k_default_tree_id =
      "NavigateWithReplanningAndSmoothing";
  static constexpr const char* k_default_tree_xml = R"(
<root BTCPP_format="4" main_tree_to_execute="NavigateWithReplanningAndSmoothing">
  <BehaviorTree ID="NavigateWithReplanningAndSmoothing">
    <RecoveryNode name="NavigateRecovery" number_of_retries="6" budget_reset_token="{goal_revision}">
      <PipelineSequence name="NavigateWithReplanning">
        <RateController hz="{planner_frequency}" reset_token="{goal_revision}">
          <RecoveryNode name="ComputePath" number_of_retries="1">
            <Sequence>
               <ComputePathToPose goal="{goal}" path="{raw_path}" path_id="{path_id}"/>
              <IsPathValid path="{raw_path}"/>
            </Sequence>
            <Sequence>
              <Recovery recovery="{clear_costmap_id}"/>
            </Sequence>
          </RecoveryNode>
        </RateController>
        <RateController hz="{smoother_frequency}" reset_token="{path_id}">
               <SmoothPath input_path="{raw_path}" path="{smoothed_path}"/>
        </RateController>
        <RecoveryNode name="FollowPath" number_of_retries="1" budget_reset_token="{goal_revision}">
           <FollowPath path="{smoothed_path}" frequency="{controller_frequency}"/>
          <Sequence>
            <Recovery recovery="{clear_costmap_id}"/>
          </Sequence>
        </RecoveryNode>
      </PipelineSequence>
      <ReactiveFallback name="RecoveryActions">
        <GoalUpdated goal="{goal}"/>
         <RoundRobin name="RecoveryActions" reset_token="{goal_revision}">
          <Sequence>
            <Recovery recovery="{spin_id}"/>
          </Sequence>
          <Sequence>
            <Recovery recovery="{wait_id}"/>
          </Sequence>
          <Sequence>
            <Recovery recovery="{backup_id}"/>
          </Sequence>
         </RoundRobin>
      </ReactiveFallback>
    </RecoveryNode>
  </BehaviorTree>
</root>)";
};

}  // namespace modules
}  // namespace ayanami
