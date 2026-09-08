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
 * - registerTrees: registers the embedded default navigation tree XML (the single production source; the original
 *   config/BT/navigation.xml file was deleted).
 * - defaultTreeId: "NavigateWithReplanningAndSmoothing".
 */
class NavigationBehaviorTreeExtension
    final : public ayanami::behavior_tree::BehaviorTreeExtension {
 public:
  std::string name() const override { return "navigation"; }

  void registerNodes(BT::BehaviorTreeFactory& factory) const override {
    ayanami::behavior_tree::registerNavigationBehaviorTreeNodes(factory);
  }

  std::vector<std::string> registerTrees(
      BT::BehaviorTreeFactory& factory) const override {
    factory.registerBehaviorTreeFromText(k_default_navigation_xml);
    return {k_default_tree_id};
  }

  std::string defaultTreeId() const override { return k_default_tree_id; }

  // Recovery role ids are a tree-level contract (this tree references them
  // by role), declared and provided here -- the core pilot stays agnostic
  // of what the recoveries do.
  std::vector<const char*> requiredBlackboardEntries() const override;

  void provideBlackboardEntries(BT::Blackboard::Ptr blackboard) const override;

 private:
  // Embedded default navigation tree (verbatim, single production source; the file version is not installed in production).
  //
  // Goal update discipline: the planner branch uses {goal_revision} as reset_token (a new goal triggers
  // replanning); the smoother branch uses {path_id} as reset_token (ComputePathToPose bumps path_id on each
  // successful plan, so a newly landed path is re-smoothed + bumps plan_version); FollowPath performs an in-place updatePlan on the running controller
  // when plan_version changes -- the robot keeps following the old path until the new one is available, avoiding a stale
  // smoothing cache where "replanning happened but is not being followed".
 //
 // Recovery rotation discipline (nav2 aligned): the outer recovery branch uses RoundRobin instead of sequential retry -- the cursor
 // keeps advancing across successes, rotating retries spin->wait->backup->spin..., avoiding a single recovery running repeatedly
 // (with sequential retries, a first recovery's "false success" would starve all later options forever).
  static constexpr const char* k_default_tree_id =
      "NavigateWithReplanningAndSmoothing";
  static constexpr const char* k_default_navigation_xml = R"(
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
