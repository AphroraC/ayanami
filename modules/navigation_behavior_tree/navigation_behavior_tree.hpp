// navigation_behavior_tree 扩展模块头 (requirement 3)。
//
// 默认导航树 (XML) + 全部自定义导航 BT 节点注册下沉到这里: 核心
// BehaviorTreePilot 不含任何默认导航 XML/策略, 只装载并持有本扩展。

#pragma once

#include "ayanami/behavior_tree/extension.hpp"
#include "ayanami/behavior_tree/navigation.hpp"

#include <string>
#include <vector>

namespace ayanami {
namespace modules {

/**
 * @brief 默认导航 BT 扩展 (BehaviorTreeExtension 派生)。
 *
 * - registerNodes: 注册全部自定义导航节点 (ComputePathToPose / SmoothPath /
 *   FollowPath / Recovery / 条件 / 控制节点, 经核心 registerNavigationBehaviorTreeNodes)。
 * - registerTrees: 注册内嵌默认导航树 XML (生产唯一来源; 原 config/BT/navigation.xml
 *   文件已删除)。
 * - defaultTreeId: "NavigateWithReplanningAndSmoothing"。
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

 private:
  // 内嵌默认导航树 (spec §6.3 verbatim, 生产单一来源; 文件版不在生产安装)。
  static constexpr const char* k_default_tree_id =
      "NavigateWithReplanningAndSmoothing";
  static constexpr const char* k_default_navigation_xml = R"(
<root BTCPP_format="4" main_tree_to_execute="NavigateWithReplanningAndSmoothing">
  <BehaviorTree ID="NavigateWithReplanningAndSmoothing">
    <RecoveryNode name="NavigateRecovery" number_of_retries="6">
      <PipelineSequence name="NavigateWithReplanning">
        <RateController hz="{planner_frequency}" reset_token="{goal_revision}">
          <RecoveryNode name="ComputePath" number_of_retries="1">
            <Sequence>
               <ComputePathToPose goal="{goal}" path="{raw_path}"/>
              <IsPathValid path="{raw_path}"/>
            </Sequence>
            <Sequence>
              <Recovery recovery="{clear_costmap_id}"/>
            </Sequence>
          </RecoveryNode>
        </RateController>
        <RateController hz="{smoother_frequency}" reset_token="{goal_revision}">
               <SmoothPath input_path="{raw_path}" path="{smoothed_path}"/>
        </RateController>
        <RecoveryNode name="FollowPath" number_of_retries="1">
           <FollowPath path="{smoothed_path}" frequency="{controller_frequency}"/>
          <Sequence>
            <Recovery recovery="{clear_costmap_id}"/>
          </Sequence>
        </RecoveryNode>
      </PipelineSequence>
      <ReactiveFallback name="RecoveryActions">
        <GoalUpdated goal="{goal}"/>
         <RecoverySequence name="RecoveryActions">
          <Sequence>
            <Recovery recovery="{spin_id}"/>
          </Sequence>
          <Sequence>
            <Recovery recovery="{wait_id}"/>
          </Sequence>
          <Sequence>
            <Recovery recovery="{backup_id}"/>
          </Sequence>
         </RecoverySequence>
      </ReactiveFallback>
    </RecoveryNode>
  </BehaviorTree>
</root>)";
};

}  // namespace modules
}  // namespace ayanami
