// navigation_behavior_tree 扩展模块头 。
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
  // 内嵌默认导航树 (verbatim, 生产单一来源; 文件版不在生产安装)。
  //
  // goal 更新纪律: planner 分支以 {goal_revision} 为 reset_token (新 goal 即重
  // 规划); smoother 分支以 {path_id} 为 reset_token (ComputePathToPose 每次成功
  // 规划 bump path_id, 新路径落地即重新平滑 + bump plan_version), FollowPath 在
  // plan_version 变化时对运行中 controller 做 in-place updatePlan —— 机器人持续
  // 跟随旧路径直到新路径可用, 不会出现"已重规划但不跟随"的陈旧平滑缓存。
 //
 // 恢复轮换纪律 (nav2 对齐): 外层恢复分支用 RoundRobin 而非顺序重试 —— 游标
 // 跨成功保持前进, 重试轮换 spin→wait→backup→spin..., 避免单一恢复反复执行
 // (顺序重试在首个恢复"假成功"时永远轮不到后续手段)。
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
        <RecoveryNode name="FollowPath" number_of_retries="1">
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
