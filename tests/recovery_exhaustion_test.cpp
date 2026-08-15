// Task 27: 复刻 legacy recovery_exhaustion_test.cpp 契约 (新接口等价改写)。
//
// 契约 (参考 MBF actionRecoveryResult §2.7; ayanami 对应 BT 外层 RecoveryNode
// retries 耗尽): 恢复行为列表全部耗尽 (planner 恒失败 + 所有 recovery 恒失败)
// 时, 整次导航必须以 FAILURE 终结, 绝不无限重试。
//
// 两层验证:
//  1. 组件层: RecoveryAction 对恒失败 recovery → 结果 outcome=INTERNAL_ERROR
//     (recovery 错误段 300+), 多次尝试逐次失败 (复刻 legacy 单/多次尝试断言)。
//  2. BT 层: 以实际导航树同构结构 (PipelineSequence + ReactiveFallback 恢复链,
//     外层 RecoveryNode retries=2) + MockNavigationServer (requestPath /
//     requestRecovery 恒 ABORTED) 驱动树, 断言 root 在有界 tick 内 FAILURE,
//     且恢复链每个 id 都被请求 (列表全耗尽), 不无限重试。

#include <gtest/gtest.h>

#include <ayanami/behavior_tree/action_nodes.hpp>
#include <ayanami/behavior_tree/blackboard_protocol.hpp>
#include <ayanami/behavior_tree/navigation.hpp>
#include <ayanami/components/navigation_server_base.hpp>
#include <ayanami/components/recovery_action.hpp>
#include <ayanami/core/abstract_recovery.hpp>
#include <ayanami/messages/actions.hpp>
#include <ayanami/messages/geometry_msgs.hpp>
#include <ayanami/messages/nav_msgs.hpp>

#include <behaviortree_cpp/bt_factory.h>

#include <Eigen/Geometry>

#include <atomic>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

using namespace ayanami::behavior_tree;
using ayanami::actions::ExePathGoal;
using ayanami::actions::ExePathResult;
using ayanami::actions::GetPathGoal;
using ayanami::actions::GetPathResult;
using ayanami::actions::GoalStatus;
using ayanami::actions::RecoveryGoal;
using ayanami::actions::RecoveryResult;
using ayanami::actions::SmoothPathGoal;
using ayanami::actions::SmoothPathResult;
using ayanami::components::TaskHandle;
using ayanami::geometry_msgs::PoseStamped;
using ayanami::nav_msgs::Path;

namespace {

PoseStamped makePose(double x, double y = 0.0) {
  PoseStamped p;
  p.header.frame_id = "map";
  p.pose = Eigen::Isometry3d::Identity();
  p.pose.translation() = Eigen::Vector3d(x, y, 0.0);
  return p;
}

Path makePath(std::size_t n) {
  Path p;
  p.header.frame_id = "map";
  for (std::size_t i = 0; i < n; ++i) {
    p.poses.push_back(makePose(static_cast<double>(i), 0.0));
  }
  return p;
}

// 恒失败 recovery: runRecovery 返回 INTERNAL_ERROR (300)。
class FailingRecovery : public ayanami::abstract::AbstractRecovery {
public:
  uint32_t runRecovery(std::string& message) override {
    ++calls;
    message = "Recovery failed";
    return ayanami::actions::RecoveryResult::INTERNAL_ERROR;
  }
  bool cancel() override { return true; }
  std::atomic<int> calls{0};
};

// ---- BT 层: 恒失败 Mock NavigationServer ----
//
// requestPath/requestSmoothing/requestExecution/requestRecovery 全部同步发布
// ABORTED (outcome 非 SUCCESS), 模拟 planner 永远失败 + recovery 永远失败。
// 记录每个 recovery id 的请求次数, 验证恢复链列表被完全遍历。
class FailingNavServer : public ayanami::components::NavigationServerBase {
public:
  using PathServer = ayanami::components::ActionServer<
      GetPathGoal, GetPathResult, ayanami::actions::GetPathFeedback>;
  using SmoothServer = ayanami::components::ActionServer<
      SmoothPathGoal, SmoothPathResult, ayanami::actions::SmoothPathFeedback>;
  using ExeServer = ayanami::components::ActionServer<
      ExePathGoal, ExePathResult, ayanami::actions::ExePathFeedback>;
  using RecoveryServer = ayanami::components::ActionServer<
      RecoveryGoal, RecoveryResult, ayanami::actions::RecoveryFeedback>;

  TaskHandle<GetPathResult> requestPath(const GetPathGoal& goal) override {
    ++path_calls;
    auto task = path_server_.acceptGoal(goal);
    GetPathResult r;
    r.outcome = GetPathResult::NO_PATH_FOUND;
    r.session_id = goal.session_id;
    r.plan_version = goal.plan_version;
    path_server_.publishResult(task.goal_id(), GoalStatus::ABORTED, r);
    return task;
  }

  TaskHandle<SmoothPathResult> requestSmoothing(
      const SmoothPathGoal& goal) override {
    auto task = smooth_server_.acceptGoal(goal);
    SmoothPathResult r;
    r.outcome = SmoothPathResult::EMPTY_PATH;
    r.session_id = goal.session_id;
    r.plan_version = goal.plan_version;
    smooth_server_.publishResult(task.goal_id(), GoalStatus::ABORTED, r);
    return task;
  }

  TaskHandle<ExePathResult> requestExecution(const ExePathGoal& goal) override {
    auto task = exe_server_.acceptGoal(goal);
    ExePathResult r;
    r.outcome = ExePathResult::NO_VALID_COMMAND;
    r.session_id = goal.session_id;
    r.plan_version = goal.plan_version;
    exe_server_.publishResult(task.goal_id(), GoalStatus::ABORTED, r);
    return task;
  }

  TaskHandle<RecoveryResult> requestRecovery(const RecoveryGoal& goal) override {
    {
      std::lock_guard<std::mutex> lock(recovery_mutex_);
      ++recovery_requests_[goal.recovery_id];
    }
    auto task = recovery_server_.acceptGoal(goal);
    RecoveryResult r;
    r.outcome = RecoveryResult::INTERNAL_ERROR;
    r.session_id = goal.session_id;
    r.plan_version = goal.plan_version;
    recovery_server_.publishResult(task.goal_id(), GoalStatus::ABORTED, r);
    return task;
  }

  int recoveryRequests(const std::string& id) const {
    std::lock_guard<std::mutex> lock(recovery_mutex_);
    auto iter = recovery_requests_.find(id);
    return iter == recovery_requests_.end() ? 0 : iter->second;
  }

  std::atomic<int> path_calls{0};
  mutable std::mutex recovery_mutex_;
  std::map<std::string, int> recovery_requests_;

  PathServer path_server_;
  SmoothServer smooth_server_;
  ExeServer exe_server_;
  RecoveryServer recovery_server_;
};

void setNavServer(const BT::Blackboard::Ptr& bb,
                  ayanami::components::NavigationServerBase* server) {
  bb->set<ayanami::components::NavigationServerBase*>(kNavigationServer, server);
}

void setRequiredEntries(const BT::Blackboard::Ptr& bb) {
  bb->set<PoseStamped>(kGoal, makePose(1.0, 1.0));
  bb->set<Path>(kRawPath, Path{});
  bb->set<Path>(kSmoothedPath, Path{});
  bb->set<double>(kPlannerFrequency, -1.0);
  bb->set<double>(kControllerFrequency, 20.0);
  bb->set<std::string>(kPlannerId, "test_planner");
  bb->set<std::string>(kSmootherId, "test_smoother");
  bb->set<std::string>(kControllerId, "test_controller");
  bb->set<std::string>(kRecoveryId, "clear_costmap");
  bb->set<std::string>(kClearCostmapId, "clear_costmap");
  bb->set<std::string>(kSpinId, "spin");
  bb->set<std::string>(kWaitId, "wait");
  bb->set<std::string>(kBackupId, "back_up");
  bb->set<uint64_t>(kSessionId, 0u);
  bb->set<uint64_t>(kPlanVersion, 0u);
  bb->set<uint64_t>(kResetStuckToken, 1u);
}

// 以实际导航树同构结构 (config/BT/navigation.xml) 建树, 外层 RecoveryNode
// retries=2。去掉 RateController (其 tick 门控会缓存 RUNNING, 不适合本测试
// 紧循环 tick); 恢复链结构 (PipelineSequence + ReactiveFallback) 保持原样。
constexpr const char* kExhaustionTreeXml = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="main">
    <RecoveryNode name="NavigateRecovery" number_of_retries="2">
      <PipelineSequence name="NavigateWithReplanning">
        <RecoveryNode name="ComputePath" number_of_retries="1">
          <Sequence>
            <ComputePathToPose goal="{goal}" path="{raw_path}"/>
            <IsPathValid path="{raw_path}"/>
            <SmoothPath  input_path="{raw_path}" path="{smoothed_path}"/>
            <IsPathValid path="{smoothed_path}"/>
          </Sequence>
          <Sequence>
            <Recovery recovery="{clear_costmap_id}"/>
            <AlwaysFailure/>
          </Sequence>
        </RecoveryNode>
        <RecoveryNode name="FollowPath" number_of_retries="1">
          <FollowPath path="{smoothed_path}" frequency="{controller_frequency}"/>
          <Sequence>
            <Recovery recovery="{clear_costmap_id}"/>
            <AlwaysFailure/>
          </Sequence>
        </RecoveryNode>
      </PipelineSequence>
      <ReactiveFallback name="RecoveryActions">
        <GoalUpdated goal="{goal}"/>
        <RoundRobin name="RecoveryActions">
          <Sequence><Recovery recovery="{spin_id}"/>    <AlwaysFailure/></Sequence>
          <Sequence><Recovery recovery="{wait_id}"/>    <AlwaysFailure/></Sequence>
          <Sequence><Recovery recovery="{backup_id}"/>  <AlwaysFailure/></Sequence>
        </RoundRobin>
      </ReactiveFallback>
    </RecoveryNode>
  </BehaviorTree>
</root>)";

}  // namespace

// ---- 组件层: RecoveryAction 对恒失败 recovery 报告失败 (复刻 legacy) ----

TEST(RecoveryExhaustion, RecoveryActionReportsFailure) {
  auto recovery = std::make_shared<FailingRecovery>();
  ayanami::components::RecoveryAction action(recovery);

  ayanami::actions::RecoveryGoal goal;
  goal.recovery_id = "test_recovery";
  auto task = action.acceptGoal(goal);
  ASSERT_TRUE(task.valid());

  for (int i = 0; i < 200 && !task.ready(); ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  ASSERT_TRUE(task.ready());
  // 失败经 outcome 传达 (recovery 错误段 300+)
  EXPECT_EQ(task.result().outcome, RecoveryResult::INTERNAL_ERROR);
  EXPECT_EQ(recovery->calls.load(), 1);
}

TEST(RecoveryExhaustion, MultipleRecoveryAttemptsAllFail) {
  auto recovery = std::make_shared<FailingRecovery>();
  ayanami::components::RecoveryAction action(recovery);

  for (int i = 0; i < 3; ++i) {
    ayanami::actions::RecoveryGoal goal;
    goal.recovery_id = "test_recovery";
    auto task = action.acceptGoal(goal);
    ASSERT_TRUE(task.valid());
    for (int j = 0; j < 200 && !task.ready(); ++j) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    ASSERT_TRUE(task.ready());
    EXPECT_EQ(task.result().outcome, RecoveryResult::INTERNAL_ERROR);
  }
  EXPECT_EQ(recovery->calls.load(), 3);
}

// ---- BT 层: 恢复链全部耗尽 → 整树 FAILURE, 不无限重试 ----

TEST(RecoveryExhaustion, RecoveryListExhaustedFailsNavigationWithinBounds) {
  FailingNavServer server;

  auto bb = BT::Blackboard::create();
  setNavServer(bb, &server);
  setRequiredEntries(bb);

  BT::BehaviorTreeFactory factory;
  registerNavigationBehaviorTreeNodes(factory);
  auto tree = factory.createTreeFromText(kExhaustionTreeXml, bb);

  // 有界 tick 直到 root FAILURE: 若无限重试则永远 RUNNING, 超限断言失败。
  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  const int kMaxTicks = 60;
  int ticks = 0;
  for (; ticks < kMaxTicks && status != BT::NodeStatus::FAILURE; ++ticks) {
    status = tree.tickExactlyOnce();
  }
  ASSERT_EQ(status, BT::NodeStatus::FAILURE) << "恢复链耗尽后树必须 FAILURE";
  EXPECT_LE(ticks, kMaxTicks);

  // planner 被调用且失败
  EXPECT_GE(server.path_calls.load(), 1);

  // 恢复链列表被完全遍历: clear_costmap (内层) + spin/wait/back_up (外层
  // ReactiveFallback) 每个都被请求
  EXPECT_GE(server.recoveryRequests("clear_costmap"), 1);
  EXPECT_GE(server.recoveryRequests("spin"), 1);
  EXPECT_GE(server.recoveryRequests("wait"), 1);
  EXPECT_GE(server.recoveryRequests("back_up"), 1);

  // 不无限重试: 总恢复请求有界 (每 id 最多一次尝试即耗尽)
  EXPECT_LE(server.recoveryRequests("spin"), 2);
  EXPECT_LE(server.recoveryRequests("wait"), 2);
  EXPECT_LE(server.recoveryRequests("back_up"), 2);
}
