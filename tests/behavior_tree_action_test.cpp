#include <gtest/gtest.h>

#include <ayanami/behavior_tree/action_nodes.hpp>
#include <ayanami/behavior_tree/blackboard_protocol.hpp>
#include <ayanami/behavior_tree/navigation.hpp>
#include <ayanami/components/navigation_server_base.hpp>
#include <ayanami/messages/actions.hpp>
#include <ayanami/messages/geometry_msgs.hpp>
#include <ayanami/messages/nav_msgs.hpp>

#include <behaviortree_cpp/bt_factory.h>

#include <Eigen/Geometry>

#include <atomic>
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

Path makePath(size_t n) {
  Path p;
  p.header.frame_id = "map";
  for (size_t i = 0; i < n; ++i) {
    PoseStamped pt;
    pt.header.frame_id = "map";
    pt.pose = Eigen::Isometry3d::Identity();
    pt.pose.translation() = Eigen::Vector3d(static_cast<double>(i), 0.0, 0.0);
    p.poses.push_back(pt);
  }
  return p;
}

// 注入 NavigationServerBase* 到黑板 (spec §6.2 构造期注入)。
void setNavServer(const BT::Blackboard::Ptr& bb,
                  ayanami::components::NavigationServerBase* server) {
  bb->set<ayanami::components::NavigationServerBase*>(kNavigationServer, server);
}

// 填满 16 个必需板项 (spec §6.5), 默认值即可。
void setRequiredEntries(const BT::Blackboard::Ptr& bb) {
  bb->set<PoseStamped>(kGoal, makePose(0.0));
  bb->set<Path>(kRawPath, Path{});
  bb->set<Path>(kSmoothedPath, Path{});
  bb->set<double>(kPlannerFrequency, 10.0);
  bb->set<double>(kControllerFrequency, 20.0);
  bb->set<std::string>(kPlannerId, "test_planner");
  bb->set<std::string>(kSmootherId, "test_smoother");
  bb->set<std::string>(kControllerId, "test_controller");
  bb->set<std::string>(kRecoveryId, "test_recovery");
  bb->set<std::string>(kClearCostmapId, "clear_costmap");
  bb->set<std::string>(kSpinId, "spin");
  bb->set<std::string>(kWaitId, "wait");
  bb->set<std::string>(kBackupId, "backup");
  bb->set<uint64_t>(kSessionId, 0u);
  bb->set<uint64_t>(kPlanVersion, 0u);
  bb->set<uint64_t>(kResetStuckToken, 1u);
}

/**
 * @brief MockNavigationServer: 继承 NavigationServerBase, 覆写 request* 返回
 * 固定 TaskHandle (Task 9 ActionServer 协议)。支持两种模式:
 *  - sync (默认): request* 内同步 publishResult, 节点下次 onRunning 即收结果;
 *  - async: 任务挂起, 用于验证 onHalted -> task.cancel() 透传。
 * session/plan_version 走基类原子计数 (currentSessionId 等非虚, 不覆写)。
 */
class MockNavServer : public ayanami::components::NavigationServerBase {
 public:
  using PathServer = ayanami::components::ActionServer<
      GetPathGoal, GetPathResult, ayanami::actions::GetPathFeedback>;
  using SmoothServer = ayanami::components::ActionServer<
      SmoothPathGoal, SmoothPathResult, ayanami::actions::SmoothPathFeedback>;
  using ExeServer = ayanami::components::ActionServer<
      ExePathGoal, ExePathResult, ayanami::actions::ExePathFeedback>;
  using RecoveryServer = ayanami::components::ActionServer<
      RecoveryGoal, RecoveryResult, ayanami::actions::RecoveryFeedback>;

  MockNavServer() {
    path_server_.setCancelCallback([this](uint64_t) { cancel_count.fetch_add(1); });
    smooth_server_.setCancelCallback([this](uint64_t) { cancel_count.fetch_add(1); });
    exe_server_.setCancelCallback([this](uint64_t) { cancel_count.fetch_add(1); });
    recovery_server_.setCancelCallback([this](uint64_t) {
      cancel_count.fetch_add(1);
    });
  }

  /// @brief 模拟新 goal 抢占使 session 过期 (spec §6.1)。
  void bumpSession() { session_id_.fetch_add(1); }

  TaskHandle<GetPathResult> requestPath(const GetPathGoal& goal) override {
    auto task = path_server_.acceptGoal(goal);
    last_path_goal_ = goal;
    if (path_sync_) {
      GetPathResult r;
      r.outcome = path_outcome_;
      r.path = path_result_;
      r.session_id = goal.session_id;
      r.plan_version = goal.plan_version;
      path_server_.publishResult(task.goal_id(), GoalStatus::SUCCEEDED, r);
    }
    return task;
  }

  TaskHandle<SmoothPathResult> requestSmoothing(
      const SmoothPathGoal& goal) override {
    auto task = smooth_server_.acceptGoal(goal);
    last_smooth_goal_ = goal;
    if (smooth_sync_) {
      SmoothPathResult r;
      r.outcome = smooth_outcome_;
      r.smoothed_path = smooth_result_;
      r.session_id = goal.session_id;
      r.plan_version = goal.plan_version;
      smooth_server_.publishResult(task.goal_id(), GoalStatus::SUCCEEDED, r);
    }
    return task;
  }

  TaskHandle<ExePathResult> requestExecution(const ExePathGoal& goal) override {
    auto task = exe_server_.acceptGoal(goal);
    last_exe_goal_ = goal;
    if (exe_sync_) {
      ExePathResult r;
      r.outcome = exe_outcome_;
      r.reached = exe_reached_;
      r.session_id = goal.session_id;
      r.plan_version = goal.plan_version;
      exe_server_.publishResult(task.goal_id(), GoalStatus::SUCCEEDED, r);
    }
    return task;
  }

  TaskHandle<RecoveryResult> requestRecovery(const RecoveryGoal& goal) override {
    auto task = recovery_server_.acceptGoal(goal);
    last_recovery_goal_ = goal;
    if (recovery_sync_) {
      RecoveryResult r;
      r.outcome = recovery_outcome_;
      r.session_id = goal.session_id;
      r.plan_version = goal.plan_version;
      recovery_server_.publishResult(task.goal_id(), GoalStatus::SUCCEEDED, r);
    }
    return task;
  }

  std::atomic<int> cancel_count{0};  // 先声明, 后析构 (servers 的 cancel 回调仍安全)

  bool path_sync_ = true;
  uint32_t path_outcome_ = GetPathResult::SUCCESS;
  Path path_result_;
  GetPathGoal last_path_goal_;

  bool smooth_sync_ = true;
  uint32_t smooth_outcome_ = SmoothPathResult::SUCCESS;
  Path smooth_result_;
  SmoothPathGoal last_smooth_goal_;

  bool exe_sync_ = true;
  uint32_t exe_outcome_ = ExePathResult::SUCCESS;
  bool exe_reached_ = true;
  ExePathGoal last_exe_goal_;

  bool recovery_sync_ = true;
  uint32_t recovery_outcome_ = RecoveryResult::SUCCESS;
  RecoveryGoal last_recovery_goal_;

  PathServer path_server_;
  SmoothServer smooth_server_;
  ExeServer exe_server_;
  RecoveryServer recovery_server_;
};

// PipelineSequence: ComputePathToPose -> IsPathValid -> SmoothPath -> IsPathValid
constexpr const char* kPipelineXml = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="main">
    <PipelineSequence name="seq">
      <ComputePathToPose goal="{goal}" path="{raw_path}"/>
      <IsPathValid path="{raw_path}"/>
      <SmoothPath  input_path="{raw_path}" path="{smoothed_path}"/>
      <IsPathValid path="{smoothed_path}"/>
    </PipelineSequence>
  </BehaviorTree>
</root>)";

constexpr const char* kComputePathXml = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="main">
    <ComputePathToPose goal="{goal}" path="{raw_path}"/>
  </BehaviorTree>
</root>)";

constexpr const char* kFollowPathXml = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="main">
    <FollowPath path="{smoothed_path}" frequency="{controller_frequency}"/>
  </BehaviorTree>
</root>)";

constexpr const char* kRecoveryXml = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="main">
    <Recovery recovery="{spin_id}"/>
  </BehaviorTree>
</root>)";

}  // namespace

TEST(BehaviorTreeActionTest, PipelineSequenceWritesPathsAndBumpsPlanVersion) {
  MockNavServer server;
  server.path_result_ = makePath(2);
  server.smooth_result_ = makePath(3);

  auto bb = BT::Blackboard::create();
  setNavServer(bb, &server);
  setRequiredEntries(bb);
  bb->set<PoseStamped>(kGoal, makePose(1.0, 2.0));

  BT::BehaviorTreeFactory factory;
  registerNavigationBehaviorTreeNodes(factory);
  auto tree = factory.createTreeFromText(kPipelineXml, bb);

  // onStart 已同步 publish -> RUNNING; 下次 onRunning 收结果
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  // ComputePath onRunning 写 raw_path + 自增 plan_version; Smooth onStart -> RUNNING
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  // Smooth onRunning 写 smoothed_path; 全 PipelineSequence SUCCESS
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::SUCCESS);

  Path raw;
  ASSERT_TRUE(bb->get<Path>(kRawPath, raw));
  EXPECT_FALSE(raw.poses.empty());
  Path smooth;
  ASSERT_TRUE(bb->get<Path>(kSmoothedPath, smooth));
  EXPECT_FALSE(smooth.poses.empty());

  // plan_version 被 ComputePathToPose 自增; 请求时带当前 session/version
  EXPECT_EQ(server.currentPlanVersion(), 1u);
  EXPECT_EQ(server.last_path_goal_.session_id, server.currentSessionId());
  EXPECT_EQ(server.last_path_goal_.plan_version, 0u);
}

TEST(BehaviorTreeActionTest, StaleSessionIsRejectedOnRunning) {
  MockNavServer server;
  server.path_result_ = makePath(2);

  auto bb = BT::Blackboard::create();
  setNavServer(bb, &server);
  setRequiredEntries(bb);
  bb->set<PoseStamped>(kGoal, makePose(1.0, 2.0));

  BT::BehaviorTreeFactory factory;
  registerNavigationBehaviorTreeNodes(factory);
  auto tree = factory.createTreeFromText(kComputePathXml, bb);

  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);

  // 新 goal 抢占: 任务完成前 session 已过期 (spec §6.1)
  server.bumpSession();
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::FAILURE);

  // 拒收: raw_path 不写入, plan_version 不自增 (spec §6.2 关键纪律)
  Path raw;
  ASSERT_TRUE(bb->get<Path>(kRawPath, raw));
  EXPECT_TRUE(raw.poses.empty());
  EXPECT_EQ(server.currentPlanVersion(), 0u);
}

TEST(BehaviorTreeActionTest, VersionStaleResultRejected) {
  MockNavServer server;
  server.path_result_ = makePath(2);

  auto bb = BT::Blackboard::create();
  setNavServer(bb, &server);
  setRequiredEntries(bb);
  bb->set<PoseStamped>(kGoal, makePose(1.0, 2.0));

  BT::BehaviorTreeFactory factory;
  registerNavigationBehaviorTreeNodes(factory);
  auto tree = factory.createTreeFromText(kComputePathXml, bb);

  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);

  // result 带 current session 但 plan_version 陈旧 (current-1): 任务完成前新路径
  // 已生成并自增 plan_version (spec §6.2 session/version 双校验)
  server.incrementPlanVersion();
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::FAILURE);

  // 拒收: raw_path 不写入 (plan_version stale 拒收纪律)
  Path raw;
  ASSERT_TRUE(bb->get<Path>(kRawPath, raw));
  EXPECT_TRUE(raw.poses.empty());
}

TEST(BehaviorTreeActionTest, HaltedNodeCancelsTask) {
  MockNavServer server;
  server.path_sync_ = false;  // 任务挂起, 节点保持 RUNNING

  auto bb = BT::Blackboard::create();
  setNavServer(bb, &server);
  setRequiredEntries(bb);
  bb->set<PoseStamped>(kGoal, makePose(1.0, 2.0));

  BT::BehaviorTreeFactory factory;
  registerNavigationBehaviorTreeNodes(factory);
  auto tree = factory.createTreeFromText(kComputePathXml, bb);

  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  ASSERT_GT(server.path_server_.activeGoalCount(), 0u);

  tree.haltTree();  // StatefulActionNode::halt -> onHalted -> task.cancel()
  EXPECT_GE(server.cancel_count.load(), 1);
}

TEST(BehaviorTreeActionTest, FollowPathCompletesWhenControllerSucceeds) {
  MockNavServer server;
  server.exe_sync_ = true;
  server.exe_outcome_ = ExePathResult::SUCCESS;
  server.exe_reached_ = true;

  auto bb = BT::Blackboard::create();
  setNavServer(bb, &server);
  setRequiredEntries(bb);
  bb->set<Path>(kSmoothedPath, makePath(2));

  BT::BehaviorTreeFactory factory;
  registerNavigationBehaviorTreeNodes(factory);
  auto tree = factory.createTreeFromText(kFollowPathXml, bb);

  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(server.last_exe_goal_.session_id, server.currentSessionId());
  EXPECT_EQ(server.last_exe_goal_.plan_version, server.currentPlanVersion());
}

TEST(BehaviorTreeActionTest, RecoveryActionCompletesOnSuccess) {
  MockNavServer server;

  auto bb = BT::Blackboard::create();
  setNavServer(bb, &server);
  setRequiredEntries(bb);

  BT::BehaviorTreeFactory factory;
  registerNavigationBehaviorTreeNodes(factory);
  auto tree = factory.createTreeFromText(kRecoveryXml, bb);

  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(server.last_recovery_goal_.recovery_id, "spin");
}

TEST(BehaviorTreeActionTest, DefaultEmbeddedXmlLoadsValidatesAndTicks) {
  MockNavServer server;
  server.path_result_ = makePath(2);
  server.smooth_result_ = makePath(2);

  auto bb = BT::Blackboard::create();
  setNavServer(bb, &server);
  setRequiredEntries(bb);
  bb->set<PoseStamped>(kGoal, makePose(1.0, 2.0));

  auto tree = createDefaultNavigationTree(bb, "NavigateWithReplanningAndSmoothing");
  EXPECT_NO_THROW(validateRequiredBlackboardEntries(bb));

  // 简单 tick 路径: 首次进入 ComputePathToPose 分支 (RateController 首 tick 立即执行)
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  EXPECT_NE(server.last_path_goal_.goal.pose.translation().x(), 0.0);
}

TEST(BehaviorTreeActionTest, DefaultFileXmlIsIsomorphicAndLoads) {
  MockNavServer server;

  auto bb = BT::Blackboard::create();
  setNavServer(bb, &server);
  setRequiredEntries(bb);
  bb->set<PoseStamped>(kGoal, makePose(1.0, 2.0));

  BT::BehaviorTreeFactory factory;
  registerNavigationBehaviorTreeNodes(factory);
  factory.registerBehaviorTreeFromFile(AYANAMI_TEST_BT_XML);
  auto tree = factory.createTree("NavigateWithReplanningAndSmoothing", bb);
  EXPECT_NO_THROW(validateRequiredBlackboardEntries(bb));

  // 文件版与内嵌版同构: 同一 tick 路径 (首 tick 进入 ComputePathToPose)
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
}
