#include <gtest/gtest.h>

#include <ayanami/behavior_tree/action_nodes.hpp>
#include <ayanami/behavior_tree/blackboard_protocol.hpp>
#include <ayanami/behavior_tree/extension.hpp>
#include <ayanami/behavior_tree/navigation.hpp>
#include <ayanami/abstract/components/navigation_server_base.hpp>
#include <ayanami/messages/actions.hpp>
#include <ayanami/messages/failure.hpp>
#include <ayanami/types.hpp>
#include <ayanami/messages/geometry_msgs.hpp>
#include <ayanami/messages/nav_msgs.hpp>
#include <ayanami/utility/extension_module.hpp>

#include <behaviortree_cpp/bt_factory.h>

#include <Eigen/Geometry>

#include <atomic>
#include <memory>
#include <string>
#include <vector>

using namespace ayanami::behavior_tree;
using ayanami::ErrorCode;
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
  p.pose = ayanami::geometry_msgs::Pose::fromIsometry(Eigen::Isometry3d::Identity());
  p.pose.position = Eigen::Vector3d(x, y, 0.0);
  return p;
}

Path makePath(size_t n) {
  Path p;
  p.header.frame_id = "map";
  for (size_t i = 0; i < n; ++i) {
    PoseStamped pt;
    pt.header.frame_id = "map";
    pt.pose = ayanami::geometry_msgs::Pose::fromIsometry(Eigen::Isometry3d::Identity());
    pt.pose.position = Eigen::Vector3d(static_cast<double>(i), 0.0, 0.0);
    p.poses.push_back(pt);
  }
  return p;
}

// 注入 NavigationRuntime::Ptr 到黑板 (构造期注入)。
void setNavServer(const BT::Blackboard::Ptr& bb,
                  ayanami::components::NavigationRuntime::Ptr server) {
  bb->set<ayanami::components::NavigationRuntime::Ptr>(k_navigation_runtime,
                                                       server);
}

// 填满 17 个必需板项 (默认值即可; 含 failure_context)。
void setRequiredEntries(const BT::Blackboard::Ptr& bb) {
  bb->set<PoseStamped>(k_goal, makePose(0.0));
  bb->set<Path>(k_raw_path, Path{});
  bb->set<Path>(k_smoothed_path, Path{});
  bb->set<double>(k_planner_frequency, 10.0);
  bb->set<double>(k_smoother_frequency, -1.0);
  bb->set<double>(k_controller_frequency, 20.0);
  bb->set<std::string>(k_clear_costmap_id, "clear_costmap");
  bb->set<std::string>(k_spin_id, "spin");
  bb->set<std::string>(k_wait_id, "wait");
  bb->set<std::string>(k_backup_id, "backup");
  bb->set<uint64_t>(k_session_id, 0u);
  bb->set<uint64_t>(k_plan_version, 0u);
  bb->set<uint64_t>(k_reset_stuck_token, 1u);
  bb->set<uint64_t>(k_goal_revision, 0u);
  bb->set<ayanami::FailureContext>(k_failure_context, ayanami::FailureContext{});
}

/**
 * @brief MockNavigationServer: 继承 NavigationServerBase, 覆写 request* 返回
 * 固定 TaskHandle (ActionServer 协议)。支持两种模式:
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
    path_server.setCancelCallback([this](uint64_t) { cancel_count.fetch_add(1); });
    smooth_server.setCancelCallback([this](uint64_t) { cancel_count.fetch_add(1); });
    exe_server.setCancelCallback([this](uint64_t) { cancel_count.fetch_add(1); });
    recovery_server.setCancelCallback([this](uint64_t) {
      cancel_count.fetch_add(1);
    });
  }

  /// @brief 模拟新 goal 抢占使 session 过期 。
  void bumpSession() { startNewSession(); }

  TaskHandle<GetPathResult> requestPath(const GetPathGoal& goal) override {
    ++path_request_count;
    auto task = path_server.acceptGoal(goal);
    last_path_goal = goal;
    if (path_sync) {
      GetPathResult r;
      r.error = path_failure;
      r.path = path_result;
      r.session_id = goal.session_id;
      r.plan_version = goal.plan_version;
      path_server.publishResult(task.goal_id(), GoalStatus::SUCCEEDED, r);
    }
    return task;
  }

  TaskHandle<SmoothPathResult> requestSmoothing(
      const SmoothPathGoal& goal) override {
    auto task = smooth_server.acceptGoal(goal);
    last_smooth_goal = goal;
    if (smooth_sync) {
      SmoothPathResult r;
      r.error = smooth_failure;
      r.smoothed_path = smooth_result;
      r.session_id = goal.session_id;
      r.plan_version = goal.plan_version;
      smooth_server.publishResult(task.goal_id(), GoalStatus::SUCCEEDED, r);
    }
    return task;
  }

  TaskHandle<ExePathResult> requestExecution(const ExePathGoal& goal) override {
    ++exe_request_count;
    auto task = exe_server.acceptGoal(goal);
    last_exe_goal = goal;
    if (exe_sync) {
      ExePathResult r;
      r.error = exe_failure;
      r.reached = exe_reached;
      r.session_id = goal.session_id;
      r.plan_version = goal.plan_version;
      exe_server.publishResult(task.goal_id(), GoalStatus::SUCCEEDED, r);
    }
    return task;
  }

  TaskHandle<RecoveryResult> requestRecovery(const RecoveryGoal& goal) override {
    ++recovery_request_count;
    auto task = recovery_server.acceptGoal(goal);
    last_recovery_goal = goal;
    if (recovery_sync) {
      RecoveryResult r;
      r.error = recovery_failure;
      r.session_id = goal.session_id;
      r.plan_version = goal.plan_version;
      recovery_server.publishResult(task.goal_id(), GoalStatus::SUCCEEDED, r);
    }
    return task;
  }

  /// @brief 模拟 in-place 控制器规划更新请求 (FollowPathNode plan_version 变化
  /// 时调用; 真实实现 = ServerControllerAction::updatePlan, 无 cancel/new
  /// Execution)。记录请求, 不模拟执行 (requestExecution 已同步发布结果)。
  bool requestPlanUpdate(uint64_t goal_id, const ExePathGoal& goal) override {
    ++plan_update_count;
    last_plan_update = goal;
    return true;
  }

  std::atomic<int> cancel_count{0};  // 先声明, 后析构 (servers 的 cancel 回调仍安全)

  bool path_sync = true;
  ErrorCode path_failure = ErrorCode::NONE;
  Path path_result;
  GetPathGoal last_path_goal;

  bool smooth_sync = true;
  ErrorCode smooth_failure = ErrorCode::NONE;
  Path smooth_result;
  SmoothPathGoal last_smooth_goal;

  bool exe_sync = true;
  ErrorCode exe_failure = ErrorCode::NONE;
  bool exe_reached = true;
  ExePathGoal last_exe_goal;
  int exe_request_count = 0;

  int path_request_count = 0;

  // in-place 规划更新记录 (FollowPathNode 经 requestPlanUpdate)。
  int plan_update_count = 0;
  ExePathGoal last_plan_update;

  bool recovery_sync = true;
  ErrorCode recovery_failure = ErrorCode::NONE;
  RecoveryGoal last_recovery_goal;
  int recovery_request_count = 0;

  PathServer path_server;
  SmoothServer smooth_server;
  ExeServer exe_server;
  RecoveryServer recovery_server;
};

// PipelineSequence: ComputePathToPose -> IsPathValid -> SmoothPath -> IsPathValid
constexpr const char* k_pipeline_xml = R"(
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

constexpr const char* k_compute_path_xml = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="main">
    <ComputePathToPose goal="{goal}" path="{raw_path}"/>
  </BehaviorTree>
</root>)";

constexpr const char* k_follow_path_xml = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="main">
    <FollowPath path="{smoothed_path}" frequency="{controller_frequency}"/>
  </BehaviorTree>
</root>)";

constexpr const char* k_recovery_xml = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="main">
    <Recovery recovery="{spin_id}"/>
  </BehaviorTree>
</root>)";

// ComputePath RecoveryNode 同构 : 主任务 = 完整规划序列, 恢复 =
// <Recovery recovery="{clear_costmap_id}"/> 直连 (无 AlwaysFailure 包装)。
// recovery 成功应返回 SUCCESS → RecoveryNode 重试主任务。
constexpr const char* k_compute_path_recovery_xml = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="main">
    <RecoveryNode name="ComputePath" number_of_retries="1">
      <Sequence>
        <ComputePathToPose goal="{goal}" path="{raw_path}"/>
        <IsPathValid path="{raw_path}"/>
        <SmoothPath  input_path="{raw_path}" path="{smoothed_path}"/>
        <IsPathValid path="{smoothed_path}"/>
      </Sequence>
      <Recovery recovery="{clear_costmap_id}"/>
    </RecoveryNode>
  </BehaviorTree>
</root>)";

}  // namespace

TEST(BehaviorTreeActionTest, pipelineSequenceWritesPathsAndBumpsPlanVersion) {
  auto server = std::make_shared<MockNavServer>();
  server->path_result = makePath(2);
  server->smooth_result = makePath(3);

  auto bb = BT::Blackboard::create();
  setNavServer(bb, server);
  setRequiredEntries(bb);
  bb->set<PoseStamped>(k_goal, makePose(1.0, 2.0));

  BT::BehaviorTreeFactory factory;
  registerNavigationBehaviorTreeNodes(factory);
  auto tree = factory.createTreeFromText(k_pipeline_xml, bb);

  // onStart 已同步 publish -> RUNNING; 下次 onRunning 收结果
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  // ComputePath onRunning 写 raw_path; Smooth onStart -> RUNNING
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  // Smooth onRunning 写 smoothed_path; 全 PipelineSequence SUCCESS
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::SUCCESS);

  Path raw;
  ASSERT_TRUE(bb->get<Path>(k_raw_path, raw));
  EXPECT_FALSE(raw.poses.empty());
  Path smooth;
  ASSERT_TRUE(bb->get<Path>(k_smoothed_path, smooth));
  EXPECT_FALSE(smooth.poses.empty());

  // Only the final smoothed path publishes a new controller plan version.
  EXPECT_EQ(server->currentPlanVersion(), 1u);
  EXPECT_EQ(server->last_path_goal.session_id, server->currentSessionId());
  EXPECT_EQ(server->last_path_goal.plan_version, 0u);
}

TEST(BehaviorTreeActionTest, followPathUpdatesPlanInPlaceWhenPlanVersionChanges) {
  auto server = std::make_shared<MockNavServer>();
  server->exe_sync = false;

  auto bb = BT::Blackboard::create();
  setNavServer(bb, server);
  setRequiredEntries(bb);
  bb->set<Path>(k_smoothed_path, makePath(2));

  BT::BehaviorTreeFactory factory;
  registerNavigationBehaviorTreeNodes(factory);
  auto tree = factory.createTreeFromText(k_follow_path_xml, bb);

  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  ASSERT_EQ(server->exe_request_count, 1);
  EXPECT_EQ(server->last_exe_goal.plan_version, 0u);

  // plan_version 变化: FollowPathNode 走 in-place 控制器更新 (requestPlanUpdate),
  // 不 cancel / 不 join / 不新建 Execution —— exe_request_count 保持 1。
  bb->set<Path>(k_smoothed_path, makePath(4));
  server->incrementPlanVersion();
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(server->exe_request_count, 1);
  EXPECT_EQ(server->plan_update_count, 1);
  EXPECT_EQ(server->last_plan_update.plan_version, 1u);
  EXPECT_FALSE(server->last_plan_update.path.poses.empty());
  // 无中间停止: 未取消旧 task (requestExecution 不再走 cancel 路径)。
  EXPECT_EQ(server->cancel_count.load(), 0);
}

TEST(BehaviorTreeActionTest, everySmoothingCompletionBumpsVersion) {
  auto server = std::make_shared<MockNavServer>();
  server->path_result = makePath(2);
  server->smooth_result = makePath(3);

  auto bb = BT::Blackboard::create();
  setNavServer(bb, server);
  setRequiredEntries(bb);
  bb->set<PoseStamped>(k_goal, makePose(1.0, 2.0));

  BT::BehaviorTreeFactory factory;
  registerNavigationBehaviorTreeNodes(factory);
  auto tree = factory.createTreeFromText(k_pipeline_xml, bb);
  for (int cycle = 0; cycle < 2; ++cycle) {
    EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
    EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
    EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::SUCCESS);
  }
  EXPECT_EQ(server->currentPlanVersion(), 2u);
}

TEST(BehaviorTreeActionTest, smoothingFailurePropagatesToRecoveryThenFails) {
  auto server = std::make_shared<MockNavServer>();
  server->path_result = makePath(2);
  server->smooth_result = makePath(3);
  server->smooth_failure = ErrorCode::SMOOTHING_FAILED;

  auto bb = BT::Blackboard::create();
  setNavServer(bb, server);
  setRequiredEntries(bb);
  bb->set<PoseStamped>(k_goal, makePose(1.0, 2.0));

  // 用 RecoveryNode 包裹: 平滑失败是显式 FAILURE → 触发 recovery (clear) →
  // recovery 成功后重试主任务 → 重试仍失败且 retries 耗尽 → 整树 FAILURE。
  BT::BehaviorTreeFactory factory;
  registerNavigationBehaviorTreeNodes(factory);
  auto tree = factory.createTreeFromText(k_compute_path_recovery_xml, bb);

  // 平滑失败 → 触发 recovery (clear) → 重试主任务 → 重试仍失败且 retries
  // 耗尽 → 整树终态 FAILURE。逐拍模型依赖内部节点状态恢复细节, 这里驱动到
  // 非 RUNNING 终态断言结果 (有界迭代防死循环)。
  BT::NodeStatus final_status = BT::NodeStatus::RUNNING;
  for (int i = 0; i < 32 && final_status == BT::NodeStatus::RUNNING; ++i) {
    final_status = tree.tickExactlyOnce();
  }
  EXPECT_EQ(final_status, BT::NodeStatus::FAILURE);

  // 平滑失败显式化后确实进入了 recovery (一次 clear)。
  EXPECT_EQ(server->recovery_request_count, 1);

  // 平滑从未成功 -> plan version 不被推进。
  EXPECT_EQ(server->currentPlanVersion(), 0u);

  // 输出路径未被降级污染 (仍为空, 无 best-effort raw path 回退)。
  Path smooth;
  ASSERT_TRUE(bb->get<Path>(k_smoothed_path, smooth));
  EXPECT_TRUE(smooth.poses.empty());

  // failure_context 记录 Smoother 失败, 供上层 recovery/诊断使用。
  ayanami::FailureContext after;
  ASSERT_TRUE(bb->get<ayanami::FailureContext>(k_failure_context, after));
  EXPECT_EQ(after.stage, ayanami::FailureStage::Smoother);
  EXPECT_EQ(after.code, ErrorCode::SMOOTHING_FAILED);
}

TEST(BehaviorTreeActionTest, staleSessionIsRejectedOnRunning) {
  auto server = std::make_shared<MockNavServer>();
  server->path_result = makePath(2);

  auto bb = BT::Blackboard::create();
  setNavServer(bb, server);
  setRequiredEntries(bb);
  bb->set<PoseStamped>(k_goal, makePose(1.0, 2.0));

  BT::BehaviorTreeFactory factory;
  registerNavigationBehaviorTreeNodes(factory);
  auto tree = factory.createTreeFromText(k_compute_path_xml, bb);

  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);

  // 新 goal 抢占: 任务完成前 session 已过期
  server->bumpSession();
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::FAILURE);

  // 拒收: raw_path 不写入, plan_version 不自增 (关键纪律)
  Path raw;
  ASSERT_TRUE(bb->get<Path>(k_raw_path, raw));
  EXPECT_TRUE(raw.poses.empty());
  EXPECT_EQ(server->currentPlanVersion(), 0u);
}

TEST(BehaviorTreeActionTest, versionStaleResultRejected) {
  auto server = std::make_shared<MockNavServer>();
  server->path_result = makePath(2);

  auto bb = BT::Blackboard::create();
  setNavServer(bb, server);
  setRequiredEntries(bb);
  bb->set<PoseStamped>(k_goal, makePose(1.0, 2.0));

  BT::BehaviorTreeFactory factory;
  registerNavigationBehaviorTreeNodes(factory);
  auto tree = factory.createTreeFromText(k_compute_path_xml, bb);

  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);

  // result 带 current session 但 plan_version 陈旧 (current-1): 任务完成前新路径
  // 已生成并自增 plan_version (session/version 双校验)
  server->incrementPlanVersion();
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::FAILURE);

  // 拒收: raw_path 不写入 (plan_version stale 拒收纪律)
  Path raw;
  ASSERT_TRUE(bb->get<Path>(k_raw_path, raw));
  EXPECT_TRUE(raw.poses.empty());
}

TEST(BehaviorTreeActionTest, haltedNodeCancelsTask) {
  auto server = std::make_shared<MockNavServer>();
  server->path_sync = false;  // 任务挂起, 节点保持 RUNNING

  auto bb = BT::Blackboard::create();
  setNavServer(bb, server);
  setRequiredEntries(bb);
  bb->set<PoseStamped>(k_goal, makePose(1.0, 2.0));

  BT::BehaviorTreeFactory factory;
  registerNavigationBehaviorTreeNodes(factory);
  auto tree = factory.createTreeFromText(k_compute_path_xml, bb);

  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  ASSERT_GT(server->path_server.activeGoalCount(), 0u);

  tree.haltTree();  // StatefulActionNode::halt -> onHalted -> task.cancel
  EXPECT_GE(server->cancel_count.load(), 1);
}

TEST(BehaviorTreeActionTest, followPathCompletesWhenControllerSucceeds) {
  auto server = std::make_shared<MockNavServer>();
  server->exe_sync = true;
  server->exe_failure = ErrorCode::NONE;
  server->exe_reached = true;

  auto bb = BT::Blackboard::create();
  setNavServer(bb, server);
  setRequiredEntries(bb);
  bb->set<Path>(k_smoothed_path, makePath(2));

  BT::BehaviorTreeFactory factory;
  registerNavigationBehaviorTreeNodes(factory);
  auto tree = factory.createTreeFromText(k_follow_path_xml, bb);

  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(server->last_exe_goal.session_id, server->currentSessionId());
  EXPECT_EQ(server->last_exe_goal.plan_version, server->currentPlanVersion());
}

TEST(BehaviorTreeActionTest, recoveryActionCompletesOnSuccess) {
  auto server = std::make_shared<MockNavServer>();

  auto bb = BT::Blackboard::create();
  setNavServer(bb, server);
  setRequiredEntries(bb);

  BT::BehaviorTreeFactory factory;
  registerNavigationBehaviorTreeNodes(factory);
  auto tree = factory.createTreeFromText(k_recovery_xml, bb);

  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(server->last_recovery_goal.recovery_id, "spin");
}

// 移除 AlwaysFailure 包装的核心契约: recovery 成功 → RecoveryNode child1 SUCCESS →
// 返回 RUNNING 让主任务 (完整规划序列) 重试; 重试成功后 RecoveryNode 整体 SUCCESS。
TEST(BehaviorTreeActionTest, successfulRecoveryRetriesMainChild) {
  auto server = std::make_shared<MockNavServer>();
  server->path_result = makePath(2);
  server->smooth_result = makePath(3);
  server->path_failure = ErrorCode::PATH_NOT_FOUND;
  server->recovery_failure = ErrorCode::NONE;

  auto bb = BT::Blackboard::create();
  setNavServer(bb, server);
  setRequiredEntries(bb);
  bb->set<PoseStamped>(k_goal, makePose(1.0, 2.0));

  BT::BehaviorTreeFactory factory;
  registerNavigationBehaviorTreeNodes(factory);
  auto tree = factory.createTreeFromText(k_compute_path_recovery_xml, bb);

  // tick 1: 首次规划 onStart (同步 SUCCEEDED 但 failure=PathNotFound)。
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  // tick 2: 规划 onRunning 收失败 → 主序列 FAILURE → RecoveryNode 进入恢复分支。
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  // tick 3: recovery action 成功 → RecoveryNode 返回 RUNNING 让主任务重试。
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);

  // recovery 已被请求一次且成功执行。
  EXPECT_EQ(server->last_recovery_goal.recovery_id, "clear_costmap");
  EXPECT_EQ(server->path_request_count, 1);

  // 恢复后规划恢复正常: 重试任务 (第 2 次 requestPath) 应成功。
  server->path_failure = ErrorCode::NONE;

  // tick 4: 主任务重试 → 规划 onStart (第 2 次 requestPath)。
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(server->path_request_count, 2);
  EXPECT_EQ(server->last_recovery_goal.recovery_id, "clear_costmap");

  // tick 5/6: 重试路径成功 (规划 → 校验 → 平滑 → 校验) → RecoveryNode 整体 SUCCESS。
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(server->path_request_count, 2);
}

TEST(BehaviorTreeActionTest, defaultEmbeddedXmlLoadsValidatesAndTicks) {
  auto server = std::make_shared<MockNavServer>();
  server->path_result = makePath(2);
  server->smooth_result = makePath(2);

  auto bb = BT::Blackboard::create();
  setNavServer(bb, server);
  setRequiredEntries(bb);
  bb->set<PoseStamped>(k_goal, makePose(1.0, 2.0));

  // 默认树来自 navigation_behavior_tree 扩展模块: 公共工厂装载 → cast →
  // registerNodes + registerTrees → defaultTreeId 建树 。
  BT::BehaviorTreeFactory factory;
  auto ext = std::dynamic_pointer_cast<ayanami::behavior_tree::BehaviorTreeExtension>(
      ayanami::ExtensionModule::load_module(AYANAMI_TEST_BT_EXT_SO));
  ASSERT_NE(ext, nullptr);
  ext->registerNodes(factory);
  ext->registerTrees(factory);
  auto tree = factory.createTree(ext->defaultTreeId(), bb);
  EXPECT_NO_THROW(validateRequiredBlackboardEntries(bb));

  // 简单 tick 路径: 首次进入 ComputePathToPose 分支 (RateController 首 tick 立即执行)
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  EXPECT_NE(server->last_path_goal.goal.pose.position.x(), 0.0);
}

TEST(BehaviorTreeActionTest, defaultFileXmlIsIsomorphicAndLoads) {
  auto server = std::make_shared<MockNavServer>();

  auto bb = BT::Blackboard::create();
  setNavServer(bb, server);
  setRequiredEntries(bb);
  bb->set<PoseStamped>(k_goal, makePose(1.0, 2.0));

  BT::BehaviorTreeFactory factory;
  registerNavigationBehaviorTreeNodes(factory);
  factory.registerBehaviorTreeFromFile(AYANAMI_TEST_BT_XML);
  auto tree = factory.createTree("NavigateWithReplanningAndSmoothing", bb);
  EXPECT_NO_THROW(validateRequiredBlackboardEntries(bb));

  // 文件版与内嵌版同构: 同一 tick 路径 (首 tick 进入 ComputePathToPose)
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
}

TEST(BehaviorTreeActionTest, recoveryForwardsPerCallBehaviorOverrides) {
  auto server = std::make_shared<MockNavServer>();

  auto bb = BT::Blackboard::create();
  setNavServer(bb, server);
  setRequiredEntries(bb);

  BT::BehaviorTreeFactory factory;
  registerNavigationBehaviorTreeNodes(factory);

  // XML 提供按次覆盖: target_yaw 应进入 RecoveryGoal.behavior; 未提供的
  // 端口保持 nullopt (空值绝不能解释为 0)。
  constexpr const char* k_override_xml = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="main">
    <Recovery recovery="{spin_id}" target_yaw="3.14" backup_distance="0.5"/>
  </BehaviorTree>
</root>)";
  auto tree = factory.createTreeFromText(k_override_xml, bb);
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::SUCCESS);

  EXPECT_EQ(server->recovery_request_count, 1);
  ASSERT_TRUE(server->last_recovery_goal.behavior.target_yaw.has_value());
  EXPECT_DOUBLE_EQ(*server->last_recovery_goal.behavior.target_yaw, 3.14);
  ASSERT_TRUE(server->last_recovery_goal.behavior.backup_distance.has_value());
  EXPECT_DOUBLE_EQ(*server->last_recovery_goal.behavior.backup_distance, 0.5);
  EXPECT_FALSE(server->last_recovery_goal.behavior.backup_speed.has_value());
  EXPECT_FALSE(server->last_recovery_goal.behavior.wait_duration.has_value());

  // 无覆盖属性的 <Recovery recovery="{...}"/>: behavior 全空。
  server->recovery_request_count = 0;
  auto tree2 = factory.createTreeFromText(k_recovery_xml, bb);
  EXPECT_EQ(tree2.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(tree2.tickExactlyOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(server->recovery_request_count, 1);
  EXPECT_TRUE(server->last_recovery_goal.behavior.empty());
}
