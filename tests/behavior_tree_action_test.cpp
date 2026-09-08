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

/// Injects the NavigationRuntime::Ptr into the blackboard (at construction time).
void setNavServer(const BT::Blackboard::Ptr& bb,
                  ayanami::components::NavigationRuntime::Ptr server) {
  bb->set<ayanami::components::NavigationRuntime::Ptr>(k_navigation_runtime,
                                                       server);
}

/// Fills all 17 required blackboard entries (defaults suffice; includes failure_context).
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
 * @brief MockNavigationServer: derives from NavigationServerBase and overrides
 * request* to return fixed TaskHandles (ActionServer protocol). Two modes:
 *  - sync (default): request* publishes the result synchronously; the node
 *    receives it on the next onRunning;
 *  - async: the task stays pending, used to verify onHalted -> task.cancel()
 *    pass-through.
 * session/plan_version use the base-class atomic counters (currentSessionId
 * etc. are non-virtual and not overridden).
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

  /// @brief Simulates a new-goal preemption that expires the session.
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

  /// @brief Simulates an in-place controller plan update request (called when the
  /// FollowPathNode plan_version changes; the real implementation is
  /// ServerControllerAction::updatePlan, with no cancel/new Execution). Records the request, does not simulate execution (requestExecution already publishes the result synchronously).
  bool requestPlanUpdate(uint64_t goal_id, const ExePathGoal& goal) override {
    ++plan_update_count;
    last_plan_update = goal;
    return true;
  }

  std::atomic<int> cancel_count{0};  // Declared before use, destroyed after (the servers cancel callbacks remain safe)

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

  // In-place plan update record (FollowPathNode via requestPlanUpdate).
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

/// Isomorphic to the ComputePath RecoveryNode: main task = the full planning sequence, recovery =
/// a direct <Recovery recovery="{clear_costmap_id}"/> (no AlwaysFailure wrapper).
/// A successful recovery should return SUCCESS -> the RecoveryNode retries the main task.
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

  // onStart already published synchronously -> RUNNING; the result arrives on the next onRunning
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  // ComputePath onRunning writes raw_path; Smooth onStart -> RUNNING
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  // Smooth onRunning writes smoothed_path; whole PipelineSequence SUCCESS
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

  // plan_version changed: FollowPathNode takes the in-place controller update path (requestPlanUpdate),
  // without cancel / join / a new Execution -- exe_request_count stays 1.
  bb->set<Path>(k_smoothed_path, makePath(4));
  server->incrementPlanVersion();
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(server->exe_request_count, 1);
  EXPECT_EQ(server->plan_update_count, 1);
  EXPECT_EQ(server->last_plan_update.plan_version, 1u);
  EXPECT_FALSE(server->last_plan_update.path.poses.empty());
  // No intermediate stop: the old task was not cancelled (requestExecution no longer takes the cancel path).
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

  // Wrapped in a RecoveryNode: a smoothing failure is an explicit FAILURE -> triggers recovery (clear) ->
  // after recovery succeeds the main task is retried -> the retry still fails and retries are exhausted -> whole tree FAILURE.
  BT::BehaviorTreeFactory factory;
  registerNavigationBehaviorTreeNodes(factory);
  auto tree = factory.createTreeFromText(k_compute_path_recovery_xml, bb);

  // Smoothing failure -> triggers recovery (clear) -> retries the main task -> the retry still fails and retries
  // are exhausted -> whole tree terminates in FAILURE. A cycle-exact model would depend on internal node state
  // restore details, so here we drive to a non-RUNNING terminal state and assert the result (bounded iterations prevent a hang).
  BT::NodeStatus final_status = BT::NodeStatus::RUNNING;
  for (int i = 0; i < 32 && final_status == BT::NodeStatus::RUNNING; ++i) {
    final_status = tree.tickExactlyOnce();
  }
  EXPECT_EQ(final_status, BT::NodeStatus::FAILURE);

  // With the smoothing failure made explicit, recovery was indeed entered (one clear).
  EXPECT_EQ(server->recovery_request_count, 1);

  // Smoothing never succeeded -> the plan version is not advanced.
  EXPECT_EQ(server->currentPlanVersion(), 0u);

  // The output path was not polluted by a degraded fallback (still empty; no best-effort raw path fallback).
  Path smooth;
  ASSERT_TRUE(bb->get<Path>(k_smoothed_path, smooth));
  EXPECT_TRUE(smooth.poses.empty());

  // failure_context records the Smoother failure for upper-layer recovery/diagnostics.
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

  // New goal preemption: the session expired before the task finished
  server->bumpSession();
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::FAILURE);

  // Rejected: raw_path not written, plan_version not incremented (key discipline)
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

  // Result carries the current session but a stale plan_version (current-1): before the task finished a new path
  // had been produced and plan_version incremented (session/version double check)
  server->incrementPlanVersion();
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::FAILURE);

  // Rejected: raw_path not written (stale plan_version rejection discipline)
  Path raw;
  ASSERT_TRUE(bb->get<Path>(k_raw_path, raw));
  EXPECT_TRUE(raw.poses.empty());
}

TEST(BehaviorTreeActionTest, haltedNodeCancelsTask) {
  auto server = std::make_shared<MockNavServer>();
  server->path_sync = false;  // Task pending, node stays RUNNING

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

/// Core contract with the AlwaysFailure wrapper removed: recovery success -> RecoveryNode child1 SUCCESS ->
/// returns RUNNING so the main task (full planning sequence) is retried; once the retry succeeds the whole RecoveryNode is SUCCESS.
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

  // tick 1: first planning onStart (synchronous SUCCEEDED but failure=PathNotFound).
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  // tick 2: planning onRunning receives the failure -> main sequence FAILURE -> RecoveryNode enters the recovery branch.
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  // tick 3: recovery action succeeds -> RecoveryNode returns RUNNING so the main task retries.
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);

  // recovery was requested once and executed successfully.
  EXPECT_EQ(server->last_recovery_goal.recovery_id, "clear_costmap");
  EXPECT_EQ(server->path_request_count, 1);

  // After recovery, planning is healthy again: the retried task (2nd requestPath) should succeed.
  server->path_failure = ErrorCode::NONE;

  // tick 4: the main task retries -> planning onStart (2nd requestPath).
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(server->path_request_count, 2);
  EXPECT_EQ(server->last_recovery_goal.recovery_id, "clear_costmap");

  // tick 5/6: the retried path succeeds (plan -> validate -> smooth -> validate) -> RecoveryNode overall SUCCESS.
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

  // The default tree comes from the navigation_behavior_tree extension module: common factory load -> cast ->
  // registerNodes + registerTrees -> defaultTreeId builds the tree.
  BT::BehaviorTreeFactory factory;
  auto ext = std::dynamic_pointer_cast<ayanami::behavior_tree::BehaviorTreeExtension>(
      ayanami::ExtensionModule::load_module(AYANAMI_TEST_BT_EXT_SO));
  ASSERT_NE(ext, nullptr);
  ext->registerNodes(factory);
  ext->registerTrees(factory);
  auto tree = factory.createTree(ext->defaultTreeId(), bb);
  EXPECT_NO_THROW(validateRequiredBlackboardEntries(bb));

  // Simple tick path: enters the ComputePathToPose branch on the first tick (RateController executes immediately on the first tick)
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

  // File version is isomorphic to the inline version: same tick path (first tick enters ComputePathToPose)
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
}

TEST(BehaviorTreeActionTest, recoveryForwardsPerCallBehaviorOverrides) {
  auto server = std::make_shared<MockNavServer>();

  auto bb = BT::Blackboard::create();
  setNavServer(bb, server);
  setRequiredEntries(bb);

  BT::BehaviorTreeFactory factory;
  registerNavigationBehaviorTreeNodes(factory);

  // The XML provides a per-instance override: target_yaw must land in RecoveryGoal.behavior; ports
  // that are not provided stay nullopt (an empty value must never be interpreted as 0).
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

  // <Recovery recovery="{...}"/> without override attributes: behavior entirely empty.
  server->recovery_request_count = 0;
  auto tree2 = factory.createTreeFromText(k_recovery_xml, bb);
  EXPECT_EQ(tree2.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(tree2.tickExactlyOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(server->recovery_request_count, 1);
  EXPECT_TRUE(server->last_recovery_goal.behavior.empty());
}
