// Re-creates the legacy recovery_exhaustion_test.cpp contract (equivalent rewrite on the new interface).
//
// Contract (cf. MBF actionRecoveryResult §2.7; the ayanami counterpart is the BT outer RecoveryNode
// retries being exhausted): when the recovery behavior list is fully exhausted (planner always fails + every recovery always fails),
// the whole navigation must terminate with FAILURE and never retry endlessly.
//
// Two layers of verification:
// 1. Component layer: RecoveryAction against an always-failing recovery -> result failure=InternalError
// (unified failure code); multiple attempts fail one by one (re-creates the legacy single/multi-attempt assertions).
// 2. BT layer: drive a tree isomorphic to the real navigation tree (PipelineSequence + ReactiveFallback recovery chain,
// outer RecoveryNode retries=2) + a MockNavigationServer (requestPath /
// requestRecovery always ABORTED); assert the root reaches FAILURE within bounded ticks,
// every id in the recovery chain was requested (the list fully exhausted), and retries are not endless.

#include <gtest/gtest.h>

#include <ayanami/behavior_tree/action_nodes.hpp>
#include <ayanami/behavior_tree/blackboard_protocol.hpp>
#include <ayanami/behavior_tree/navigation.hpp>
#include <ayanami/abstract/components/navigation_server_base.hpp>
#include <ayanami/abstract/components/recovery_action.hpp>
#include <ayanami/abstract/core/abstract_recovery.hpp>
#include <ayanami/messages/actions.hpp>
#include <ayanami/messages/failure.hpp>
#include <ayanami/types.hpp>
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
using ayanami::ErrorCode;
using ayanami::RecoveryCycleData;
using ayanami::ResultT;
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
  p.pose.position = Eigen::Vector3d(x, y, 0.0);
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

/// Always-failing recovery: tick returns the INTERNAL_ERROR failure terminal state.
class FailingRecovery : public ayanami::abstract::AbstractRecovery {
public:
  ResultT<void> tick(RecoveryCycleData& cycle) override {
    ++calls;
    return ResultT<void>::failure(ErrorCode::INTERNAL_ERROR,
                                    "Recovery failed");
  }
  bool cancel() override { return true; }
  std::atomic<int> calls{0};
};

// ---- BT layer: always-failing mock NavigationServer ----
//
/// requestPath/requestSmoothing/requestExecution/requestRecovery all synchronously publish
/// ABORTED (error non-NONE), simulating a planner that always fails + recoveries that always fail.
/// Records the request count per recovery id to verify the recovery chain list was fully traversed.
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
    auto task = path_server.acceptGoal(goal);
    GetPathResult r;
    r.error = ErrorCode::PATH_NOT_FOUND;
    r.session_id = goal.session_id;
    r.plan_version = goal.plan_version;
    path_server.publishResult(task.goal_id(), GoalStatus::ABORTED, r);
    return task;
  }

  TaskHandle<SmoothPathResult> requestSmoothing(
      const SmoothPathGoal& goal) override {
    auto task = smooth_server.acceptGoal(goal);
    SmoothPathResult r;
    r.error = ErrorCode::INVALID_INPUT;
    r.session_id = goal.session_id;
    r.plan_version = goal.plan_version;
    smooth_server.publishResult(task.goal_id(), GoalStatus::ABORTED, r);
    return task;
  }

  TaskHandle<ExePathResult> requestExecution(const ExePathGoal& goal) override {
    auto task = exe_server.acceptGoal(goal);
    ExePathResult r;
    r.error = ErrorCode::NO_VALID_COMMAND;
    r.session_id = goal.session_id;
    r.plan_version = goal.plan_version;
    exe_server.publishResult(task.goal_id(), GoalStatus::ABORTED, r);
    return task;
  }

  TaskHandle<RecoveryResult> requestRecovery(const RecoveryGoal& goal) override {
    {
      std::lock_guard<std::mutex> lock(recovery_mutex);
      ++recovery_requests[goal.recovery_id];
    }
    auto task = recovery_server.acceptGoal(goal);
    RecoveryResult r;
    r.error = ErrorCode::INTERNAL_ERROR;
    r.session_id = goal.session_id;
    r.plan_version = goal.plan_version;
    recovery_server.publishResult(task.goal_id(), GoalStatus::ABORTED, r);
    return task;
  }

  int recoveryRequests(const std::string& id) const {
    std::lock_guard<std::mutex> lock(recovery_mutex);
    auto iter = recovery_requests.find(id);
    return iter == recovery_requests.end() ? 0 : iter->second;
  }

  std::atomic<int> path_calls{0};
  mutable std::mutex recovery_mutex;
  std::map<std::string, int> recovery_requests;

  PathServer path_server;
  SmoothServer smooth_server;
  ExeServer exe_server;
  RecoveryServer recovery_server;
};

void setNavServer(const BT::Blackboard::Ptr& bb,
                  ayanami::components::NavigationRuntime::Ptr server) {
  bb->set<ayanami::components::NavigationRuntime::Ptr>(k_navigation_runtime,
                                                       server);
}

void setRequiredEntries(const BT::Blackboard::Ptr& bb) {
  bb->set<PoseStamped>(k_goal, makePose(1.0, 1.0));
  bb->set<Path>(k_raw_path, Path{});
  bb->set<Path>(k_smoothed_path, Path{});
  bb->set<double>(k_planner_frequency, -1.0);
  bb->set<double>(k_smoother_frequency, -1.0);
  bb->set<double>(k_controller_frequency, 20.0);
  bb->set<std::string>(k_clear_costmap_id, "clear_costmap");
  bb->set<std::string>(k_spin_id, "spin");
  bb->set<std::string>(k_wait_id, "wait");
  bb->set<std::string>(k_backup_id, "back_up");
  bb->set<uint64_t>(k_session_id, 0u);
  bb->set<uint64_t>(k_raw_path_revision, 0u);
  bb->set<uint64_t>(k_goal_revision, 0u);
  bb->set<ayanami::FailureContext>(k_failure_context, ayanami::FailureContext{});
}

// Build a tree isomorphic to the real navigation tree (the embedded production default tree) with the outer RecoveryNode
// retries=2. The RateController is removed (its tick gating caches RUNNING, unsuitable for this test tight tick loop);
// the recovery chain structure (PipelineSequence + ReactiveFallback) is kept as-is.
constexpr const char* k_exhaustion_tree_xml = R"(
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
        <RecoverySequence name="RecoveryActions">
          <Sequence><Recovery recovery="{spin_id}"/>    <AlwaysFailure/></Sequence>
          <Sequence><Recovery recovery="{wait_id}"/>    <AlwaysFailure/></Sequence>
          <Sequence><Recovery recovery="{backup_id}"/>  <AlwaysFailure/></Sequence>
        </RecoverySequence>
      </ReactiveFallback>
    </RecoveryNode>
  </BehaviorTree>
</root>)";

}  // namespace

// ---- Component layer: RecoveryAction reports failure against an always-failing recovery (re-creates legacy) ----

TEST(RecoveryExhaustion, recoveryActionReportsFailure) {
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
  // The failure is conveyed via error (unified failure code INTERNAL_ERROR)
  EXPECT_EQ(task.result().error, ErrorCode::INTERNAL_ERROR);
  EXPECT_EQ(recovery->calls.load(), 1);
}

TEST(RecoveryExhaustion, multipleRecoveryAttemptsAllFail) {
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
    EXPECT_EQ(task.result().error, ErrorCode::INTERNAL_ERROR);
  }
  EXPECT_EQ(recovery->calls.load(), 3);
}

// ---- BT layer: the recovery chain fully exhausted -> whole tree FAILURE, no endless retries ----

TEST(RecoveryExhaustion, recoveryListExhaustedFailsNavigationWithinBounds) {
  auto server = std::make_shared<FailingNavServer>();

  auto bb = BT::Blackboard::create();
  setNavServer(bb, server);
  setRequiredEntries(bb);

  BT::BehaviorTreeFactory factory;
  registerNavigationBehaviorTreeNodes(factory);
  auto tree = factory.createTreeFromText(k_exhaustion_tree_xml, bb);

  // Bounded ticking until root FAILURE: endless retries would stay RUNNING forever and trip this assertion.
  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  const int k_max_ticks = 60;
  int ticks = 0;
  for (; ticks < k_max_ticks && status != BT::NodeStatus::FAILURE; ++ticks) {
    status = tree.tickExactlyOnce();
  }
  ASSERT_EQ(status, BT::NodeStatus::FAILURE) << "tree must return FAILURE after the recovery chain is exhausted";
  EXPECT_LE(ticks, k_max_ticks);

  // The planner was called and failed
  EXPECT_GE(server->path_calls.load(), 1);

  // The recovery chain list was fully traversed: clear_costmap (inner) + spin/wait/back_up (outer
  // ReactiveFallback) were each requested
  EXPECT_GE(server->recoveryRequests("clear_costmap"), 1);
  EXPECT_GE(server->recoveryRequests("spin"), 1);
  EXPECT_GE(server->recoveryRequests("wait"), 1);
  EXPECT_GE(server->recoveryRequests("back_up"), 1);
  EXPECT_LE(server->recoveryRequests("spin"), 2);
  EXPECT_LE(server->recoveryRequests("wait"), 2);
  EXPECT_LE(server->recoveryRequests("back_up"), 2);
}
