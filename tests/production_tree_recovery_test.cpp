// production_tree_recovery_test: recovery reentry integration test using the production default tree
// (NavigateWithReplanningAndSmoothing) + a programmable MockNavigationRuntime.
//
// Background bug (observed live in rviz + rosbag evidence): after a controller NoProgress ABORT, the outer
// RecoveryActions ran 28 consecutive recoveries (a spin storm) while FollowPath never reentered, and
// 28 > the outer number_of_retries=6 -- both recovery reentry and the retry cap had failed; the robot spun in place
// forever and never resumed navigation.
//
// Using production-identical tree XML + a pilot-style tick loop, this test locks two invariants:
// I1 (reentry): after a controller failure + a successful recovery, FollowPath must be requested again
// (exe_request_count >= 2), and the planner must replan after the tree restarts;
// I2 (bounded): within a single tree lifetime the recovery request count <= the sum of the inner+outer retry budgets.
// Transient-failure variant: the second execution succeeds -> whole tree SUCCESS; recovery only takes the clear_costmap path.
//
// Debugging: AYANAMI_BT_TRACE=1 prints per-node status transitions.

#include <gtest/gtest.h>

#include <ayanami/behavior_tree/action_nodes.hpp>
#include <ayanami/behavior_tree/blackboard_protocol.hpp>
#include <ayanami/behavior_tree/navigation.hpp>
#include <ayanami/abstract/components/navigation_server_base.hpp>
#include <ayanami/messages/actions.hpp>
#include <ayanami/types.hpp>

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>

#include <algorithm>
#include <atomic>
#include <cstdlib>
#include <functional>
#include <memory>
#include <string>
#include <vector>

using namespace ayanami::behavior_tree;
using ayanami::ErrorCode;
using ayanami::actions::ExePathResult;
using ayanami::actions::GetPathResult;
using ayanami::actions::RecoveryResult;
using ayanami::actions::SmoothPathResult;
using ayanami::components::NavigationRuntime;
using ayanami::components::TaskHandle;
using ayanami::components::TaskState;
using ayanami::geometry_msgs::PoseStamped;
using ayanami::nav_msgs::Path;

namespace {

// Verbatim identical to the production tree embedded in navigation_behavior_tree.hpp.

PoseStamped makeGoal() {
  PoseStamped p;
  p.header.frame_id = "map";
  p.pose.position = Eigen::Vector3d(-7.0, -3.0, 0.0);
  return p;
}

Path makePath() {
  Path p;
  p.header.frame_id = "map";
  for (int i = 0; i < 4; ++i) {
    PoseStamped pt;
    pt.header.frame_id = "map";
    pt.pose.position = Eigen::Vector3d(-1.0 * i, 0.0, 0.0);
    p.poses.push_back(pt);
  }
  return p;
}

/// @brief The smoothed path carries a distinct pose count so MockRuntime can
/// tell it apart from the raw path when applying per-path validity policy.
constexpr std::size_t k_smoothed_poses = 2;

Path makeSmoothedPath() {
  Path p;
  p.header.frame_id = "map";
  for (std::size_t i = 0; i < k_smoothed_poses; ++i) {
    PoseStamped pt;
    pt.header.frame_id = "map";
    pt.pose.position = Eigen::Vector3d(-1.0 * static_cast<double>(i), 0.5, 0.0);
    p.poses.push_back(pt);
  }
  return p;
}

/// @brief Programmable deferred completion: requests enter a pending queue; pump advances their age and settles them per policy.
class MockRuntime final : public NavigationRuntime {
 public:
  // ---- Observation ----
  int path_requests{0};
  int smooth_requests{0};
  int exe_requests{0};
  int recovery_requests{0};
  std::map<std::string, int> recovery_by_id;

  // ---- Policy ----
  int settle_after{2};  // settled after this many pumps
  bool controller_always_fails{true};
  /// @brief Report the smoothed path as invalid (overlay rejection) while the
  /// raw path stays valid. The two are told apart by pose count, so the
  /// smoother branch's IsPathValid can be exercised without also failing the
  /// planner branch's raw-path check.
  bool smoothed_path_invalid{false};

  // ---- Session/version ----
  std::atomic<uint64_t> session{1};
  std::atomic<uint64_t> plan_version{0};

  void pump() {
    std::vector<Pending> still;
    for (auto& p : pending) {
      Pending item = std::move(p);
      if (++item.age < settle_after) {
        still.push_back(std::move(item));
        continue;
      }
      item.settle();
    }
    pending.swap(still);
  }

  TaskHandle<GetPathResult> requestPath(
      const ayanami::actions::GetPathGoal&) override {
    ++path_requests;
    auto state = std::make_shared<TaskState<GetPathResult>>();
    pending.push_back({0, [this, state] { settlePath(state); }});
    return TaskHandle<GetPathResult>(
        1, state, [] { return false; });
  }

  TaskHandle<SmoothPathResult> requestSmoothing(
      const ayanami::actions::SmoothPathGoal&) override {
    ++smooth_requests;
    auto state = std::make_shared<TaskState<SmoothPathResult>>();
    pending.push_back({0, [this, state] { settleSmooth(state); }});
    return TaskHandle<SmoothPathResult>(
        1, state, [] { return false; });
  }

  TaskHandle<ExePathResult> requestExecution(
      const ayanami::actions::ExePathGoal&) override {
    ++exe_requests;
    auto state = std::make_shared<TaskState<ExePathResult>>();
    pending.push_back({0, [this, state] { settleExe(state); }});
    return TaskHandle<ExePathResult>(1, state, [] { return false; });
  }

  TaskHandle<RecoveryResult> requestRecovery(
      const ayanami::actions::RecoveryGoal& goal) override {
    ++recovery_requests;
    ++recovery_by_id[goal.recovery_id];
    auto state = std::make_shared<TaskState<RecoveryResult>>();
    pending.push_back({0, [this, state] { settleRecovery(state); }});
    return TaskHandle<RecoveryResult>(1, state, [] { return false; });
  }

  uint64_t currentSessionId() const override { return session.load(); }
  uint64_t currentPlanVersion() const override { return plan_version.load(); }
  bool sessionIsCurrent(uint64_t id) const override {
    return id == session.load();
  }
  bool planVersionIsCurrent(uint64_t v) const override {
    return v == plan_version.load();
  }
  void incrementPlanVersion() override { ++plan_version; }
  bool isPathValid(const ayanami::nav_msgs::Path& path) const override {
    if (smoothed_path_invalid && path.poses.size() == k_smoothed_poses) {
      return false;
    }
    return !path.poses.empty();
  }
  bool requestPlanUpdate(
      uint64_t, const ayanami::actions::ExePathGoal&) override {
    return false;
  }
  void cancelAllControllers() override {}
  void setSessionState(
      std::shared_ptr<ayanami::components::NavigationSessionState>) override {}

 private:
  struct Pending {
    int age{0};
    std::function<void()> settle;
  };
  std::vector<Pending> pending;


  void stamp(ayanami::actions::GetPathResult& r) {
    r.session_id = session.load();
    r.plan_version = plan_version.load();
  }
  void stamp(ayanami::actions::SmoothPathResult& r) {
    r.session_id = session.load();
    r.plan_version = plan_version.load();
  }
  void stamp(ayanami::actions::ExePathResult& r) {
    r.session_id = session.load();
    r.plan_version = plan_version.load();
  }
  void stamp(ayanami::actions::RecoveryResult& r) {
    r.session_id = session.load();
    r.plan_version = plan_version.load();
  }

  void settlePath(std::shared_ptr<TaskState<GetPathResult>> state) {
    typename TaskState<GetPathResult>::Completion c;
    c.status = ayanami::actions::GoalStatus::SUCCEEDED;
    c.result.path = makePath();
    stamp(c.result);
    state->complete(c);
  }

  void settleSmooth(std::shared_ptr<TaskState<SmoothPathResult>> state) {
    typename TaskState<SmoothPathResult>::Completion c;
    c.status = ayanami::actions::GoalStatus::SUCCEEDED;
    c.result.smoothed_path = makeSmoothedPath();
    // Aligned with production: the result carries the version "at accept time"; only after the node onRunning check passes
    // does incrementPlanVersion run (via the mock incrementPlanVersion).
    stamp(c.result);
    state->complete(c);
  }

  void settleExe(std::shared_ptr<TaskState<ExePathResult>> state) {
    typename TaskState<ExePathResult>::Completion c;
    if (controller_always_fails) {
      c.status = ayanami::actions::GoalStatus::ABORTED;
      c.result.error = ErrorCode::NO_PROGRESS;
      c.result.message = "No progress made executing the path";
      c.result.reached = false;
    } else {
      c.status = ayanami::actions::GoalStatus::SUCCEEDED;
      c.result.error = ErrorCode::NONE;
      c.result.reached = true;
    }
    stamp(c.result);
    state->complete(c);
  }

  void settleRecovery(std::shared_ptr<TaskState<RecoveryResult>> state) {
    typename TaskState<RecoveryResult>::Completion c;
    c.status = ayanami::actions::GoalStatus::SUCCEEDED;
    c.result.error = ErrorCode::NONE;
    stamp(c.result);
    state->complete(c);
  }
};

class ProductionTreeRecovery : public ::testing::Test {
 protected:
  void SetUp() override {
    ayanami::behavior_tree::registerNavigationBehaviorTreeNodes(factory);
    // Load the SHIPPED tree, not a copy: the point of this test is to catch
    // production regressions (the historical "planner never replans + recovery
    // storm"), so it must run the policy the robot runs. A local copy would
    // drift and quietly keep passing.
    factory.registerBehaviorTreeFromFile(AYANAMI_TEST_BT_XML);
    runtime = std::make_shared<MockRuntime>();
  }

  void writeBlackboard() {
    bb = BT::Blackboard::create();
    bb->set<NavigationRuntime::Ptr>(k_navigation_runtime, runtime);
    bb->set<PoseStamped>(k_goal, makeGoal());
    bb->set<Path>(k_raw_path, Path{});
    bb->set<Path>(k_smoothed_path, Path{});
    // Production config (costmap_navigation.json): planner/smoother both -1 (single-shot caching)
    bb->set<double>(k_planner_frequency, -1.0);
    bb->set<double>(k_smoother_frequency, -1.0);
    bb->set<double>(k_controller_frequency, 20.0);
    bb->set<std::string>(k_clear_costmap_id, "clear_costmap");
    bb->set<std::string>(k_spin_id, "spin");
    bb->set<std::string>(k_wait_id, "wait");
    bb->set<std::string>(k_backup_id, "backup");
    bb->set<uint64_t>(k_session_id, runtime->currentSessionId());
    bb->set<uint64_t>(k_raw_path_revision, 0u);
    bb->set<uint64_t>(k_goal_revision, 0u);
    bb->set<ayanami::FailureContext>(k_failure_context,
                                     ayanami::FailureContext{});
  }

  /// @brief Build this attempt's tree instance (fresh object + blackboard).
  void buildTree() {
    writeBlackboard();
    tree = factory.createTree("NavigateWithReplanningAndSmoothing", bb);
    instance_alive = true;
    if (std::getenv("AYANAMI_BT_TRACE") != nullptr) {
      logger = std::make_unique<BT::StdCoutLogger>(tree);
    }
  }

  /// @brief Test mirror of the pilot contract: one tree INSTANCE per navigation
  /// attempt. A terminal status ends the attempt, so the next tick starts a
  /// fresh instance with a fresh blackboard -- exactly what BehaviorTreePilot
  /// does (releaseTreeInstance + createTreeInstance). The former model kept one
  /// tree alive and walked it with a full logical reset to emulate this; the
  /// assertions below are unchanged, which is the point: the observable
  /// behaviour (replan + re-enter FollowPath after recovery, bounded budget) is
  /// the same, but nothing has to be reset for it to hold.
  BT::NodeStatus tickTree() {
    if (!instance_alive) {
      buildTree();
    }
    const BT::NodeStatus status = tree.tickExactlyOnce();
    if (status != BT::NodeStatus::RUNNING) {
      tree = BT::Tree{};
      instance_alive = false;
    }
    return status;
  }

  BT::BehaviorTreeFactory factory;
  std::shared_ptr<MockRuntime> runtime;
  BT::Blackboard::Ptr bb;
  BT::Tree tree;
  bool instance_alive = false;
  std::unique_ptr<BT::StdCoutLogger> logger;
};

// ---- I1/I2: controller permanent NoProgress -> must reenter after recovery + stay bounded ----
TEST_F(ProductionTreeRecovery,
       controllerFailureMustReenterFollowPathWithBoundedRecoveries) {
  writeBlackboard();
  runtime->controller_always_fails = true;
  buildTree();

  for (int i = 0; i < 200; ++i) {
    tickTree();
    runtime->pump();
  }

  GTEST_SUCCEED() << "path=" << runtime->path_requests
                  << " smooth=" << runtime->smooth_requests
                  << " exe=" << runtime->exe_requests
                  << " recovery=" << runtime->recovery_requests
                  << " spin=" << runtime->recovery_by_id["spin"]
                  << " backup=" << runtime->recovery_by_id["backup"];

  // I1: after a successful recovery FollowPath must reenter (production bug: exe stayed constant at 1).
  EXPECT_GE(runtime->exe_requests, 2)
      << "FollowPath was never re-entered after recovery (storm bug)";
  // I1b: the planner must replan within the restarted tree lifetime (production bug: path stayed constant at 1).
  EXPECT_GE(runtime->path_requests, 2)
      << "planner never replanned across tree lives";
  // I2: bounded retry budget -- nav2-equivalent budget: per tree life = 6 outer retries, each retry at most
  // 1 outer recovery + 1 inner clear_costmap = 12; every tree life is accompanied by a replan,
  // so the recovery count must be bounded relative to the plan count (storm bug: path stuck at 1 while recoveries were unbounded).
  EXPECT_LE(runtime->recovery_requests,
            13 * std::max(1, runtime->path_requests))
      << "recovery storm: unbounded recoveries per tree life";
}

// ---- Transient failure: the second execution succeeds -> whole tree SUCCESS, the outer recovery must not fire ----
TEST_F(ProductionTreeRecovery, transientControllerFailureRecoversAndSucceeds) {
  writeBlackboard();
  runtime->controller_always_fails = true;  // The first execution fails (NoProgress)
  buildTree();

  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  for (int i = 0; i < 100; ++i) {
    status = tickTree();
    runtime->pump();
    if (runtime->exe_requests >= 1) {
      runtime->controller_always_fails = false;  // Afterwards it always succeeds
    }
    if (status != BT::NodeStatus::RUNNING) {
      break;
    }
  }
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS)
      << "path=" << runtime->path_requests
      << " exe=" << runtime->exe_requests
      << " recovery=" << runtime->recovery_requests;
  // Recovery budget: 1 clear_costmap in the inner FollowPath branch (the outer must not fire).
  EXPECT_LE(runtime->recovery_requests, 1)
      << "recovery=" << runtime->recovery_requests;
}

// ---- Smoothed-path validation: an overlay-invalid smoothed path must never reach FollowPath ----
// Rationale: the smoother's own collision check only sees the LOCAL map (family read contract), so it
// cannot judge the global layer. Before this the tree validated only {raw_path} on the planner branch and
// handed {smoothed_path} straight to FollowPath -- a smoother could push the path into the global map and
// the framework would not notice. The invariant under test is the gate, not the recovery ladder.
TEST_F(ProductionTreeRecovery, invalidSmoothedPathNeverStartsFollowPath) {
  writeBlackboard();
  runtime->smoothed_path_invalid = true;
  runtime->controller_always_fails = false;  // isolate: the controller itself is healthy
  buildTree();

  for (int i = 0; i < 200; ++i) {
    tickTree();
    runtime->pump();
  }

  EXPECT_EQ(runtime->exe_requests, 0)
      << "FollowPath was started on a smoothed path the overlay rejected "
      << "(path=" << runtime->path_requests
      << " smooth=" << runtime->smooth_requests << ")";
  // The smoother branch is what failed, so the tree kept replanning and escalated through recovery.
  EXPECT_GE(runtime->smooth_requests, 1);
  EXPECT_GE(runtime->path_requests, 2) << "planner never replanned after an invalid smoothed path";
  EXPECT_GE(runtime->recovery_requests, 1)
      << "an invalid smoothed path must escalate into the recovery branch";
}

// ---- Control for the test above: the same setup with a valid smoothed path reaches FollowPath and succeeds.
// Without this, "exe_requests == 0" could equally well mean the tree is broken for an unrelated reason. ----
TEST_F(ProductionTreeRecovery, validSmoothedPathReachesFollowPathAndSucceeds) {
  writeBlackboard();
  runtime->smoothed_path_invalid = false;
  runtime->controller_always_fails = false;
  buildTree();

  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  for (int i = 0; i < 100; ++i) {
    status = tickTree();
    runtime->pump();
    if (status != BT::NodeStatus::RUNNING) {
      break;
    }
  }
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS)
      << "path=" << runtime->path_requests << " smooth=" << runtime->smooth_requests
      << " exe=" << runtime->exe_requests;
  EXPECT_GE(runtime->exe_requests, 1);
}

}  // namespace

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
