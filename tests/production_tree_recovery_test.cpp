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
// Transient-failure variant: the second execution succeeds -> whole tree SUCCESS; recovery only takes the clear_costmap
// path.
//
// Debugging: AYANAMI_BT_TRACE=1 prints per-node status transitions.

#include <gtest/gtest.h>

#include "test_transform_source.hpp"

#include <ayanami/behavior_tree/action_nodes.hpp>
#include <ayanami/behavior_tree/blackboard_protocol.hpp>
#include <ayanami/behavior_tree/navigation.hpp>
#include <ayanami/abstract/components/navigation_server_base.hpp>
#include <ayanami/abstract/components/robot_state_controller.hpp>
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

using ayanami::testing::testTransforms;
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

/// @brief Programmable deferred completion: requests enter a pending queue; pump advances their age and settles them
/// per policy.
class MockRuntime final : public NavigationRuntime {
public:
  // ---- Observation ----
  int path_requests{0};
  int smooth_requests{0};
  int exe_requests{0};
  /// @brief Pose count of every submitted FollowPath goal (raw vs smoothed
  /// discrimination for the degrade contract).
  std::vector<std::size_t> exe_path_sizes;
  int recovery_requests{0};
  std::map<std::string, int> recovery_by_id;

  // ---- Policy ----
  int settle_after{2};  // settled after this many pumps
  std::vector<std::shared_ptr<TaskState<ExePathResult>>> exe_states;
  bool controller_always_fails{true};
  /// @brief Per-attempt settle policy (equivalence harness): when > 0, the
  /// first fail_first_exe settled executions of an attempt fail with
  /// NO_PROGRESS and the rest succeed (exe_settle_count resets per attempt).
  /// Takes precedence over controller_always_fails when set.
  int fail_first_exe{0};
  int exe_settle_count{0};
  /// @brief The FollowPath task NEVER completes (stays RUNNING): the
  /// oscillation-gate test needs a long-lived attempt to sample.
  bool controller_never_settles{false};
  /// @brief Report the smoothed path as invalid (overlay rejection) while the
  /// raw path stays valid. The two are told apart by pose count, so the
  /// smoother branch's IsPathValid can be exercised without also failing the
  /// planner branch's raw-path check.
  bool smoothed_path_invalid{false};

  // ---- Session/version ----
  std::atomic<uint64_t> session{1};
  std::atomic<uint64_t> plan_version{0};

  /// @brief Drop unfinished tasks (the harness calls it at an attempt
  /// boundary; the real server winds tasks down on their own threads).
  void dropPending() { pending.clear(); }

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

  TaskHandle<GetPathResult> requestPath(const ayanami::actions::GetPathGoal&) override {
    ++path_requests;
    auto state = std::make_shared<TaskState<GetPathResult>>();
    pending.push_back({0, [this, state] { settlePath(state); }});
    return TaskHandle<GetPathResult>(1, state, [] { return false; });
  }

  TaskHandle<SmoothPathResult> requestSmoothing(const ayanami::actions::SmoothPathGoal&) override {
    ++smooth_requests;
    auto state = std::make_shared<TaskState<SmoothPathResult>>();
    pending.push_back({0, [this, state] { settleSmooth(state); }});
    return TaskHandle<SmoothPathResult>(1, state, [] { return false; });
  }

  TaskHandle<ExePathResult> requestExecution(const ayanami::actions::ExePathGoal& goal) override {
    ++exe_requests;
    exe_path_sizes.push_back(goal.path.poses.size());
    auto state = std::make_shared<TaskState<ExePathResult>>();
    if (!controller_never_settles) {
      pending.push_back({0, [this, state] { settleExe(state); }});
    }
    exe_states.push_back(state);
    // A cancel settles the task CANCELED (the recovery dispatch's
    // preemptAllControllers signature move; the production
    // ControllerExecution does the same through cancel_requested).
    return TaskHandle<ExePathResult>(1, state, [state] {
      typename TaskState<ExePathResult>::Completion c;
      c.status = ayanami::actions::GoalStatus::CANCELED;
      c.result.error = ErrorCode::CANCELED;
      state->complete(c);
      return true;
    });
  }

  /// @brief Simulate a recovery dispatch's controller preemption: the
  /// in-flight exe task settles CANCELED out from under the RUNNING node.
  void cancelExe() {
    ASSERT_FALSE(exe_states.empty());
    typename TaskState<ExePathResult>::Completion c;
    c.status = ayanami::actions::GoalStatus::CANCELED;
    c.result.error = ErrorCode::CANCELED;
    exe_states.back()->complete(c);
  }

  TaskHandle<RecoveryResult> requestRecovery(const ayanami::actions::RecoveryGoal& goal) override {
    ++recovery_requests;
    ++recovery_by_id[goal.recovery_id];
    auto state = std::make_shared<TaskState<RecoveryResult>>();
    pending.push_back({0, [this, state] { settleRecovery(state); }});
    return TaskHandle<RecoveryResult>(1, state, [] { return false; });
  }

  uint64_t currentSessionId() const override { return session.load(); }
  uint64_t currentPlanVersion() const override { return plan_version.load(); }
  bool sessionIsCurrent(uint64_t id) const override { return id == session.load(); }
  bool planVersionIsCurrent(uint64_t v) const override { return v == plan_version.load(); }
  void incrementPlanVersion() override { ++plan_version; }
  bool isPathValid(const ayanami::nav_msgs::Path& path) const override {
    if (smoothed_path_invalid && path.poses.size() == k_smoothed_poses) {
      return false;
    }
    return !path.poses.empty();
  }
  bool requestPlanUpdate(uint64_t, const ayanami::actions::ExePathGoal&) override { return false; }
  void setSessionState(std::shared_ptr<ayanami::components::NavigationSessionState>) override {}

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
    // Aligned with production: the result carries the version "at accept time"; only after the node onRunning check
    // passes does incrementPlanVersion run (via the mock incrementPlanVersion).
    stamp(c.result);
    state->complete(c);
  }

  void settleExe(std::shared_ptr<TaskState<ExePathResult>> state) {
    typename TaskState<ExePathResult>::Completion c;
    bool fail = controller_always_fails;
    if (fail_first_exe > 0) {
      fail = exe_settle_count < fail_first_exe;
      ++exe_settle_count;
    }
    if (fail) {
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
    bb->set<ayanami::FailureContext>(k_failure_context, ayanami::FailureContext{});
    bb->set<uint32_t>(k_nav_retries, 6u);
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

  /// @brief Pilot applyPendingGoal parity for a NEW attempt on the RESIDENT
  /// tree: the revision advances (monotonic, never reset -- the change signal
  /// the token-keyed nodes key their caches off), the per-goal keys are
  /// rewritten and the finished attempt's path artifacts are dropped.
  void beginAttempt() {
    ++attempt_revision;
    bb->set<uint64_t>(k_goal_revision, attempt_revision);
    bb->set<uint64_t>(k_raw_path_revision, 0u);
    bb->set<uint64_t>(k_session_id, runtime->currentSessionId());
    bb->set<ayanami::FailureContext>(k_failure_context, ayanami::FailureContext{});
    resetAttemptBlackboard(bb);
    tree.haltTree();  // defensive (pilot parity); every halt() is idempotent
    runtime->dropPending();
  }

  /// @brief Test mirror of the pilot contract (nav2 BtActionServer model):
  /// the tree is RESIDENT -- built once, and a terminal status HALTS it
  /// (every node back to IDLE, halt() overrides clear cached state) instead
  /// of dropping it; the next tick re-arms the per-attempt blackboard keys
  /// and re-enters on the same object. The assertions in the tests below are
  /// unchanged, which is the point: the observable behaviour (replan +
  /// re-enter FollowPath after recovery, bounded budget) is the same.
  BT::NodeStatus tickTree() {
    if (!instance_alive) {
      if (bb == nullptr) {
        buildTree();  // first attempt: one-time tree + blackboard build
      } else {
        beginAttempt();  // re-arm the resident tree for the next attempt
        instance_alive = true;
      }
    }
    const BT::NodeStatus status = tree.tickExactlyOnce();
    if (status != BT::NodeStatus::RUNNING) {
      tree.haltTree();
      instance_alive = false;
    }
    return status;
  }

  uint64_t attempt_revision{0};

  BT::BehaviorTreeFactory factory;
  std::shared_ptr<MockRuntime> runtime;
  BT::Blackboard::Ptr bb;
  BT::Tree tree;
  bool instance_alive = false;
  std::unique_ptr<BT::StdCoutLogger> logger;
};

// ---- I1/I2: controller permanent NoProgress -> must reenter after recovery + stay bounded ----
TEST_F(ProductionTreeRecovery, controllerFailureMustReenterFollowPathWithBoundedRecoveries) {
  writeBlackboard();
  runtime->controller_always_fails = true;
  buildTree();

  for (int i = 0; i < 200; ++i) {
    tickTree();
    runtime->pump();
  }

  GTEST_SUCCEED() << "path=" << runtime->path_requests << " smooth=" << runtime->smooth_requests
                  << " exe=" << runtime->exe_requests << " recovery=" << runtime->recovery_requests
                  << " spin=" << runtime->recovery_by_id["spin"] << " backup=" << runtime->recovery_by_id["backup"];

  // I1: after a successful recovery FollowPath must reenter (production bug: exe stayed constant at 1).
  EXPECT_GE(runtime->exe_requests, 2) << "FollowPath was never re-entered after recovery (storm bug)";
  // I1b: the planner must replan within the restarted tree lifetime (production bug: path stayed constant at 1).
  EXPECT_GE(runtime->path_requests, 2) << "planner never replanned across tree lives";
  // I2: bounded retry budget -- nav2-equivalent budget: per tree life = 6 outer retries, each retry at most
  // 1 outer recovery + 1 inner clear_costmap = 12; every tree life is accompanied by a replan,
  // so the recovery count must be bounded relative to the plan count (storm bug: path stuck at 1 while recoveries were
  // unbounded).
  EXPECT_LE(runtime->recovery_requests, 13 * std::max(1, runtime->path_requests))
    << "recovery storm: unbounded recoveries per tree life";
}

// ---- Navigation-level oscillation: every component succeeds, the robot
// never moves -> the IsRobotProgressing gate fails the pipeline and the
// recovery ladder fires WITHOUT any planner/controller failure. ----
TEST_F(ProductionTreeRecovery, navigationLevelOscillationTriggersRecoveryLadder) {
  runtime->controller_always_fails = false;
  runtime->controller_never_settles = true;
  runtime->settle_after = 1;
  buildTree();
  // AFTER buildTree: writeBlackboard recreates the blackboard, so the
  // attempt-specific parameters land on the live instance.
  bb->set<uint32_t>(k_nav_retries, 2u);
  bb->set<double>(k_oscillation_distance, 0.1);
  bb->set<double>(k_oscillation_timeout, 0.2);

  // A robot pose that never changes: the very definition of "going
  // nowhere" for the gate.
  auto& rsc = ayanami::components::RobotStateController::instance();
  testTransforms().set("odom", "base_link", ayanami::Timestamp::latest(), Eigen::Isometry3d::Identity());
  testTransforms().set("map", "odom", ayanami::Timestamp::latest(), Eigen::Isometry3d::Identity());
  {
    PoseStamped probe;
    ayanami::geometry_msgs::TwistStamped vel;
    ASSERT_TRUE(rsc.getRobotState(probe, vel)) << "robot pose must resolve for the oscillation gate";
  }

  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  for (int i = 0; i < 400 && status == BT::NodeStatus::RUNNING; ++i) {
    status = tickTree();
    runtime->pump();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  // The attempt terminates in FAILURE (the retry budget exhausts against a
  // robot that never makes progress).
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
  // The ladder fired on the gate alone: planner/controller never failed.
  EXPECT_EQ(runtime->exe_requests, 1) << "FollowPath should have kept running (never_settles)";
  EXPECT_GE(runtime->recovery_requests, 2) << "oscillation gate never dispatched the recovery ladder";
  EXPECT_GE(runtime->recovery_requests, 2) << "the ladder should have rotated through the recoveries";
  // The ladder leads with the cheapest recovery: a costmap clear (nav2
  // ClearingActions first), then spin/wait/backup.
  EXPECT_GE(runtime->recovery_by_id["clear_costmap"], 1) << "the recovery ladder must try the costmap clear first";
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
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS) << "path=" << runtime->path_requests << " exe=" << runtime->exe_requests
                                             << " recovery=" << runtime->recovery_requests;
  // Recovery budget: 1 clear_costmap in the inner FollowPath branch (the outer must not fire).
  EXPECT_LE(runtime->recovery_requests, 1) << "recovery=" << runtime->recovery_requests;
}

// ---- Smoothed-path validation parity: the tree carries NO smoothed-path IsPathValid ----
// Rationale: the smoother module gates its own output on the fused overlay
// (global band + local marks at solve time) -- the same terms the raw path
// is validated on at plan time. A BT-level re-validation against the LIVE
// local map at 20Hz (while the followed raw path is never re-judged) let
// transient perception marks reject every smoothing attempt near obstacles:
// the output never landed and the branch flapped to raw forever (2026-09-30
// forensics). Contract under test: an "overlay-invalid" smoothed path still
// reaches FollowPath and the attempt completes with ZERO recoveries -- a
// smoothing verdict never escalates.
TEST_F(ProductionTreeRecovery, invalidSmoothedPathStillNavigatesWithoutEscalation) {
  writeBlackboard();
  runtime->smoothed_path_invalid = true;
  runtime->controller_always_fails = false;  // isolate: the controller itself is healthy
  buildTree();

  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  for (int i = 0; i < 200; ++i) {
    status = tickTree();
    runtime->pump();
    if (status != BT::NodeStatus::RUNNING) {
      break;
    }
  }

  EXPECT_EQ(status, BT::NodeStatus::SUCCESS)
      << "path=" << runtime->path_requests << " smooth=" << runtime->smooth_requests
      << " exe=" << runtime->exe_requests;
  EXPECT_GE(runtime->exe_requests, 1) << "FollowPath never started";
  EXPECT_EQ(runtime->recovery_requests, 0)
      << "a smoothing verdict must never escalate into the recovery branch";
  EXPECT_GE(runtime->smooth_requests, 1);
  EXPECT_GE(runtime->path_requests, 1);
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
    << "path=" << runtime->path_requests << " smooth=" << runtime->smooth_requests << " exe=" << runtime->exe_requests;
  EXPECT_GE(runtime->exe_requests, 1);
}

// ---- Framework-preemption resume: a recovery dispatch cancels the
// running controller (preemptAllControllers); the FollowPath node must
// treat the CANCELED result as "canceled out from under me" and RESUME the
// same goal in place -- not report a failure (which used to cascade into
// the stop -> stale-failure -> retry storm). ----
TEST_F(ProductionTreeRecovery, canceledControllerTaskResumesInsteadOfFailing) {
  writeBlackboard();
  runtime->controller_always_fails = false;
  buildTree();

  // Tick until FollowPath has submitted its task.
  for (int i = 0; i < 50 && runtime->exe_requests < 1; ++i) {
    tickTree();
    runtime->pump();
  }
  ASSERT_GE(runtime->exe_requests, 1) << "FollowPath never started";

  // Preemption signature: the in-flight exe task settles CANCELED.
  runtime->cancelExe();

  // The node drops the canceled task and RESUBMITS (exe_requests 2) while
  // staying RUNNING -- no failure, no recovery dispatch.
  for (int i = 0; i < 50 && runtime->exe_requests < 2; ++i) {
    tickTree();
    runtime->pump();
  }
  EXPECT_GE(runtime->exe_requests, 2)
      << "a canceled controller task must be resumed in place";
  EXPECT_EQ(runtime->recovery_requests, 0)
      << "a framework preemption must not trigger any recovery";

  // The resumed task completes successfully: the whole tree SUCCEEDS.
  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  for (int i = 0; i < 100 && status == BT::NodeStatus::RUNNING; ++i) {
    status = tickTree();
    runtime->pump();
  }
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

// ---- Resident-tree equivalence guard: the same scenario run N times on ONE
// resident tree (the pilot model: halt + per-attempt blackboard reset between
// attempts) must produce the SAME per-attempt request transcript as N fresh
// trees (the former per-instance model). Any node state that leaks across
// attempts on the resident tree -- a RateController cache, a RecoveryNode
// budget, a RecoverySequence cursor, a stale blackboard artifact -- diverges
// the transcripts. ----
namespace {

struct AttemptTranscript {
  int path{0};
  int smooth{0};
  int exe{0};
  int recovery{0};
  std::map<std::string, int> recovery_by_id;
  BT::NodeStatus final_status{BT::NodeStatus::IDLE};
};

/// @brief Drives navigation attempts against one MockRuntime with the shipped
/// production tree. Scenario per attempt: the controller's first TWO
/// executions fail with NO_PROGRESS, the rest succeed -- exercising the inner
/// RecoveryNode (clear_costmap), the outer recovery ladder entry, the planner
/// re-plan and the FollowPath re-entry, then a SUCCESS.
class AttemptRunner {
public:
  explicit AttemptRunner(bool resident) : resident_(resident) {
    registerNavigationBehaviorTreeNodes(factory_);
    factory_.registerBehaviorTreeFromFile(AYANAMI_TEST_BT_XML);
    runtime_ = std::make_shared<MockRuntime>();
    runtime_->controller_always_fails = false;
    runtime_->fail_first_exe = 2;
    runtime_->settle_after = 1;
  }

  void start() {
    makeBlackboard();
    tree_ = factory_.createTree("NavigateWithReplanningAndSmoothing", bb_);
    alive_ = true;
  }

  AttemptTranscript runAttempt() {
    const AttemptTranscript before = snapshot();
    if (!resident_) {
      // Former per-instance model: every attempt is a fresh tree object on a
      // fresh blackboard.
      makeBlackboard();
      tree_ = factory_.createTree("NavigateWithReplanningAndSmoothing", bb_);
    } else if (!alive_) {
      beginAttempt();  // re-arm the resident tree (pilot applyPendingGoal parity)
    }
    runtime_->exe_settle_count = 0;

    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    for (int i = 0; i < 300 && status == BT::NodeStatus::RUNNING; ++i) {
      status = tree_.tickExactlyOnce();
      runtime_->pump();
    }
    EXPECT_NE(status, BT::NodeStatus::RUNNING) << "attempt did not terminate";
    if (status != BT::NodeStatus::RUNNING) {
      tree_.haltTree();  // pilot parity: halt the tree at the terminal status
    }

    AttemptTranscript t = snapshot();
    t.final_status = status;
    t.path -= before.path;
    t.smooth -= before.smooth;
    t.exe -= before.exe;
    t.recovery -= before.recovery;
    for (auto& [id, n] : t.recovery_by_id) {
      n -= before.recovery_by_id.count(id) ? before.recovery_by_id.at(id) : 0;
    }
    if (resident_) {
      alive_ = (status == BT::NodeStatus::RUNNING);
    } else {
      tree_ = BT::Tree{};  // the former model drops the instance
    }
    return t;
  }

private:
  void makeBlackboard() {
    bb_ = BT::Blackboard::create();
    bb_->set<NavigationRuntime::Ptr>(k_navigation_runtime, runtime_);
    bb_->set<PoseStamped>(k_goal, makeGoal());
    bb_->set<Path>(k_raw_path, Path{});
    bb_->set<Path>(k_smoothed_path, Path{});
    bb_->set<double>(k_planner_frequency, -1.0);
    bb_->set<double>(k_smoother_frequency, -1.0);
    bb_->set<double>(k_controller_frequency, 20.0);
    bb_->set<std::string>(k_clear_costmap_id, "clear_costmap");
    bb_->set<std::string>(k_spin_id, "spin");
    bb_->set<std::string>(k_wait_id, "wait");
    bb_->set<std::string>(k_backup_id, "backup");
    bb_->set<uint64_t>(k_session_id, runtime_->currentSessionId());
    bb_->set<uint64_t>(k_raw_path_revision, 0u);
    bb_->set<uint64_t>(k_goal_revision, revision_);
    bb_->set<ayanami::FailureContext>(k_failure_context, ayanami::FailureContext{});
    bb_->set<uint32_t>(k_nav_retries, 6u);
  }

  /// @brief Pilot applyPendingGoal parity (new attempt on the resident tree).
  void beginAttempt() {
    ++revision_;
    bb_->set<uint64_t>(k_goal_revision, revision_);
    bb_->set<uint64_t>(k_raw_path_revision, 0u);
    bb_->set<ayanami::FailureContext>(k_failure_context, ayanami::FailureContext{});
    resetAttemptBlackboard(bb_);
    tree_.haltTree();  // defensive (pilot parity)
    runtime_->dropPending();
  }

  AttemptTranscript snapshot() const {
    AttemptTranscript t;
    t.path = runtime_->path_requests;
    t.smooth = runtime_->smooth_requests;
    t.exe = runtime_->exe_requests;
    t.recovery = runtime_->recovery_requests;
    t.recovery_by_id = runtime_->recovery_by_id;
    return t;
  }

  bool resident_{false};
  bool alive_{false};
  uint64_t revision_{0};
  BT::BehaviorTreeFactory factory_;
  std::shared_ptr<MockRuntime> runtime_;
  BT::Blackboard::Ptr bb_;
  BT::Tree tree_;
};

}  // namespace

TEST(ProductionTreeResidentEquivalence, residentTreeMatchesFreshTreesPerAttempt) {
  auto collect = [](bool resident) {
    AttemptRunner runner(resident);
    runner.start();
    std::vector<AttemptTranscript> out;
    for (int i = 0; i < 3; ++i) {
      out.push_back(runner.runAttempt());
    }
    return out;
  };
  const auto resident = collect(/*resident=*/true);
  const auto fresh = collect(/*resident=*/false);
  ASSERT_EQ(resident.size(), fresh.size());
  for (size_t i = 0; i < resident.size(); ++i) {
    EXPECT_EQ(resident[i].final_status, fresh[i].final_status) << "attempt " << i << ": final status diverged";
    EXPECT_EQ(resident[i].path, fresh[i].path) << "attempt " << i << ": path requests diverged";
    EXPECT_EQ(resident[i].smooth, fresh[i].smooth) << "attempt " << i << ": smooth requests diverged";
    EXPECT_EQ(resident[i].exe, fresh[i].exe) << "attempt " << i << ": exe requests diverged";
    EXPECT_EQ(resident[i].recovery, fresh[i].recovery) << "attempt " << i << ": recovery requests diverged";
    EXPECT_EQ(resident[i].recovery_by_id, fresh[i].recovery_by_id) << "attempt " << i << ": recovery ids diverged";
    // The scenario must actually exercise the machinery it claims to:
    // double controller failure -> inner clear + outer ladder entry ->
    // FollowPath re-entry -> SUCCESS. (The planner branch stays on its
    // single-shot cache across the ladder re-entry in this harness --
    // planner_frequency=-1 and a pipeline-FAILURE halt only resets RUNNING
    // children -- so path stays 1 per attempt; the production config runs
    // planner_frequency>0, which re-executes the branch on its period.)
    EXPECT_EQ(resident[i].final_status, BT::NodeStatus::SUCCESS);
    EXPECT_GE(resident[i].path, 1);
    EXPECT_GE(resident[i].exe, 3);
    EXPECT_GE(resident[i].recovery, 2);
    const auto clear_it = resident[i].recovery_by_id.find("clear_costmap");
    ASSERT_NE(clear_it, resident[i].recovery_by_id.end());
    EXPECT_GE(clear_it->second, 2);
  }
}

}  // namespace

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
