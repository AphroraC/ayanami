#include <gtest/gtest.h>

#include <ayanami/behavior_tree/blackboard_protocol.hpp>
#include <ayanami/behavior_tree/condition_nodes.hpp>
#include <ayanami/behavior_tree/pipeline_sequence.hpp>
#include <ayanami/behavior_tree/rate_controller.hpp>
#include <ayanami/behavior_tree/recovery_node.hpp>
#include <ayanami/behavior_tree/navigation.hpp>
#include <ayanami/messages/geometry_msgs.hpp>
#include <ayanami/messages/nav_msgs.hpp>

#include <behaviortree_cpp/bt_factory.h>

#include <Eigen/Geometry>

#include "test_transform_source.hpp"

#include <ayanami/abstract/components/robot_state_controller.hpp>

#include <array>
#include <chrono>
#include <functional>
#include <string>
#include <thread>
#include <vector>

using namespace ayanami::behavior_tree;
using ayanami::ErrorCode;
using ayanami::geometry_msgs::PoseStamped;
using ayanami::nav_msgs::Odometry;
using ayanami::nav_msgs::Path;

namespace {

/// Async action stub that can return any NodeStatus (needs RUNNING capability, so it derives from ActionNodeBase rather
/// than SyncActionNode). The test directly controls the tick count and each returned status.
class MockChild : public BT::ActionNodeBase {
public:
  MockChild(const std::string& name, const BT::NodeConfig& config) : BT::ActionNodeBase(name, config) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override {
    ++tick_count;
    return status_fn ? status_fn() : BT::NodeStatus::SUCCESS;
  }

  void halt() override { resetStatus(); }

  int tick_count = 0;
  std::function<BT::NodeStatus()> status_fn;
};

std::vector<MockChild*> collectMocks(BT::Tree& tree) {
  std::vector<MockChild*> out;
  tree.applyVisitor([&out](BT::TreeNode* node) {
    if (auto* mock = dynamic_cast<MockChild*>(node)) {
      out.push_back(mock);
    }
  });
  return out;
}

BT::BehaviorTreeFactory makeFactory() {
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<MockChild>("MockChild");
  return factory;
}

PoseStamped makePose(double x, double y = 0.0) {
  PoseStamped p;
  p.header.frame_id = "map";
  p.pose = ayanami::geometry_msgs::Pose::fromIsometry(Eigen::Isometry3d::Identity());
  p.pose.position = Eigen::Vector3d(x, y, 0.0);
  return p;
}

constexpr const char* k_recovery_xml = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="main">
    <RecoveryNode name="rec" number_of_retries="2">
      <MockChild name="c0"/>
      <MockChild name="c1"/>
    </RecoveryNode>
  </BehaviorTree>
</root>)";

}  // namespace

TEST(BehaviorTreeControlTest, recoveryNodeExhaustsRetriesAfterThreeFailures) {
  auto factory = makeFactory();
  factory.registerNodeType<RecoveryNode>("RecoveryNode");
  auto bb = BT::Blackboard::create();
  auto tree = factory.createTreeFromText(k_recovery_xml, bb);

  auto mocks = collectMocks(tree);
  ASSERT_EQ(mocks.size(), 2u);
  mocks[0]->status_fn = [] { return BT::NodeStatus::FAILURE; };
  mocks[1]->status_fn = [] { return BT::NodeStatus::SUCCESS; };

  // fail #1 -> recovery SUCCESS -> retry, returns RUNNING
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  // fail #2 -> recovery SUCCESS -> retry, returns RUNNING
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  // fail #3 -> retries(2) exhausted -> overall FAILURE
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(mocks[0]->tick_count, 3);
  EXPECT_EQ(mocks[1]->tick_count, 2);
}

TEST(BehaviorTreeControlTest, recoveryNodeRecoveryChildFailsImmediately) {
  auto factory = makeFactory();
  factory.registerNodeType<RecoveryNode>("RecoveryNode");
  auto bb = BT::Blackboard::create();
  auto tree = factory.createTreeFromText(k_recovery_xml, bb);

  auto mocks = collectMocks(tree);
  ASSERT_EQ(mocks.size(), 2u);
  mocks[0]->status_fn = [] { return BT::NodeStatus::FAILURE; };
  mocks[1]->status_fn = [] { return BT::NodeStatus::FAILURE; };

  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(mocks[0]->tick_count, 1);
  EXPECT_EQ(mocks[1]->tick_count, 1);
}

TEST(BehaviorTreeControlTest, pipelineSequenceReticksPlannerWhileControllerRuns) {
  auto factory = makeFactory();
  factory.registerNodeType<PipelineSequence>("PipelineSequence");
  auto bb = BT::Blackboard::create();
  auto tree = factory.createTreeFromText(
    R"(
<root BTCPP_format="4">
  <BehaviorTree ID="main">
    <PipelineSequence name="seq">
      <MockChild name="planner"/>
      <MockChild name="controller"/>
    </PipelineSequence>
  </BehaviorTree>
</root>)",
    bb);

  auto mocks = collectMocks(tree);
  ASSERT_EQ(mocks.size(), 2u);
  mocks[0]->status_fn = [] { return BT::NodeStatus::SUCCESS; };
  mocks[1]->status_fn = [] { return BT::NodeStatus::RUNNING; };

  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(mocks[0]->tick_count, 2);
  EXPECT_EQ(mocks[1]->tick_count, 2);
}

TEST(BehaviorTreeControlTest, recoverySequenceAdvancesOnFailureAndStopsOnSuccess) {
  auto factory = makeFactory();
  factory.registerNodeType<RecoverySequence>("RecoverySequence");
  auto tree = factory.createTreeFromText(
    R"(
<root BTCPP_format="4"><BehaviorTree ID="main">
  <RecoverySequence><MockChild name="first"/><MockChild name="second"/></RecoverySequence>
</BehaviorTree></root>)",
    BT::Blackboard::create());
  auto mocks = collectMocks(tree);
  ASSERT_EQ(mocks.size(), 2u);
  mocks[0]->status_fn = [] { return BT::NodeStatus::FAILURE; };
  mocks[1]->status_fn = [] { return BT::NodeStatus::SUCCESS; };
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(mocks[0]->tick_count, 1);
  EXPECT_EQ(mocks[1]->tick_count, 1);
}

TEST(BehaviorTreeControlTest, recoverySequenceRotatesAcrossSuccesses) {
  // Rotating recovery semantics (nav2 RoundRobin): the cursor keeps
  // advancing across successes, so consecutive recovery rounds rotate
  // through the means instead of repeating the first one.
  auto factory = makeFactory();
  factory.registerNodeType<RecoverySequence>("RecoverySequence");
  auto tree = factory.createTreeFromText(
    R"(
<root BTCPP_format="4"><BehaviorTree ID="main">
  <RecoverySequence><MockChild name="first"/><MockChild name="second"/><MockChild name="third"/></RecoverySequence>
</BehaviorTree></root>)",
    BT::Blackboard::create());
  auto mocks = collectMocks(tree);
  ASSERT_EQ(mocks.size(), 3u);

  // Round 1: first fails, second succeeds -> cursor advances PAST second.
  mocks[0]->status_fn = [] { return BT::NodeStatus::FAILURE; };
  mocks[1]->status_fn = [] { return BT::NodeStatus::SUCCESS; };
  mocks[2]->status_fn = [] { return BT::NodeStatus::SUCCESS; };
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(mocks[0]->tick_count, 1);
  EXPECT_EQ(mocks[1]->tick_count, 1);
  EXPECT_EQ(mocks[2]->tick_count, 0);

  // Round 2: enters at THIRD (rotated), not at first.
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(mocks[0]->tick_count, 1);
  EXPECT_EQ(mocks[1]->tick_count, 1);
  EXPECT_EQ(mocks[2]->tick_count, 1);
}

TEST(BehaviorTreeControlTest, recoverySequenceExhaustsToFailure) {
  // All recoveries fail -> overall FAILURE (recovery sequence exhausted).
  auto factory = makeFactory();
  factory.registerNodeType<RecoverySequence>("RecoverySequence");
  auto tree = factory.createTreeFromText(
    R"(
<root BTCPP_format="4"><BehaviorTree ID="main">
  <RecoverySequence><MockChild name="first"/><MockChild name="second"/><MockChild name="third"/></RecoverySequence>
</BehaviorTree></root>)",
    BT::Blackboard::create());
  auto mocks = collectMocks(tree);
  ASSERT_EQ(mocks.size(), 3u);
  for (auto& mock : mocks) {
    mock->status_fn = [] { return BT::NodeStatus::FAILURE; };
  }
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(mocks[0]->tick_count, 1);
  EXPECT_EQ(mocks[1]->tick_count, 1);
  EXPECT_EQ(mocks[2]->tick_count, 1);
  // Reset after exhaustion: the next tick restarts from first (recovery rounds can be entered again).
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(mocks[0]->tick_count, 2);
}

TEST(BehaviorTreeControlTest, recoveryNodeResetsRetriesAfterSuccess) {
  auto factory = makeFactory();
  factory.registerNodeType<RecoveryNode>("RecoveryNode");
  auto bb = BT::Blackboard::create();
  auto tree = factory.createTreeFromText(k_recovery_xml, bb);

  auto mocks = collectMocks(tree);
  ASSERT_EQ(mocks.size(), 2u);
  mocks[1]->status_fn = [] { return BT::NodeStatus::SUCCESS; };

  // First round succeeds -> retry counter resets
  mocks[0]->status_fn = [] { return BT::NodeStatus::SUCCESS; };
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(mocks[0]->tick_count, 1);
  EXPECT_EQ(mocks[1]->tick_count, 0);

  // Re-count after reset: 2 failures + 2 recovery successes, then the 3rd failure exhausts
  mocks[0]->status_fn = [] { return BT::NodeStatus::FAILURE; };
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(mocks[0]->tick_count, 4);
  EXPECT_EQ(mocks[1]->tick_count, 2);
}

TEST(BehaviorTreeControlTest, rateControllerGatesTicksAtHz) {
  BT::TimePoint now{};
  auto clock = [&now] { return now; };

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<MockChild>("MockChild");
  factory.registerNodeType<RateController, RateController::ClockFn>("RateController", clock);
  auto bb = BT::Blackboard::create();
  auto tree = factory.createTreeFromText(
    R"(
<root BTCPP_format="4">
  <BehaviorTree ID="main">
    <RateController hz="10.0">
      <MockChild name="c"/>
    </RateController>
  </BehaviorTree>
</root>)",
    bb);

  auto mocks = collectMocks(tree);
  ASSERT_EQ(mocks.size(), 1u);
  mocks[0]->status_fn = [] { return BT::NodeStatus::SUCCESS; };

  // 100Hz tick for 100ms (10 ticks, one every 10ms): hz=10 -> 100ms period; the first tick executes immediately,
  // the rest return the cached status during the cooldown, so the child executes only once.
  for (int i = 0; i < 10; ++i) {
    EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::SUCCESS);
    now += std::chrono::milliseconds(10);
  }
  EXPECT_EQ(mocks[0]->tick_count, 1);
}

TEST(BehaviorTreeControlTest, rateControllerRunsAgainAfterPeriodElapsed) {
  BT::TimePoint now{};
  auto clock = [&now] { return now; };

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<MockChild>("MockChild");
  factory.registerNodeType<RateController, RateController::ClockFn>("RateController", clock);
  auto bb = BT::Blackboard::create();
  auto tree = factory.createTreeFromText(
    R"(
<root BTCPP_format="4">
  <BehaviorTree ID="main">
    <RateController hz="10.0">
      <MockChild name="c"/>
    </RateController>
  </BehaviorTree>
</root>)",
    bb);

  auto mocks = collectMocks(tree);
  ASSERT_EQ(mocks.size(), 1u);
  mocks[0]->status_fn = [] { return BT::NodeStatus::SUCCESS; };

  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::SUCCESS);
  now += std::chrono::milliseconds(50);
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::SUCCESS);
  now += std::chrono::milliseconds(50);
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(mocks[0]->tick_count, 2);

  // No execution within the (100ms) period; executes again after crossing the period
  now += std::chrono::milliseconds(50);
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(mocks[0]->tick_count, 2);
  now += std::chrono::milliseconds(50);
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(mocks[0]->tick_count, 3);
}

TEST(BehaviorTreeControlTest, rateControllerHzMinusOneRunsOnceThenCaches) {
  BT::TimePoint now{};
  auto clock = [&now] { return now; };

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<MockChild>("MockChild");
  factory.registerNodeType<RateController, RateController::ClockFn>("RateController", clock);
  auto bb = BT::Blackboard::create();
  auto tree = factory.createTreeFromText(
    R"(
<root BTCPP_format="4">
  <BehaviorTree ID="main">
    <RateController hz="-1.0">
      <MockChild name="c"/>
    </RateController>
  </BehaviorTree>
</root>)",
    bb);

  auto mocks = collectMocks(tree);
  ASSERT_EQ(mocks.size(), 1u);
  mocks[0]->status_fn = [] { return BT::NodeStatus::SUCCESS; };

  // hz=-1: caches SUCCESS after a single execution
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(mocks[0]->tick_count, 1);
}

TEST(BehaviorTreeControlTest, rateControllerResetTokenReexecutesCachedChild) {
  BT::TimePoint now{};
  auto clock = [&now] { return now; };

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<MockChild>("MockChild");
  factory.registerNodeType<RateController, RateController::ClockFn>("RateController", clock);
  auto bb = BT::Blackboard::create();
  bb->set<uint64_t>("goal_revision", 0u);
  auto tree = factory.createTreeFromText(
    R"(
<root BTCPP_format="4">
  <BehaviorTree ID="main">
    <RateController hz="-1.0" reset_token="{goal_revision}">
      <MockChild name="c"/>
    </RateController>
  </BehaviorTree>
</root>)",
    bb);

  auto mocks = collectMocks(tree);
  ASSERT_EQ(mocks.size(), 1u);
  mocks[0]->status_fn = [] { return BT::NodeStatus::SUCCESS; };

  // Single-shot mode caches SUCCESS; subsequent ticks no longer execute the child
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(mocks[0]->tick_count, 1);

  // reset_token changed (new goal -> goal_revision incremented): re-executes immediately, ignoring the cache.
  // This is the mechanism behind "replan/re-smooth immediately on a new goal" (in the default tree the planner/smoother
  // RateControllers both bind reset_token="{goal_revision}").
  bb->set<uint64_t>("goal_revision", 1u);
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(mocks[0]->tick_count, 2);

  // Keeps caching while the token is unchanged
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(mocks[0]->tick_count, 2);

  bb->set<uint64_t>("goal_revision", 2u);
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(mocks[0]->tick_count, 3);
}

TEST(BehaviorTreeControlTest, pipelineSequenceStaysOnRunningChildAndAdvances) {
  auto factory = makeFactory();
  factory.registerNodeType<PipelineSequence>("PipelineSequence");
  auto bb = BT::Blackboard::create();
  auto tree = factory.createTreeFromText(
    R"(
<root BTCPP_format="4">
  <BehaviorTree ID="main">
    <PipelineSequence name="seq">
      <MockChild name="c0"/>
      <MockChild name="c1"/>
    </PipelineSequence>
  </BehaviorTree>
</root>)",
    bb);

  auto mocks = collectMocks(tree);
  ASSERT_EQ(mocks.size(), 2u);
  mocks[1]->status_fn = [] { return BT::NodeStatus::SUCCESS; };

  // While child0 is RUNNING, the next tick still executes child0
  mocks[0]->status_fn = [] { return BT::NodeStatus::RUNNING; };
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(mocks[0]->tick_count, 2);
  EXPECT_EQ(mocks[1]->tick_count, 0);

  // After child0 SUCCESS, moves from child0 on to child1
  mocks[0]->status_fn = [] { return BT::NodeStatus::SUCCESS; };
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(mocks[0]->tick_count, 3);
  EXPECT_EQ(mocks[1]->tick_count, 1);
}

TEST(BehaviorTreeControlTest, pipelineSequenceFailsWhenAnyChildFails) {
  auto factory = makeFactory();
  factory.registerNodeType<PipelineSequence>("PipelineSequence");
  auto bb = BT::Blackboard::create();
  auto tree = factory.createTreeFromText(
    R"(
<root BTCPP_format="4">
  <BehaviorTree ID="main">
    <PipelineSequence name="seq">
      <MockChild name="c0"/>
      <MockChild name="c1"/>
    </PipelineSequence>
  </BehaviorTree>
</root>)",
    bb);

  auto mocks = collectMocks(tree);
  ASSERT_EQ(mocks.size(), 2u);

  // child0 fails outright -> whole tree FAILURE
  mocks[0]->status_fn = [] { return BT::NodeStatus::FAILURE; };
  mocks[1]->status_fn = [] { return BT::NodeStatus::SUCCESS; };
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(mocks[0]->tick_count, 1);
  EXPECT_EQ(mocks[1]->tick_count, 0);

  // child1 FAILURE -> whole tree FAILURE
  auto factory2 = makeFactory();
  factory2.registerNodeType<PipelineSequence>("PipelineSequence");
  auto bb2 = BT::Blackboard::create();
  auto tree2 = factory2.createTreeFromText(
    R"(
<root BTCPP_format="4">
  <BehaviorTree ID="main">
    <PipelineSequence name="seq">
      <MockChild name="c0"/>
      <MockChild name="c1"/>
    </PipelineSequence>
  </BehaviorTree>
</root>)",
    bb2);
  auto mocks2 = collectMocks(tree2);
  ASSERT_EQ(mocks2.size(), 2u);
  mocks2[0]->status_fn = [] { return BT::NodeStatus::SUCCESS; };
  mocks2[1]->status_fn = [] { return BT::NodeStatus::FAILURE; };
  EXPECT_EQ(tree2.tickExactlyOnce(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(mocks2[0]->tick_count, 1);
  EXPECT_EQ(mocks2[1]->tick_count, 1);
}

TEST(BehaviorTreeControlTest, isPathValidEmptyPathFailsNonEmptySucceeds) {
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<IsPathValid>("IsPathValid");
  auto bb = BT::Blackboard::create();
  auto tree = factory.createTreeFromText(
    R"(
<root BTCPP_format="4">
  <BehaviorTree ID="main">
    <IsPathValid path="{raw_path}"/>
  </BehaviorTree>
</root>)",
    bb);

  bb->set<Path>(k_raw_path, Path{});
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::FAILURE);

  Path path;
  path.header.frame_id = "map";
  path.poses.push_back(makePose(0.0));
  path.poses.push_back(makePose(1.0));
  bb->set<Path>(k_raw_path, path);
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::SUCCESS);
}

TEST(BehaviorTreeControlTest, goalUpdatedDetectsGoalChange) {
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<GoalUpdated>("GoalUpdated");
  auto bb = BT::Blackboard::create();
  auto tree = factory.createTreeFromText(
    R"(
<root BTCPP_format="4">
  <BehaviorTree ID="main">
    <GoalUpdated goal="{goal}"/>
  </BehaviorTree>
</root>)",
    bb);

  // First tick initializes the baseline -> FAILURE
  bb->set<PoseStamped>(k_goal, makePose(0.0));
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::FAILURE);

  // Blackboard goal changed -> SUCCESS
  auto changed = makePose(5.0);
  changed.header.stamp = ayanami::Timestamp{static_cast<std::int64_t>(1)};
  bb->set<PoseStamped>(k_goal, changed);
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::SUCCESS);

  // Unchanged -> FAILURE
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::FAILURE);
}

TEST(BehaviorTreeControlTest, goalUpdatedReArmsOnSessionChange) {
  // Resident-tree re-arm discipline: with the session_id port bound (the
  // production tree binds {session_id}), a session change is exactly an
  // attempt boundary -- the boundary tick re-establishes the baseline and
  // never reports an update (nav2 GoalUpdatedCondition IDLE-edge parity),
  // so the new attempt's recovery ladder is not short-circuited by the
  // previous attempt's goal. Within an attempt, a rewritten {goal} (an
  // in-flight revision on the same session) reads as an update. BT.CPP v4
  // offers no reliable per-attempt IDLE edge for conditions
  // (ReactiveSequence resets non-running children every tick), so the
  // pilot-written session is the boundary signal.
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<GoalUpdated>("GoalUpdated");
  auto bb = BT::Blackboard::create();
  bb->set<uint64_t>(k_session_id, 1u);
  auto tree = factory.createTreeFromText(
    R"(
<root BTCPP_format="4">
  <BehaviorTree ID="main">
    <GoalUpdated goal="{goal}" session_id="{session_id}"/>
  </BehaviorTree>
</root>)",
    bb);

  // First tick: baseline -> FAILURE
  bb->set<PoseStamped>(k_goal, makePose(0.0));
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::FAILURE);
  // Same session, same goal -> FAILURE
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::FAILURE);

  // Mid-attempt revision (same session, rewritten goal) -> SUCCESS
  bb->set<PoseStamped>(k_goal, makePose(5.0));
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::SUCCESS);
  // Unchanged -> FAILURE
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::FAILURE);

  // Attempt boundary (session advance): re-baseline -> FAILURE, even though
  // the goal port changed -- the new attempt's ladder must not be
  // short-circuited by the previous attempt's goal
  bb->set<uint64_t>(k_session_id, 2u);
  bb->set<PoseStamped>(k_goal, makePose(9.0));
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::FAILURE);
  // Same session, changed goal -> SUCCESS again
  bb->set<PoseStamped>(k_goal, makePose(11.0));
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::SUCCESS);
}

TEST(BehaviorTreeControlTest, isRobotProgressingTokenClearsHistory) {
  // Stationary robot: the gate must fail after a full window of stillness,
  // and the reset_token (the pilot's goal_revision) must clear the sample
  // history so a fresh attempt starts with the "progressing" assumption --
  // the previous attempt's tail stillness must never false-trigger the
  // recovery ladder of the next one.
  auto& transforms = ayanami::testing::testTransforms();
  transforms.install();  // stub TF bridge into the RobotStateController singleton
  transforms.set("odom", "base_link", ayanami::Timestamp::latest(), Eigen::Isometry3d::Identity());
  transforms.set("map", "odom", ayanami::Timestamp::latest(), Eigen::Isometry3d::Identity());

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<IsRobotProgressing>("IsRobotProgressing");
  auto bb = BT::Blackboard::create();
  bb->set<uint64_t>(k_goal_revision, 0u);
  bb->set<double>(k_tick_frequency, 100.0);
  auto tree = factory.createTreeFromText(
    R"(
<root BTCPP_format="4">
  <BehaviorTree ID="main">
    <IsRobotProgressing distance="0.1" window="0.12" reset_token="{goal_revision}"/>
  </BehaviorTree>
</root>)",
    bb);

  auto tickUntilFailure = [&] {
    for (int i = 0; i < 100; ++i) {
      if (tree.tickExactlyOnce() == BT::NodeStatus::FAILURE) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return false;
  };
  EXPECT_TRUE(tickUntilFailure()) << "a stationary robot must trip the oscillation gate";

  // Attempt boundary (the pilot bumped the revision): the history clears and
  // the gate reads "progressing" again even though the robot never moved.
  bb->set<uint64_t>(k_goal_revision, 1u);
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::SUCCESS);

  // ... and stillness trips it again after another full window.
  EXPECT_TRUE(tickUntilFailure());
}

// ---- Deadline decorator: the BT-level watchdog for slow module calls ----

constexpr const char* k_deadline_xml = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="main">
    <Deadline name="dl" msec="{msec}" reset_token="{goal_revision}" failure_context="{failure_context}">
      <MockChild name="c"/>
    </Deadline>
  </BehaviorTree>
</root>)";

BT::Tree buildDeadlineTree(BT::BehaviorTreeFactory& factory,
                           const BT::Blackboard::Ptr& bb, unsigned msec) {
  bb->set<unsigned>("msec", msec);
  bb->set<uint64_t>(k_goal_revision, 0u);
  bb->set<ayanami::FailureContext>(k_failure_context, ayanami::FailureContext{});
  return factory.createTreeFromText(k_deadline_xml, bb);
}

TEST(BehaviorTreeControlTest, deadlineHaltsRunningChildAndFailsWithTimeout) {
  auto factory = makeFactory();
  factory.registerNodeType<Deadline>("Deadline");
  auto bb = BT::Blackboard::create();
  auto tree = buildDeadlineTree(factory, bb, 60u);
  auto mocks = collectMocks(tree);
  ASSERT_EQ(mocks.size(), 1u);
  mocks[0]->status_fn = [] { return BT::NodeStatus::RUNNING; };

  // First tick starts the clock; the child is RUNNING.
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  std::this_thread::sleep_for(std::chrono::milliseconds(120));

  // Past the budget: the child is HALTED (status back to IDLE) and the
  // decorator fails with a structured TIMEOUT context for the RecoveryNode.
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(mocks[0]->status(), BT::NodeStatus::IDLE);
  ayanami::FailureContext ctx;
  ASSERT_TRUE(bb->get<ayanami::FailureContext>(k_failure_context, ctx));
  EXPECT_EQ(ctx.code, ErrorCode::TIMEOUT);
}

TEST(BehaviorTreeControlTest, deadlineZeroMeansUnlimitedPassthrough) {
  auto factory = makeFactory();
  factory.registerNodeType<Deadline>("Deadline");
  auto bb = BT::Blackboard::create();
  auto tree = buildDeadlineTree(factory, bb, 0u);
  auto mocks = collectMocks(tree);
  ASSERT_EQ(mocks.size(), 1u);
  mocks[0]->status_fn = [] { return BT::NodeStatus::RUNNING; };

  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING)
      << "msec=0 must disable the deadline entirely";
}

TEST(BehaviorTreeControlTest, deadlineRestartsClockOnResetTokenAndTerminalChild) {
  auto factory = makeFactory();
  factory.registerNodeType<Deadline>("Deadline");
  auto bb = BT::Blackboard::create();
  auto tree = buildDeadlineTree(factory, bb, 50u);
  auto mocks = collectMocks(tree);
  ASSERT_EQ(mocks.size(), 1u);
  mocks[0]->status_fn = [] { return BT::NodeStatus::RUNNING; };

  // Burn most of the budget.
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  std::this_thread::sleep_for(std::chrono::milliseconds(40));

  // A goal revision restarts the clock: the next tick must NOT fail even
  // though the original budget is nearly spent.
  bb->set<uint64_t>(k_goal_revision, 1u);
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);

  // Terminal child stops the clock (and returns the child's status as-is).
  mocks[0]->status_fn = [] { return BT::NodeStatus::SUCCESS; };
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::SUCCESS);
}

// ---- RecoveryNode same-cause pruning ----

TEST(BehaviorTreeControlTest, recoveryNodePrunesRepeatedSameCause) {
  auto factory = makeFactory();
  factory.registerNodeType<RecoveryNode>("RecoveryNode");
  auto bb = BT::Blackboard::create();
  bb->set<uint64_t>(k_goal_revision, 0u);
  bb->set<uint32_t>("max_same_cause", 3u);
  bb->set<ayanami::FailureContext>(k_failure_context, ayanami::FailureContext{});
  auto tree = factory.createTreeFromText(R"(
<root BTCPP_format="4">
  <BehaviorTree ID="main">
    <RecoveryNode name="rec" number_of_retries="6" budget_reset_token="{goal_revision}"
                  max_same_cause="{max_same_cause}" failure_context="{failure_context}">
      <MockChild name="c0"/>
      <MockChild name="c1"/>
    </RecoveryNode>
  </BehaviorTree>
</root>)", bb);
  const auto mocks = collectMocks(tree);
  ASSERT_EQ(mocks.size(), 2u);
  mocks[0]->status_fn = [] { return BT::NodeStatus::FAILURE; };
  mocks[1]->status_fn = [] { return BT::NodeStatus::SUCCESS; };

  // Round 1: c0 fails with MAP_STALE (count 1) -> recovery dispatched.
  bb->set<ayanami::FailureContext>(
      k_failure_context, ayanami::FailureContext{ayanami::FailureStage::Controller,
                                                 ErrorCode::MAP_STALE, "", "", 0, 0});
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  // Round 2: same code (count 2) -> recovery dispatched again.
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  // Round 3: same code reaches the budget (count 3) -> pruned: FAILURE with
  // NO recovery dispatch this round.
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::FAILURE);

  EXPECT_EQ(mocks[1]->tick_count, 2)
      << "the third same-cause failure must prune the recovery dispatch";
  // The consumed cause is cleared so nothing stale leaks onward.
  ayanami::FailureContext ctx;
  ASSERT_TRUE(bb->get<ayanami::FailureContext>(k_failure_context, ctx));
  EXPECT_EQ(ctx.code, ErrorCode::NONE);
}

TEST(BehaviorTreeControlTest, recoveryNodeSameCauseWatchRestartsOnDistinctCode) {
  auto factory = makeFactory();
  factory.registerNodeType<RecoveryNode>("RecoveryNode");
  auto bb = BT::Blackboard::create();
  bb->set<uint64_t>(k_goal_revision, 0u);
  bb->set<uint32_t>("max_same_cause", 3u);
  bb->set<ayanami::FailureContext>(k_failure_context, ayanami::FailureContext{});
  auto tree = factory.createTreeFromText(R"(
<root BTCPP_format="4">
  <BehaviorTree ID="main">
    <RecoveryNode name="rec" number_of_retries="6" budget_reset_token="{goal_revision}"
                  max_same_cause="{max_same_cause}" failure_context="{failure_context}">
      <MockChild name="c0"/>
      <MockChild name="c1"/>
    </RecoveryNode>
  </BehaviorTree>
</root>)", bb);
  const auto mocks = collectMocks(tree);
  ASSERT_EQ(mocks.size(), 2u);
  mocks[0]->status_fn = [] { return BT::NodeStatus::FAILURE; };
  mocks[1]->status_fn = [] { return BT::NodeStatus::SUCCESS; };

  // Alternating codes never accumulate: the watch restarts on each distinct
  // code, so the ladder keeps rotating until the retry budget (6) exhausts.
  const std::array<ErrorCode, 6> codes = {
      ErrorCode::MAP_STALE,  ErrorCode::NO_PROGRESS,   ErrorCode::MAP_STALE,
      ErrorCode::NO_PROGRESS, ErrorCode::MAP_STALE,  ErrorCode::NO_PROGRESS};
  for (unsigned round = 0; round < codes.size(); ++round) {
    bb->set<ayanami::FailureContext>(
        k_failure_context,
        ayanami::FailureContext{ayanami::FailureStage::Controller, codes[round],
                                "", "", 0, 0});
    EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING) << "round " << round;
  }
  EXPECT_EQ(mocks[1]->tick_count, codes.size())
      << "distinct causes must not trigger the same-cause prune";
}

// ---- RecoverySequence: RESOURCE_BUSY is a transient dispatch conflict ----
// A recovery rejected because another one still holds the exclusive slot
// (e.g. the local clear-costmap winding down after RecoveryNode's
// haltChild canceled it) must NOT burn the ladder: the sequence stays on
// the same child and retries next tick. Any other failure advances the
// cursor as before.
class StatusedRecoveryStub : public BT::ActionNodeBase {
public:
  StatusedRecoveryStub(const std::string& name, const BT::NodeConfig& config)
  : BT::ActionNodeBase(name, config) {}

  static BT::PortsList providedPorts() {
    return {
      BT::OutputPort<ayanami::FailureContext>(k_failure_context,
                                              "written before FAILURE"),
    };
  }

  BT::NodeStatus tick() override {
    ++tick_count;
    if (write_busy) {
      ayanami::FailureContext ctx;
      ctx.code = ayanami::ErrorCode::RESOURCE_BUSY;
      // The XML binding below must exist: an unbound output port makes
      // setOutput a silent no-op (BT.CPP drops the write).
      EXPECT_TRUE(setOutput(k_failure_context, ctx));
    } else {
      ayanami::FailureContext ctx;
      ctx.code = ayanami::ErrorCode::RECOVERY_FAILED;
      setOutput(k_failure_context, ctx);
    }
    return BT::NodeStatus::FAILURE;
  }

  void halt() override { resetStatus(); }

  int tick_count = 0;
  bool write_busy = true;
};

TEST(BehaviorTreeControlTest, recoverySequenceRetriesOnResourceBusy) {
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<StatusedRecoveryStub>("RecStub");
  factory.registerNodeType<RecoverySequence>("RecoverySequence");
  auto bb = BT::Blackboard::create();
  bb->set<uint64_t>(k_goal_revision, 0u);
  bb->set<ayanami::FailureContext>(k_failure_context, ayanami::FailureContext{});
  auto tree = factory.createTreeFromText(R"(
<root BTCPP_format="4">
  <BehaviorTree ID="main">
    <RecoverySequence name="ladder" reset_token="{goal_revision}"
                      failure_context="{failure_context}">
      <RecStub name="first" failure_context="{failure_context}"/>
      <RecStub name="second" failure_context="{failure_context}"/>
    </RecoverySequence>
  </BehaviorTree>
</root>)", bb);
  std::vector<StatusedRecoveryStub*> stubs;
  tree.applyVisitor([&stubs](BT::TreeNode* n) {
    if (auto* r = dynamic_cast<StatusedRecoveryStub*>(n)) stubs.push_back(r);
  });
  ASSERT_EQ(stubs.size(), 2u);
  stubs[0]->write_busy = true;
  stubs[1]->write_busy = true;

  // RESOURCE_BUSY: stays on the FIRST child (RUNNING), never advances.
  for (int i = 0; i < 3; ++i) {
    EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING) << "tick " << i;
  }
  EXPECT_EQ(stubs[0]->tick_count, 3) << "the busy child must be retried in place";
  EXPECT_EQ(stubs[1]->tick_count, 0) << "the ladder must not advance on RESOURCE_BUSY";

  // The first child then fails for REAL: the cursor advances, and the second
  // child -- itself still busy -- is retried in place instead of failing the
  // whole ladder.
  stubs[0]->write_busy = false;
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(stubs[0]->tick_count, 4);
  EXPECT_EQ(stubs[1]->tick_count, 1);
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(stubs[1]->tick_count, 2) << "the busy second child is retried too";
}

// real-failure advance control: non-busy failures still advance the cursor
TEST(BehaviorTreeControlTest, recoverySequenceAdvancesOnRealFailure) {
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<StatusedRecoveryStub>("RecStub");
  factory.registerNodeType<RecoverySequence>("RecoverySequence");
  auto bb = BT::Blackboard::create();
  bb->set<uint64_t>(k_goal_revision, 0u);
  bb->set<ayanami::FailureContext>(k_failure_context, ayanami::FailureContext{});
  auto tree = factory.createTreeFromText(R"(
<root BTCPP_format="4">
  <BehaviorTree ID="main">
    <RecoverySequence name="ladder" reset_token="{goal_revision}"
                      failure_context="{failure_context}">
      <RecStub name="first" failure_context="{failure_context}"/>
      <RecStub name="second" failure_context="{failure_context}"/>
    </RecoverySequence>
  </BehaviorTree>
</root>)", bb);
  std::vector<StatusedRecoveryStub*> stubs;
  tree.applyVisitor([&stubs](BT::TreeNode* n) {
    if (auto* r = dynamic_cast<StatusedRecoveryStub*>(n)) stubs.push_back(r);
  });
  ASSERT_EQ(stubs.size(), 2u);
  stubs[0]->write_busy = false;
  stubs[1]->write_busy = false;
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(stubs[0]->tick_count, 1);
  EXPECT_EQ(stubs[1]->tick_count, 1) << "a real failure must advance the cursor";
}

// A failure WITHOUT a recorded cause (an inner gate cleared its context
// before the failure propagated) must not accumulate as a repeating cause:
// the whole ladder was once pruned on three consecutive code-0 failures.
TEST(BehaviorTreeControlTest, recoveryNodeSameCauseWatchIgnoresEmptyCause) {
  auto factory = makeFactory();
  factory.registerNodeType<RecoveryNode>("RecoveryNode");
  auto bb = BT::Blackboard::create();
  bb->set<uint64_t>(k_goal_revision, 0u);
  bb->set<uint32_t>("max_same_cause", 3u);
  bb->set<ayanami::FailureContext>(k_failure_context, ayanami::FailureContext{});
  auto tree = factory.createTreeFromText(R"(
<root BTCPP_format="4">
  <BehaviorTree ID="main">
    <RecoveryNode name="rec" number_of_retries="6" budget_reset_token="{goal_revision}"
                  max_same_cause="{max_same_cause}" failure_context="{failure_context}">
      <MockChild name="c0"/>
      <MockChild name="c1"/>
    </RecoveryNode>
  </BehaviorTree>
</root>)", bb);
  const auto mocks = collectMocks(tree);
  ASSERT_EQ(mocks.size(), 2u);
  mocks[0]->status_fn = [] { return BT::NodeStatus::FAILURE; };
  mocks[1]->status_fn = [] { return BT::NodeStatus::SUCCESS; };

  // MAP_STALE, then a context-less failure, then MAP_STALE again: the empty
  // cause resets the watch, so the two real MAP_STALE failures never reach
  // the prune threshold.
  const std::array<ErrorCode, 4> codes = {
      ErrorCode::MAP_STALE, ErrorCode::NONE,
      ErrorCode::MAP_STALE, ErrorCode::NONE};
  for (unsigned round = 0; round < codes.size(); ++round) {
    bb->set<ayanami::FailureContext>(
        k_failure_context,
        ayanami::FailureContext{ayanami::FailureStage::Controller, codes[round],
                                "", "", 0, 0});
    EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING) << "round " << round;
  }
  EXPECT_EQ(mocks[1]->tick_count, codes.size())
      << "empty-cause failures must reset, not feed, the same-cause watch";
}

TEST(BehaviorTreeControlTest, validateRequiredBlackboardEntriesThrowsOnMissing) {
  auto empty = BT::Blackboard::create();
  EXPECT_THROW(validateRequiredBlackboardEntries(empty), std::runtime_error);

  auto full = BT::Blackboard::create();
  for (const char* key : requiredBlackboardEntries()) {
    full->set<std::string>(key, "x");
  }
  EXPECT_NO_THROW(validateRequiredBlackboardEntries(full));
}
