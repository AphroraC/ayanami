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

#include <functional>
#include <string>
#include <vector>

using namespace ayanami::behavior_tree;
using ayanami::geometry_msgs::PoseStamped;
using ayanami::nav_msgs::Odometry;
using ayanami::nav_msgs::Path;

namespace {

/// Async action stub that can return any NodeStatus (needs RUNNING capability, so it derives from ActionNodeBase rather
/// than SyncActionNode). The test directly controls the tick count and each returned status.
class MockChild : public BT::ActionNodeBase {
 public:
  MockChild(const std::string& name, const BT::NodeConfig& config)
    : BT::ActionNodeBase(name, config) {}

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

} // namespace

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
  auto tree = factory.createTreeFromText(R"(
<root BTCPP_format="4">
  <BehaviorTree ID="main">
    <PipelineSequence name="seq">
      <MockChild name="planner"/>
      <MockChild name="controller"/>
    </PipelineSequence>
  </BehaviorTree>
</root>)", bb);

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
  auto tree = factory.createTreeFromText(R"(
<root BTCPP_format="4"><BehaviorTree ID="main">
  <RecoverySequence><MockChild name="first"/><MockChild name="second"/></RecoverySequence>
</BehaviorTree></root>)", BT::Blackboard::create());
  auto mocks = collectMocks(tree);
  ASSERT_EQ(mocks.size(), 2u);
  mocks[0]->status_fn = [] { return BT::NodeStatus::FAILURE; };
  mocks[1]->status_fn = [] { return BT::NodeStatus::SUCCESS; };
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(mocks[0]->tick_count, 1);
  EXPECT_EQ(mocks[1]->tick_count, 1);
}

TEST(BehaviorTreeControlTest, recoverySequenceRestartsFromFirstAfterSuccess) {
  // Sequential recovery semantics: one recovery succeeding -> overall SUCCESS (the outer RecoveryNode returns to the
  // main task); the next entry into the recovery branch restarts from the first recovery (recovery rounds after escaping).
  auto factory = makeFactory();
  factory.registerNodeType<RecoverySequence>("RecoverySequence");
  auto tree = factory.createTreeFromText(R"(
<root BTCPP_format="4"><BehaviorTree ID="main">
  <RecoverySequence><MockChild name="first"/><MockChild name="second"/></RecoverySequence>
</BehaviorTree></root>)", BT::Blackboard::create());
  auto mocks = collectMocks(tree);
  ASSERT_EQ(mocks.size(), 2u);

  // Round 1: first fails, second succeeds -> overall SUCCESS.
  mocks[0]->status_fn = [] { return BT::NodeStatus::FAILURE; };
  mocks[1]->status_fn = [] { return BT::NodeStatus::SUCCESS; };
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::SUCCESS);

  // Round 2 (stuck again after escaping): restarts from first.
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(mocks[0]->tick_count, 2);
  EXPECT_EQ(mocks[1]->tick_count, 2);
}

TEST(BehaviorTreeControlTest, recoverySequenceExhaustsToFailure) {
  // All recoveries fail -> overall FAILURE (recovery sequence exhausted).
  auto factory = makeFactory();
  factory.registerNodeType<RecoverySequence>("RecoverySequence");
  auto tree = factory.createTreeFromText(R"(
<root BTCPP_format="4"><BehaviorTree ID="main">
  <RecoverySequence><MockChild name="first"/><MockChild name="second"/><MockChild name="third"/></RecoverySequence>
</BehaviorTree></root>)", BT::Blackboard::create());
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
  factory.registerNodeType<RateController, RateController::ClockFn>("RateController",
                                                                    clock);
  auto bb = BT::Blackboard::create();
  auto tree = factory.createTreeFromText(R"(
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
  factory.registerNodeType<RateController, RateController::ClockFn>("RateController",
                                                                    clock);
  auto bb = BT::Blackboard::create();
  auto tree = factory.createTreeFromText(R"(
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
  factory.registerNodeType<RateController, RateController::ClockFn>("RateController",
                                                                    clock);
  auto bb = BT::Blackboard::create();
  auto tree = factory.createTreeFromText(R"(
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
  factory.registerNodeType<RateController, RateController::ClockFn>("RateController",
                                                                    clock);
  auto bb = BT::Blackboard::create();
  bb->set<uint64_t>("goal_revision", 0u);
  auto tree = factory.createTreeFromText(R"(
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
  auto tree = factory.createTreeFromText(R"(
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
  auto tree = factory.createTreeFromText(R"(
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
  auto tree2 = factory2.createTreeFromText(R"(
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
  auto tree = factory.createTreeFromText(R"(
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
  auto tree = factory.createTreeFromText(R"(
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

TEST(BehaviorTreeControlTest, validateRequiredBlackboardEntriesThrowsOnMissing) {
  auto empty = BT::Blackboard::create();
  EXPECT_THROW(validateRequiredBlackboardEntries(empty), std::runtime_error);

  auto full = BT::Blackboard::create();
  for (const char* key : requiredBlackboardEntries()) {
    full->set<std::string>(key, "x");
  }
  EXPECT_NO_THROW(validateRequiredBlackboardEntries(full));
}
