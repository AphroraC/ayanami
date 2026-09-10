// pipeline_sequence_scenarios_test: dedicated tests for key PipelineSequence / RecoveryNode scenarios
// (checked against nav2 semantics), plus RateController reset_token semantics tests.
//
// Covered scenarios:
// S1 planner RUNNING + controller RUNNING -> both are ticked, overall RUNNING
// S2 planner SUCCESS + controller RUNNING -> both are ticked, overall RUNNING
// S3 planner FAILURE -> overall FAILURE, controller never ticked
// S4 controller FAILURE -> overall FAILURE
// S5 planner replans while the controller runs -> the controller is still ticked every tick
// (tracking uninterrupted, in-place following)
// S6 goal update triggers planner reentry -> the controller keeps being ticked
// R1 RateController reset_token change -> the in-flight child is halted and restarts via onStart
// R2 RecoveryNode: child0 FAILURE -> recovery SUCCESS -> main task retried as RUNNING

#include <gtest/gtest.h>

#include <ayanami/behavior_tree/navigation.hpp>
#include <ayanami/behavior_tree/pipeline_sequence.hpp>
#include <ayanami/behavior_tree/rate_controller.hpp>
#include <ayanami/behavior_tree/recovery_node.hpp>

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>

#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace {

using ayanami::behavior_tree::PipelineSequence;
using ayanami::behavior_tree::RateController;
using ayanami::behavior_tree::RecoveryNode;

/// Programmable mock child: its status function is switchable (simulating e.g. the planner reentering RUNNING
/// while the controller runs) and records tick / halt counts. Needs RUNNING capability, so it derives from
/// ActionNodeBase rather than SyncActionNode (BehaviorTree.CPP forbids Sync nodes returning RUNNING).
class MockChild : public BT::ActionNodeBase {
 public:
  MockChild(const std::string& name, const BT::NodeConfig& config)
    : BT::ActionNodeBase(name, config) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override {
    ++tick_count;
    return status_fn ? status_fn() : BT::NodeStatus::SUCCESS;
  }

  void halt() override {
    resetStatus();
    ++halt_count;
  }

  std::function<BT::NodeStatus()> status_fn;
  int tick_count{0};
  int halt_count{0};
};

BT::BehaviorTreeFactory makeFactory() {
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<PipelineSequence>("PipelineSequence");
  factory.registerNodeType<RecoveryNode>("RecoveryNode");
  factory.registerNodeType<MockChild>("MockChild");
  return factory;
}

std::vector<MockChild*> collectMocks(BT::Tree& tree) {
  std::vector<MockChild*> out;
  tree.applyVisitor([&out](BT::TreeNode* node) {
    if (auto* mock = dynamic_cast<MockChild*>(node)) {
      out.push_back(mock);
    }
  });
  return out;
}

const char* k_two_child_xml = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="main">
    <PipelineSequence name="seq">
      <MockChild name="planner"/>
      <MockChild name="controller"/>
    </PipelineSequence>
  </BehaviorTree>
</root>)";

// S1: planner RUNNING + controller RUNNING -> both are ticked, overall RUNNING.
TEST(PipelineSequenceScenarios, plannerRunningAndControllerRunningTickBoth) {
  auto factory = makeFactory();
  auto tree = factory.createTreeFromText(k_two_child_xml);
  auto mocks = collectMocks(tree);
  ASSERT_EQ(mocks.size(), 2u);
  mocks[0]->status_fn = [] { return BT::NodeStatus::RUNNING; };
  mocks[1]->status_fn = [] { return BT::NodeStatus::RUNNING; };

  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  // The controller has not been advanced to yet (the predecessor has not succeeded): only the planner is ticked.
  EXPECT_EQ(mocks[0]->tick_count, 1);
  EXPECT_EQ(mocks[1]->tick_count, 0);
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(mocks[0]->tick_count, 2);
  EXPECT_EQ(mocks[1]->tick_count, 0);
}

// S2: planner SUCCESS + controller RUNNING -> overall RUNNING.
TEST(PipelineSequenceScenarios, plannerSuccessAndControllerRunning) {
  auto factory = makeFactory();
  auto tree = factory.createTreeFromText(k_two_child_xml);
  auto mocks = collectMocks(tree);
  ASSERT_EQ(mocks.size(), 2u);
  mocks[0]->status_fn = [] { return BT::NodeStatus::SUCCESS; };
  mocks[1]->status_fn = [] { return BT::NodeStatus::RUNNING; };

  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(mocks[0]->tick_count, 1);
  EXPECT_EQ(mocks[1]->tick_count, 1);
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(mocks[0]->tick_count, 2);
  EXPECT_EQ(mocks[1]->tick_count, 2);
}

// S3: planner FAILURE -> overall FAILURE, controller never ticked.
TEST(PipelineSequenceScenarios, plannerFailureFailsWithoutTickingController) {
  auto factory = makeFactory();
  auto tree = factory.createTreeFromText(k_two_child_xml);
  auto mocks = collectMocks(tree);
  ASSERT_EQ(mocks.size(), 2u);
  mocks[0]->status_fn = [] { return BT::NodeStatus::FAILURE; };
  mocks[1]->status_fn = [] { return BT::NodeStatus::SUCCESS; };

  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(mocks[0]->tick_count, 1);
  EXPECT_EQ(mocks[1]->tick_count, 0);
}

// S4: controller FAILURE -> overall FAILURE (the planner, already SUCCESS, is re-ticked once more).
TEST(PipelineSequenceScenarios, controllerFailureFailsPipeline) {
  auto factory = makeFactory();
  auto tree = factory.createTreeFromText(k_two_child_xml);
  auto mocks = collectMocks(tree);
  ASSERT_EQ(mocks.size(), 2u);
  mocks[0]->status_fn = [] { return BT::NodeStatus::SUCCESS; };
  mocks[1]->status_fn = [] { return BT::NodeStatus::FAILURE; };

  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(mocks[0]->tick_count, 1);
  EXPECT_EQ(mocks[1]->tick_count, 1);
}

/// S5: the planner produces a new path while the controller runs (planner reenters RUNNING) ->
/// the controller is still ticked every tick (tracking uninterrupted; the new path is followed via the in-place update).
TEST(PipelineSequenceScenarios,
     plannerReentryKeepsControllerTickingDuringReplan) {
  auto factory = makeFactory();
  auto tree = factory.createTreeFromText(k_two_child_xml);
  auto mocks = collectMocks(tree);
  ASSERT_EQ(mocks.size(), 2u);
  mocks[1]->status_fn = [] { return BT::NodeStatus::RUNNING; };

  // Cycles 1-2: initial planning; cycle 3: new path done; cycle 4: planner reenters after the goal update.
  mocks[0]->status_fn = [] { return BT::NodeStatus::RUNNING; };
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  mocks[0]->status_fn = [] { return BT::NodeStatus::RUNNING; };
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  mocks[0]->status_fn = [] { return BT::NodeStatus::SUCCESS; };
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(mocks[1]->tick_count, 1);  // The controller is already following
  mocks[0]->status_fn = [] { return BT::NodeStatus::RUNNING; };
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  // The controller keeps being ticked during the replan (it does not stall because of the replan).
  EXPECT_EQ(mocks[1]->tick_count, 3);
}

/// S6: on goal update the planner reenters RUNNING while the controller stays RUNNING ->
/// PipelineSequence keeps ticking both branches (the BT does not interrupt either side).
TEST(PipelineSequenceScenarios, goalUpdateReticksPlannerAndController) {
  auto factory = makeFactory();
  auto tree = factory.createTreeFromText(k_two_child_xml);
  auto mocks = collectMocks(tree);
  ASSERT_EQ(mocks.size(), 2u);
  mocks[0]->status_fn = [] { return BT::NodeStatus::SUCCESS; };
  mocks[1]->status_fn = [] { return BT::NodeStatus::RUNNING; };
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);

  // Goal update: the planner is RUNNING again; the controller keeps following the old path.
  mocks[0]->status_fn = [] { return BT::NodeStatus::RUNNING; };
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(mocks[0]->tick_count, 2);
  EXPECT_EQ(mocks[1]->tick_count, 2);
}

/// R1: a RateController reset_token change -> the in-flight RUNNING child is halted (onHalted) and
/// restarts via onStart in the same tick (fresh input restart, not continued consumption of the old input).
class HaltObserverAction : public BT::StatefulActionNode {
 public:
  HaltObserverAction(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus onStart() override {
    ++start_count;
    return running ? BT::NodeStatus::RUNNING : BT::NodeStatus::SUCCESS;
  }
  BT::NodeStatus onRunning() override {
    return running ? BT::NodeStatus::RUNNING : BT::NodeStatus::SUCCESS;
  }
  void onHalted() override { ++halt_count; }

  bool running{true};
  int start_count{0};
  int halt_count{0};
};

TEST(RateControllerResetTest, tokenChangeHaltsInFlightChildAndRestarts) {
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<RateController>("RateController");
  factory.registerNodeType<HaltObserverAction>("Observer");
  auto bb = BT::Blackboard::create();
  bb->set<uint64_t>("reset_token", 1u);
  auto tree = factory.createTreeFromText(R"(
<root BTCPP_format="4">
  <BehaviorTree ID="main">
    <RateController hz="-1" reset_token="{reset_token}">
      <Observer name="obs"/>
    </RateController>
  </BehaviorTree>
</root>)", bb);
  std::shared_ptr<HaltObserverAction> observer;
  tree.applyVisitor([&observer](BT::TreeNode* node) {
    if (auto* found = dynamic_cast<HaltObserverAction*>(node)) {
      // The exclusive ownership order matches tree destruction (shared_ptr view); safe for the test lifetime.
      observer.reset(found, [](HaltObserverAction*) {});
    }
  });
  ASSERT_NE(observer, nullptr);

  // First tick: onStart, RUNNING; next tick: onRunning polling.
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(observer->start_count, 1);
  EXPECT_EQ(observer->halt_count, 0);

  // Token change (new path landed semantics): the in-flight child is halted and restarts via onStart in the same tick.
  bb->set<uint64_t>("reset_token", 2u);
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(observer->halt_count, 1);
  EXPECT_EQ(observer->start_count, 2);
}

/// R2: RecoveryNode child0 FAILURE -> recovery child SUCCESS -> returns RUNNING
/// so the main task retries (nav2 RecoveryNode semantics).
TEST(RecoveryNodeSemantics, recoverySuccessRetriesMainTaskAsRunning) {
  auto factory = makeFactory();
  auto tree = factory.createTreeFromText(R"(
<root BTCPP_format="4">
  <BehaviorTree ID="main">
    <RecoveryNode name="rn" number_of_retries="2">
      <MockChild name="main_task"/>
      <MockChild name="recovery"/>
    </RecoveryNode>
  </BehaviorTree>
</root>)");
  auto mocks = collectMocks(tree);
  ASSERT_EQ(mocks.size(), 2u);
  MockChild* main_task = mocks[0];
  mocks[0]->status_fn = [main_task] {
    return main_task->tick_count <= 1 ? BT::NodeStatus::FAILURE
                                      : BT::NodeStatus::SUCCESS;
  };
  mocks[1]->status_fn = [] { return BT::NodeStatus::SUCCESS; };

  // tick1: main FAILURE -> recovery SUCCESS -> overall RUNNING (retry entry).
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(mocks[0]->tick_count, 1);
  EXPECT_EQ(mocks[1]->tick_count, 1);
  // tick2: the main task retry SUCCEEDS -> overall SUCCESS.
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(mocks[0]->tick_count, 2);
  EXPECT_EQ(mocks[1]->tick_count, 1);
}


/// R3: RateController hz>0 -- a RUNNING child is polled immediately (nav2 semantics); the rate only gates
/// a new round; the timer resets only when the child SUCCEEDS.
TEST(RateControllerPollingTest, runningChildIsPolledEveryTickAndTimerResetsOnSuccess) {
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<RateController>("RateController");
  factory.registerNodeType<MockChild>("MockChild");

  using Clock = std::chrono::system_clock;  // BT::TimePoint is based on system_clock
  Clock::time_point now = Clock::now();
  auto clock_fn = [&now] { return now; };

  BT::NodeConfig config;
  config.input_ports["hz"] = "10.0";  // period=100ms
  RateController::ClockFn clock = clock_fn;
  RateController rc("rc", config, clock);

 // Attach the child manually (DecoratorNode::setChild takes over the raw pointer; holding it via a shared_ptr outside
 // the tree is enough -- same order as tree destruction).
  auto child_owned = std::make_shared<MockChild>("child", BT::NodeConfig());
  rc.setChild(child_owned.get());
  MockChild* mock = child_owned.get();

  const auto tick = [&](int advance_ms) {
    now += std::chrono::milliseconds(advance_ms);
    return rc.executeTick();
  };

  mock->status_fn = [] { return BT::NodeStatus::RUNNING; };
  EXPECT_EQ(tick(0), BT::NodeStatus::RUNNING);      // First tick (started)
  EXPECT_EQ(mock->tick_count, 1);
  EXPECT_EQ(tick(10), BT::NodeStatus::RUNNING);     // Period not reached but RUNNING -> polled immediately
  EXPECT_EQ(mock->tick_count, 2);
  EXPECT_EQ(tick(10), BT::NodeStatus::RUNNING);
  EXPECT_EQ(mock->tick_count, 3);

 // Completed (SUCCESS): the timer resets
  mock->status_fn = [] { return BT::NodeStatus::SUCCESS; };
  EXPECT_EQ(tick(10), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(mock->tick_count, 4);

 // After SUCCESS with the period not reached (20ms accumulated < 100ms): no more ticks
  mock->status_fn = [] { return BT::NodeStatus::SUCCESS; };
  EXPECT_EQ(tick(20), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(mock->tick_count, 4);

 // Period reached: a new round
  EXPECT_EQ(tick(90), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(mock->tick_count, 5);
}

// ---- RecoveryNode budget_reset_token: goal updates never consume budget ----

TEST(RecoveryNodeBudgetTest, tokenChangeResetsBudget) {
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<RecoveryNode>("RecoveryNode");
  factory.registerNodeType<MockChild>("MockChild");
  auto bb = BT::Blackboard::create();
  bb->set<uint64_t>("tok", 0u);
  auto tree = factory.createTreeFromText(R"(
<root BTCPP_format="4">
  <BehaviorTree ID="main">
    <RecoveryNode name="rn" number_of_retries="1" budget_reset_token="{tok}">
      <MockChild name="main_task"/>
      <MockChild name="recovery"/>
    </RecoveryNode>
  </BehaviorTree>
</root>)", bb);
  auto mocks = collectMocks(tree);
  ASSERT_EQ(mocks.size(), 2u);
  MockChild* main_task = mocks[0];
  MockChild* recovery = mocks[1];

  // main always fails, recovery always succeeds: with retries=1 the second
  // cycle must fail without a budget reset.
  main_task->status_fn = [] { return BT::NodeStatus::FAILURE; };
  recovery->status_fn = [] { return BT::NodeStatus::SUCCESS; };

  // Cycle 1: main FAILURE -> recovery SUCCESS -> retry_count=1, RUNNING.
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(main_task->tick_count, 1);
  EXPECT_EQ(recovery->tick_count, 1);

  // Cycle 2 (no token change): retry_count(1) >= max_retries(1) -> FAILURE,
  // recovery is not ticked again.
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(recovery->tick_count, 1);
  tree.haltTree();

  // New tree life + token change (new goal): budget reset, the same
  // sequence runs through a full cycle again.
  bb->set<uint64_t>("tok", 1u);
  main_task->status_fn = [main_task] {
    return main_task->tick_count <= 3 ? BT::NodeStatus::FAILURE
                                      : BT::NodeStatus::SUCCESS;
  };
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(recovery->tick_count, 2);  // budget was reset, recovery allowed
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::SUCCESS);
}

// ---- H3 resident-tree reset protocol: RecoverySequence reset_token ----

TEST(RecoverySequenceResetTest, tokenChangeRestartsCursorAndHaltsInFlightChild) {
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<ayanami::behavior_tree::RecoverySequence>(
      "RecoverySequence");
  factory.registerNodeType<MockChild>("MockChild");
  auto bb = BT::Blackboard::create();
  bb->set<uint64_t>("goal_revision", 0u);
  auto tree = factory.createTreeFromText(R"(
<root BTCPP_format="4">
  <BehaviorTree ID="main">
    <RecoverySequence name="rs" reset_token="{goal_revision}">
      <MockChild name="first"/>
      <MockChild name="second"/>
    </RecoverySequence>
  </BehaviorTree>
</root>)", bb);
  auto mocks = collectMocks(tree);
  ASSERT_EQ(mocks.size(), 2u);
  mocks[0]->status_fn = [] { return BT::NodeStatus::SUCCESS; };
  mocks[1]->status_fn = [] { return BT::NodeStatus::RUNNING; };

  // tick1: first SUCCESS -> cursor 1; tick2: second RUNNING -> the cursor holds.
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(mocks[0]->tick_count, 1);
  EXPECT_EQ(mocks[1]->tick_count, 1);

  // New navigation round (goal_revision advanced): the cursor resets to first and the in-flight second is halted.
  bb->set<uint64_t>("goal_revision", 1u);
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(mocks[0]->tick_count, 2);
  EXPECT_EQ(mocks[1]->halt_count, 1);
  EXPECT_EQ(mocks[1]->tick_count, 1);
}

}  // namespace
