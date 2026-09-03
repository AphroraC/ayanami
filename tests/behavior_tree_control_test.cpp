#include <gtest/gtest.h>

#include <ayanami/behavior_tree/blackboard_protocol.hpp>
#include <ayanami/behavior_tree/condition_nodes.hpp>
#include <ayanami/behavior_tree/pipeline_sequence.hpp>
#include <ayanami/behavior_tree/rate_controller.hpp>
#include <ayanami/behavior_tree/recovery_node.hpp>
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

// 可返回任意 NodeStatus 的异步 Action 桩（需 RUNNING 能力，故继承 ActionNodeBase 而非
// SyncActionNode）。tick 次数与每次返回的状态由测试直接控制。
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

  // fail #1 -> recovery SUCCESS -> 重试, 返回 RUNNING
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  // fail #2 -> recovery SUCCESS -> 重试, 返回 RUNNING
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  // fail #3 -> retries(2) 耗尽 -> 整体 FAILURE
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

TEST(BehaviorTreeControlTest, recoveryNodeResetsRetriesAfterSuccess) {
  auto factory = makeFactory();
  factory.registerNodeType<RecoveryNode>("RecoveryNode");
  auto bb = BT::Blackboard::create();
  auto tree = factory.createTreeFromText(k_recovery_xml, bb);

  auto mocks = collectMocks(tree);
  ASSERT_EQ(mocks.size(), 2u);
  mocks[1]->status_fn = [] { return BT::NodeStatus::SUCCESS; };

  // 首轮成功 -> retry 计数复位
  mocks[0]->status_fn = [] { return BT::NodeStatus::SUCCESS; };
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(mocks[0]->tick_count, 1);
  EXPECT_EQ(mocks[1]->tick_count, 0);

  // 复位后重新计数: 2 次失败 + 2 次 recovery 成功后第 3 次失败耗尽
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

  // 100Hz tick 100ms (10 tick, 每 10ms): hz=10 -> 周期 100ms, 首 tick 立即执行,
  // 其余在冷却期内返回缓存, 因此 child 只被执行 1 次。
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

  // 周期 (100ms) 内不执行, 跨过周期后再次执行
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

  // hz=-1: 单次执行后缓存 SUCCESS
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

  // 单次模式缓存 SUCCESS, 后续 tick 不再执行 child
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(mocks[0]->tick_count, 1);

  // reset_token 变化 (新 goal -> goal_revision 递增): 无视缓存立即重执行。
  // 这是 "新 goal 立即重规划/重平滑" 的机制 (默认树 planner/smoother
  // RateController 均绑定 reset_token="{goal_revision}")。
  bb->set<uint64_t>("goal_revision", 1u);
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(mocks[0]->tick_count, 2);

  // token 未变时继续缓存
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

  // child0 RUNNING 时下次 tick 依然执行 child0
  mocks[0]->status_fn = [] { return BT::NodeStatus::RUNNING; };
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(mocks[0]->tick_count, 2);
  EXPECT_EQ(mocks[1]->tick_count, 0);

  // child0 SUCCESS 后从 child0 转 child1 继续
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

  // child0 直接 FAILURE -> 整树 FAILURE
  mocks[0]->status_fn = [] { return BT::NodeStatus::FAILURE; };
  mocks[1]->status_fn = [] { return BT::NodeStatus::SUCCESS; };
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(mocks[0]->tick_count, 1);
  EXPECT_EQ(mocks[1]->tick_count, 0);

  // child1 FAILURE -> 整树 FAILURE
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

  // 首次 tick 初始化基线 -> FAILURE
  bb->set<PoseStamped>(k_goal, makePose(0.0));
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::FAILURE);

  // 黑板 goal 变化 -> SUCCESS
  auto changed = makePose(5.0);
  changed.header.stamp = ayanami::Timestamp{static_cast<std::int64_t>(1)};
  bb->set<PoseStamped>(k_goal, changed);
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::SUCCESS);

  // 未变化 -> FAILURE
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::FAILURE);
}

TEST(BehaviorTreeControlTest, isGoalReachedWithinTolerance) {
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<IsGoalReached>("IsGoalReached");
  auto bb = BT::Blackboard::create();
  auto tree = factory.createTreeFromText(R"(
<root BTCPP_format="4">
  <BehaviorTree ID="main">
    <IsGoalReached goal="{goal}" robot_pose="{robot_pose}" tolerance="0.5"/>
  </BehaviorTree>
</root>)",
                                          bb);

  bb->set<PoseStamped>("robot_pose", makePose(0.0));

  bb->set<PoseStamped>(k_goal, makePose(1.0));
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::FAILURE);

  bb->set<PoseStamped>(k_goal, makePose(0.25));
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::SUCCESS);
}

TEST(BehaviorTreeControlTest, isFarFromGoalBeyondThreshold) {
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<IsFarFromGoal>("IsFarFromGoal");
  auto bb = BT::Blackboard::create();
  auto tree = factory.createTreeFromText(R"(
<root BTCPP_format="4">
  <BehaviorTree ID="main">
    <IsFarFromGoal goal="{goal}" robot_pose="{robot_pose}" threshold="0.5"/>
  </BehaviorTree>
</root>)",
                                          bb);

  bb->set<PoseStamped>("robot_pose", makePose(0.0));

  bb->set<PoseStamped>(k_goal, makePose(1.0));
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::SUCCESS);

  bb->set<PoseStamped>(k_goal, makePose(0.25));
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::FAILURE);
}

TEST(BehaviorTreeControlTest, isRobotStuckTokenResetClearsState) {
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<IsRobotStuck>("IsRobotStuck");
  auto bb = BT::Blackboard::create();
  auto tree = factory.createTreeFromText(R"(
<root BTCPP_format="4">
  <BehaviorTree ID="main">
    <IsRobotStuck robot_odometry="{robot_odometry}" robot_pose="{robot_pose}"
                  goal="{goal}" reset_stuck_token="{reset_stuck_token}"
                  stuck_velocity_threshold="0.001" goal_tolerance="0.5"
                  consecutive_stuck_ticks="3"/>
  </BehaviorTree>
</root>)",
                                          bb);

  // 静止 + 远离目标 (连续 3 tick 判定卡住)
  bb->set<uint64_t>(k_reset_stuck_token, 1u);
  bb->set<Odometry>("robot_odometry", Odometry{});
  bb->set<PoseStamped>("robot_pose", makePose(0.0));
  bb->set<PoseStamped>(k_goal, makePose(10.0));

  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::SUCCESS);

  // reset_stuck_token 变化 -> 状态清零 (防 static 跨任务残留)
  bb->set<uint64_t>(k_reset_stuck_token, 2u);
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::SUCCESS);
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
