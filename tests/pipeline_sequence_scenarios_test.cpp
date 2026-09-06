// pipeline_sequence_scenarios_test: PipelineSequence / RecoveryNode 关键场景
// 专项测试 (对照 nav2 语义), 以及 RateController reset_token 语义测试。
//
// 覆盖场景:
// S1 planner RUNNING + controller RUNNING -> 两者都被 tick, 整体 RUNNING
// S2 planner SUCCESS + controller RUNNING -> 两者都被 tick, 整体 RUNNING
// S3 planner FAILURE -> 整体 FAILURE, controller 不被 tick
// S4 controller FAILURE -> 整体 FAILURE
// S5 planner 在 controller 运行中重规划 -> controller 每 tick 仍被 tick
// (跟踪不中断, in-place 跟随)
// S6 goal 更新触发 planner 重入 -> controller 持续被 tick
// R1 RateController reset_token 变化 -> 在飞 child 被 halt 后重新 onStart
// R2 RecoveryNode: child0 FAILURE -> recovery SUCCESS -> 主任务重试 RUNNING

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

// 可编程 mock child: 状态函数可切换 (模拟 planner 在 controller 运行中重新
// 进入 RUNNING 等), 记录 tick / halt 次数。需 RUNNING 能力, 继承
// ActionNodeBase 而非 SyncActionNode (BT.CPP 禁止 Sync 节点返回 RUNNING)。
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

// S1: planner RUNNING + controller RUNNING -> 两者都被 tick, 整体 RUNNING。
TEST(PipelineSequenceScenarios, plannerRunningAndControllerRunningTickBoth) {
  auto factory = makeFactory();
  auto tree = factory.createTreeFromText(k_two_child_xml);
  auto mocks = collectMocks(tree);
  ASSERT_EQ(mocks.size(), 2u);
  mocks[0]->status_fn = [] { return BT::NodeStatus::RUNNING; };
  mocks[1]->status_fn = [] { return BT::NodeStatus::RUNNING; };

  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  // controller 尚未被推进到 (前序未成功): 只 tick planner。
  EXPECT_EQ(mocks[0]->tick_count, 1);
  EXPECT_EQ(mocks[1]->tick_count, 0);
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(mocks[0]->tick_count, 2);
  EXPECT_EQ(mocks[1]->tick_count, 0);
}

// S2: planner SUCCESS + controller RUNNING -> 整体 RUNNING。
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

// S3: planner FAILURE -> 整体 FAILURE, controller 从未被 tick。
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

// S4: controller FAILURE -> 整体 FAILURE (planner 已 SUCCESS 仍被重 tick 一次)。
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

// S5: planner 在 controller 运行中产生新路径 (planner 重入 RUNNING) ->
// controller 每 tick 仍被 tick (跟踪不中断, 新路径经 in-place 更新跟随)。
TEST(PipelineSequenceScenarios,
     plannerReentryKeepsControllerTickingDuringReplan) {
  auto factory = makeFactory();
  auto tree = factory.createTreeFromText(k_two_child_xml);
  auto mocks = collectMocks(tree);
  ASSERT_EQ(mocks.size(), 2u);
  mocks[1]->status_fn = [] { return BT::NodeStatus::RUNNING; };

  // 周期1-2: 初次规划; 周期3: 新路径完成; 周期4: goal 更新后 planner 重入。
  mocks[0]->status_fn = [] { return BT::NodeStatus::RUNNING; };
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  mocks[0]->status_fn = [] { return BT::NodeStatus::RUNNING; };
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  mocks[0]->status_fn = [] { return BT::NodeStatus::SUCCESS; };
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(mocks[1]->tick_count, 1);  // controller 已在跟随
  mocks[0]->status_fn = [] { return BT::NodeStatus::RUNNING; };
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  // controller 在 planner 重规划期间持续被 tick (不因 replan 停摆)。
  EXPECT_EQ(mocks[1]->tick_count, 3);
}

// S6: goal 更新时 planner 重入 RUNNING 而 controller 保持 RUNNING ->
// PipelineSequence 对两个分支都保持 tick (BT 不中断任何一侧)。
TEST(PipelineSequenceScenarios, goalUpdateReticksPlannerAndController) {
  auto factory = makeFactory();
  auto tree = factory.createTreeFromText(k_two_child_xml);
  auto mocks = collectMocks(tree);
  ASSERT_EQ(mocks.size(), 2u);
  mocks[0]->status_fn = [] { return BT::NodeStatus::SUCCESS; };
  mocks[1]->status_fn = [] { return BT::NodeStatus::RUNNING; };
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);

  // goal 更新: planner 重新 RUNNING; controller 继续跟旧路径。
  mocks[0]->status_fn = [] { return BT::NodeStatus::RUNNING; };
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(mocks[0]->tick_count, 2);
  EXPECT_EQ(mocks[1]->tick_count, 2);
}

// R1: RateController reset_token 变化 -> 在飞 RUNNING child 被 halt (onHalted),
// 同一 tick 重新 onStart (全新输入重新开始, 而非继续消费旧输入)。
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
      // 独占持有顺序与树析构一致 (shared_ptr 视图), 测试生命周期内安全。
      observer.reset(found, [](HaltObserverAction*) {});
    }
  });
  ASSERT_NE(observer, nullptr);

  // 首 tick: onStart, RUNNING; 再 tick: onRunning 轮询。
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(observer->start_count, 1);
  EXPECT_EQ(observer->halt_count, 0);

  // 令牌变化 (新路径落地语义): 在飞 child 被 halt, 同 tick 重新 onStart。
  bb->set<uint64_t>("reset_token", 2u);
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(observer->halt_count, 1);
  EXPECT_EQ(observer->start_count, 2);
}

// R2: RecoveryNode child0 FAILURE -> recovery child SUCCESS -> 返回 RUNNING
// 让主任务重试 (nav2 RecoveryNode 语义)。
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

  // tick1: main FAILURE -> recovery SUCCESS -> 整体 RUNNING (重试入口)。
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(mocks[0]->tick_count, 1);
  EXPECT_EQ(mocks[1]->tick_count, 1);
  // tick2: 主任务重试 SUCCESS -> 整体 SUCCESS。
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(mocks[0]->tick_count, 2);
  EXPECT_EQ(mocks[1]->tick_count, 1);
}


// R3: RateController hz>0 —— RUNNING child 立即轮询 (nav2 语义), 速率只限制
// 新一轮; 计时器仅在 child SUCCESS 时复位。
TEST(RateControllerPollingTest, runningChildIsPolledEveryTickAndTimerResetsOnSuccess) {
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<RateController>("RateController");
  factory.registerNodeType<MockChild>("MockChild");

  using Clock = std::chrono::system_clock;  // BT::TimePoint 基于 system_clock
  Clock::time_point now = Clock::now();
  auto clock_fn = [&now] { return now; };

  BT::NodeConfig config;
  config.input_ports["hz"] = "10.0";  // period=100ms
  RateController::ClockFn clock = clock_fn;
  RateController rc("rc", config, clock);

 // 手工挂 child (DecoratorNode::setChild 接管裸指针, 生命周期由树外
 // shared_ptr 持有即可 —— 与树析构同序)。
  auto child_owned = std::make_shared<MockChild>("child", BT::NodeConfig());
  rc.setChild(child_owned.get());
  MockChild* mock = child_owned.get();

  const auto tick = [&](int advance_ms) {
    now += std::chrono::milliseconds(advance_ms);
    return rc.executeTick();
  };

  mock->status_fn = [] { return BT::NodeStatus::RUNNING; };
  EXPECT_EQ(tick(0), BT::NodeStatus::RUNNING);      // 首 tick (started)
  EXPECT_EQ(mock->tick_count, 1);
  EXPECT_EQ(tick(10), BT::NodeStatus::RUNNING);     // 周期未到但 RUNNING -> 立即轮询
  EXPECT_EQ(mock->tick_count, 2);
  EXPECT_EQ(tick(10), BT::NodeStatus::RUNNING);
  EXPECT_EQ(mock->tick_count, 3);

 // 完成 (SUCCESS): 计时器复位
  mock->status_fn = [] { return BT::NodeStatus::SUCCESS; };
  EXPECT_EQ(tick(10), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(mock->tick_count, 4);

 // SUCCESS 后周期未到 (累计 20ms < 100ms): 不再 tick
  mock->status_fn = [] { return BT::NodeStatus::SUCCESS; };
  EXPECT_EQ(tick(20), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(mock->tick_count, 4);

 // 周期到达: 新一轮
  EXPECT_EQ(tick(90), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(mock->tick_count, 5);
}

// ---- H3 常驻树复位协议: RoundRobin / RecoverySequence reset_token ----

TEST(RoundRobinResetTest, tokenChangeRestartsCursorAndHaltsInFlightChild) {
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<ayanami::behavior_tree::RoundRobin>("RoundRobin");
  factory.registerNodeType<MockChild>("MockChild");
  auto bb = BT::Blackboard::create();
  bb->set<uint64_t>("goal_revision", 0u);
  auto tree = factory.createTreeFromText(R"(
<root BTCPP_format="4">
  <BehaviorTree ID="main">
    <RoundRobin name="rr" reset_token="{goal_revision}">
      <MockChild name="first"/>
      <MockChild name="second"/>
    </RoundRobin>
  </BehaviorTree>
</root>)", bb);
  auto mocks = collectMocks(tree);
  ASSERT_EQ(mocks.size(), 2u);
  mocks[0]->status_fn = [] { return BT::NodeStatus::SUCCESS; };
  mocks[1]->status_fn = [] { return BT::NodeStatus::RUNNING; };

  // tick1: first SUCCESS -> 游标 1; tick2: second RUNNING -> 游标停住。
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(mocks[0]->tick_count, 1);
  EXPECT_EQ(mocks[1]->tick_count, 1);

  // 新导航轮 (goal_revision 推进): 游标复位回 first, 在飞 second 被 halt。
  bb->set<uint64_t>("goal_revision", 1u);
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(mocks[0]->tick_count, 2);
  EXPECT_EQ(mocks[1]->halt_count, 1);
  EXPECT_EQ(mocks[1]->tick_count, 1);
}

TEST(RecoverySequenceResetTest, tokenChangeRestartsFromFirstRecovery) {
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<ayanami::behavior_tree::RecoverySequence>("RecoverySequence");
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
  MockChild* first = mocks[0];
  mocks[0]->status_fn = [first] {
    return first->tick_count <= 1 ? BT::NodeStatus::FAILURE
                                  : BT::NodeStatus::SUCCESS;
  };
  mocks[1]->status_fn = [] { return BT::NodeStatus::RUNNING; };

  // tick1: first FAILURE -> second RUNNING (游标 1, 恢复进行中)。
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(mocks[0]->tick_count, 1);
  EXPECT_EQ(mocks[1]->tick_count, 1);

  // 新导航轮: 复位游标, 从 first 重新开始 (本轮 first 已可用)。
  bb->set<uint64_t>("goal_revision", 1u);
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(mocks[0]->tick_count, 2);
  EXPECT_EQ(mocks[1]->halt_count, 1);
}

}  // namespace
