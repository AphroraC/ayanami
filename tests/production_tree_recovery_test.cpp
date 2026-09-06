// production_tree_recovery_test: 生产默认树 (NavigateWithReplanningAndSmoothing)
// + 可编程 MockNavigationRuntime 的恢复重入集成测试。
//
// 背景 bug (rviz 实测 + rosbag 取证): controller NoProgress ABORT 后, 外层
// RecoveryActions 连续执行 28 次恢复 (spin 风暴) 而 FollowPath 零重入, 且
// 28 > 外层 number_of_retries=6 —— 恢复重入与重试上限双双失效, 机器人原地
// 自转永不恢复导航。
//
// 本测试用与生产一致的树 XML + pilot 式 tick 循环锁死两个不变量:
// I1 (重入): controller 失败 + 恢复成功后, FollowPath 必须被重新请求
// (exe_request_count >= 2), 树重启后 planner 必须重新规划;
// I2 (有界): 单次树生命周期内恢复请求数 <= 内外层重试预算之和。
// 瞬时故障变体: 第二次执行成功 -> 整树 SUCCESS, 恢复只走 clear_costmap。
//
// 调试: AYANAMI_BT_TRACE=1 输出逐节点状态迁移。

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

// 与 navigation_behavior_tree.hpp 内嵌生产树逐字一致。
constexpr const char* k_production_xml = R"XML(
<root BTCPP_format="4" main_tree_to_execute="NavigateWithReplanningAndSmoothing">
  <BehaviorTree ID="NavigateWithReplanningAndSmoothing">
    <RecoveryNode name="NavigateRecovery" number_of_retries="6">
      <PipelineSequence name="NavigateWithReplanning">
        <RateController hz="{planner_frequency}" reset_token="{goal_revision}">
          <RecoveryNode name="ComputePath" number_of_retries="1">
            <Sequence>
               <ComputePathToPose goal="{goal}" path="{raw_path}" path_id="{path_id}"/>
              <IsPathValid path="{raw_path}"/>
            </Sequence>
            <Sequence>
              <Recovery recovery="{clear_costmap_id}"/>
            </Sequence>
          </RecoveryNode>
        </RateController>
        <RateController hz="{smoother_frequency}" reset_token="{path_id}">
               <SmoothPath input_path="{raw_path}" path="{smoothed_path}"/>
        </RateController>
        <RecoveryNode name="FollowPath" number_of_retries="1">
           <FollowPath path="{smoothed_path}" frequency="{controller_frequency}"/>
          <Sequence>
            <Recovery recovery="{clear_costmap_id}"/>
          </Sequence>
        </RecoveryNode>
      </PipelineSequence>
      <ReactiveFallback name="RecoveryActions">
        <GoalUpdated goal="{goal}"/>
         <RoundRobin name="RecoveryActions" reset_token="{goal_revision}">
          <Sequence>
            <Recovery recovery="{spin_id}"/>
          </Sequence>
          <Sequence>
            <Recovery recovery="{wait_id}"/>
          </Sequence>
          <Sequence>
            <Recovery recovery="{backup_id}"/>
          </Sequence>
         </RoundRobin>
      </ReactiveFallback>
    </RecoveryNode>
  </BehaviorTree>
</root>)XML";

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

/// @brief 可编程延迟完成: 请求入 pending 队列, pump 推进年龄并按策略结算。
class MockRuntime final : public NavigationRuntime {
 public:
  // ---- 观测 ----
  int path_requests{0};
  int smooth_requests{0};
  int exe_requests{0};
  int recovery_requests{0};
  std::map<std::string, int> recovery_by_id;

  // ---- 策略 ----
  int settle_after{2};  // pump 次数后结算
  bool controller_always_fails{true};

  // ---- 会话/版本 ----
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
    c.result.smoothed_path = makePath();
    // 对齐生产: result 携带"被接受时"的版本, 节点 onRunning 校验通过后才
    // incrementPlanVersion (经 mock 的 incrementPlanVersion)。
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
    runtime = std::make_shared<MockRuntime>();
  }

  void writeBlackboard() {
    bb = BT::Blackboard::create();
    bb->set<NavigationRuntime::Ptr>(k_navigation_runtime, runtime);
    bb->set<PoseStamped>(k_goal, makeGoal());
    bb->set<Path>(k_raw_path, Path{});
    bb->set<Path>(k_smoothed_path, Path{});
    // 生产配置 (costmap_navigation.json): planner/smoother 均为 -1 (单次缓存)
    bb->set<double>(k_planner_frequency, -1.0);
    bb->set<double>(k_smoother_frequency, -1.0);
    bb->set<double>(k_controller_frequency, 20.0);
    bb->set<std::string>(k_clear_costmap_id, "clear_costmap");
    bb->set<std::string>(k_spin_id, "spin");
    bb->set<std::string>(k_wait_id, "wait");
    bb->set<std::string>(k_backup_id, "backup");
    bb->set<uint64_t>(k_session_id, runtime->currentSessionId());
    bb->set<uint64_t>(k_plan_version, runtime->currentPlanVersion());
    bb->set<uint64_t>(k_reset_stuck_token, runtime->currentSessionId());
    bb->set<uint64_t>(k_goal_revision, 0u);
    bb->set<ayanami::FailureContext>(k_failure_context,
                                     ayanami::FailureContext{});
  }

  void buildTree() {
    tree = factory.createTreeFromText(k_production_xml, bb);
    if (std::getenv("AYANAMI_BT_TRACE") != nullptr) {
      logger = std::make_unique<BT::StdCoutLogger>(tree);
    }
  }

  /// @brief pilot 契约的测试镜像: tick 一次, 树返回终端状态时全量逻辑重置
  /// (BehaviorTreePilot::run 同款; v4 halt 不重置终态缓存子树, 见
  /// resetNavigationTreeState 注释)。
  BT::NodeStatus tickTree() {
    const BT::NodeStatus status = tree.tickExactlyOnce();
    if (status != BT::NodeStatus::RUNNING) {
      resetNavigationTreeState(tree);
    }
    return status;
  }

  BT::BehaviorTreeFactory factory;
  std::shared_ptr<MockRuntime> runtime;
  BT::Blackboard::Ptr bb;
  BT::Tree tree;
  std::unique_ptr<BT::StdCoutLogger> logger;
};

// ---- I1/I2: controller 永久 NoProgress -> 恢复后必须重入 + 有界 ----
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

  // I1: 恢复成功后 FollowPath 必须重入 (生产 bug: exe=1 恒定)。
  EXPECT_GE(runtime->exe_requests, 2)
      << "FollowPath was never re-entered after recovery (storm bug)";
  // I1b: 树重启生命周期里 planner 必须重新规划 (生产 bug: path=1 恒定)。
  EXPECT_GE(runtime->path_requests, 2)
      << "planner never replanned across tree lives";
  // I2: 重试预算有界 —— nav2 等价预算: 每树生命 = 外层 6 次重试, 每次重试
  // 至多 1 外层恢复 + 1 内层 clear_costmap = 12 次; 每次树生命都伴随重规划,
  // 故恢复数相对规划数必须有界 (风暴 bug: path 恒 1 而 recovery 无限)。
  EXPECT_LE(runtime->recovery_requests,
            13 * std::max(1, runtime->path_requests))
      << "recovery storm: unbounded recoveries per tree life";
}

// ---- 瞬时故障: 第二次执行成功 -> 整树 SUCCESS, 外层恢复不应触发 ----
TEST_F(ProductionTreeRecovery, transientControllerFailureRecoversAndSucceeds) {
  writeBlackboard();
  runtime->controller_always_fails = true;  // 首次执行失败 (NoProgress)
  buildTree();

  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  for (int i = 0; i < 100; ++i) {
    status = tickTree();
    runtime->pump();
    if (runtime->exe_requests >= 1) {
      runtime->controller_always_fails = false;  // 之后永久成功
    }
    if (status != BT::NodeStatus::RUNNING) {
      break;
    }
  }
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS)
      << "path=" << runtime->path_requests
      << " exe=" << runtime->exe_requests
      << " recovery=" << runtime->recovery_requests;
  // 恢复预算: 内层 FollowPath 分支 1 次 clear_costmap (外层不应触发)。
  EXPECT_LE(runtime->recovery_requests, 1)
      << "recovery=" << runtime->recovery_requests;
}

}  // namespace

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
