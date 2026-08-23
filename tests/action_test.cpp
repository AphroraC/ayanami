#include <gtest/gtest.h>

#include <ayanami/abstract/components/action_server.hpp>
#include <ayanami/abstract/components/controller_action.hpp>
#include <ayanami/abstract/components/planner_action.hpp>
#include <ayanami/abstract/components/recovery_action.hpp>
#include <ayanami/abstract/components/smoother_action.hpp>
#include <ayanami/abstract/core/abstract_goal_checker.hpp>
#include <ayanami/abstract/core/abstract_progress_checker.hpp>
#include <ayanami/interfaces/callbacks.hpp>
#include <ayanami/messages/failure.hpp>

#include <Eigen/Geometry>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace {

using ayanami::geometry_msgs::PoseStamped;
using ayanami::FailureCode;

using ayanami::geometry_msgs::TwistStamped;
using ayanami::nav_msgs::Path;

// 阻塞直到被 cancel: 验证取消/抢占中断 RUNNING 的 planner
class BlockingPlanner : public ayanami::abstract::AbstractPlanner {
public:
  FailureCode makePlan(const PoseStamped& start, const PoseStamped& goal, double,
                       Path& plan, double&, std::string&) override {
    calls_++;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      entered_.notify_all();
      release_.wait_for(lock, std::chrono::milliseconds(2000),
                        [this] { return canceled_.load(); });
    }
    if (canceled_.exchange(false)) {
      return FailureCode::None;  // 被取消: 让 execution 判 CANCELED
    }
    plan.poses.push_back(start);
    plan.poses.push_back(goal);
    return FailureCode::None;
  }
  bool cancel() override {
    canceled_ = true;
    release_.notify_all();
    return true;
  }
  std::atomic<int> calls_{0};
  std::mutex mutex_;
  std::condition_variable entered_;
  std::condition_variable release_;
  std::atomic<bool> canceled_{false};
};

// 即时成功的 planner: 填 path, 验 SUCCEEDED + path 非空
class FillingPlanner : public ayanami::abstract::AbstractPlanner {
public:
  FailureCode makePlan(const PoseStamped&, const PoseStamped&, double, Path& plan,
                       double& cost, std::string&) override {
    calls_++;
    plan.header.frame_id = "map";
    plan.poses.resize(2);
    cost = 1.5;
    return FailureCode::None;
  }
  bool cancel() override { return true; }
  std::atomic<int> calls_{0};
};

// controller: 到达判定交给 goal_checker (完成后 N 帧 compute 触发到达)。
class TestController : public ayanami::abstract::AbstractController {
public:
  FailureCode computeVelocityCommands(
      const PoseStamped&, const TwistStamped&, TwistStamped& cmd_vel,
      std::string&,
      const ayanami::abstract::AbstractGoalChecker::Ptr&) override {
    calls_++;
    cmd_vel.twist.linear.x() = 0.5;
    return FailureCode::None;
  }
  bool setPlan(const Path&) override { return true; }
  bool cancel() override { return true; }
  std::atomic<int> calls_{0};
};

// ---- checker 测试桩 ----

class LambdaGoalChecker : public ayanami::abstract::AbstractGoalChecker {
 public:
  explicit LambdaGoalChecker(std::function<bool()> reached)
      : reached_(std::move(reached)) {}
  void reset() override {}
  bool isGoalReached(const PoseStamped&, const PoseStamped&,
                     const TwistStamped&) override {
    return reached_();
  }
  ayanami::abstract::GoalCheckerTolerances getTolerances() const override {
    return {};
  }

 private:
  std::function<bool()> reached_;
};

class AlwaysTrueProgressChecker
    : public ayanami::abstract::AbstractProgressChecker {
 public:
  void reset() override {}
  bool check(const PoseStamped&) override { return true; }
};

// ControllerAction 派生: createExecution 内注入 checker (server 由
// NavigationServerBase 注入, 直连 action 测试用此子类)。
class TestControllerAction : public ayanami::components::ControllerAction {
 public:
  using ControllerAction::ControllerAction;
  void initCheckers(ayanami::abstract::AbstractGoalChecker::Ptr goal,
                    ayanami::abstract::AbstractProgressChecker::Ptr progress) {
    goal_ = std::move(goal);
    progress_ = std::move(progress);
  }

 protected:
  ayanami::components::ControllerExecution::Ptr createExecution() override {
    auto exec = ControllerAction::createExecution();
    exec->setGoalChecker(goal_);
    exec->setProgressChecker(progress_);
    return exec;
  }

 private:
  ayanami::abstract::AbstractGoalChecker::Ptr goal_;
  ayanami::abstract::AbstractProgressChecker::Ptr progress_;
};

class FillingSmoother : public ayanami::abstract::AbstractSmoother {
public:
  FailureCode smoothPath(const Path& in, Path& out, double, bool,
                         std::string&) override {
    out.header.frame_id = "map";
    out.poses = in.poses;
    return FailureCode::None;
  }
  bool cancel() override { return true; }
};

class TestRecovery : public ayanami::abstract::AbstractRecovery {
public:
  FailureCode runRecovery(std::string&) override { return FailureCode::None; }
  bool cancel() override { return true; }
};

template <typename Task>
bool waitReady(Task& t, int max_ms = 3000) {
  for (int i = 0; i < max_ms / 10 && !t.ready(); ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return t.ready();
}

using Server = ayanami::components::ActionServer<
    ayanami::actions::GetPathGoal, ayanami::actions::GetPathResult,
    ayanami::actions::GetPathFeedback>;

}  // namespace

// ---- ActionServer 协议 (复刻 legacy action_protocol_test, 按 Ruling #2 ACTIVE 通态) ----

TEST(ActionProtocol, CompletionCarriesResultAndCleansGoal) {
  Server server;
  auto task = server.acceptGoal({});
  ASSERT_TRUE(task.valid());
  EXPECT_EQ(server.activeGoalCount(), 1U);

  ayanami::actions::GetPathResult result;
  result.failure = FailureCode::None;
  result.path.poses.resize(2);
  ASSERT_TRUE(server.publishResult(
      task.goal_id(), ayanami::actions::GoalStatus::SUCCEEDED, result));

  ASSERT_TRUE(waitReady(task));
  EXPECT_EQ(task.status(), ayanami::actions::GoalStatus::SUCCEEDED);
  EXPECT_EQ(task.result().failure, FailureCode::None);
  EXPECT_FALSE(task.result().path.poses.empty());
  EXPECT_EQ(server.activeGoalCount(), 0U);
  // 已完成的 goal 已清理, 不能再次 finish
  EXPECT_FALSE(server.publishResult(
      task.goal_id(), ayanami::actions::GoalStatus::SUCCEEDED, result));
}

TEST(ActionProtocol, CancelIsARequestUntilExecutionFinishes) {
  Server server;
  auto task = server.acceptGoal({});
  ASSERT_TRUE(task.cancel());
  EXPECT_TRUE(server.isCancelRequested(task.goal_id()));
  EXPECT_EQ(server.activeGoalCount(), 1U);

  ayanami::actions::GetPathResult result;
  result.failure = FailureCode::None;
  ASSERT_TRUE(server.publishResult(
      task.goal_id(), ayanami::actions::GoalStatus::CANCELED, result));
  ASSERT_TRUE(waitReady(task));
  EXPECT_EQ(task.status(), ayanami::actions::GoalStatus::CANCELED);
  EXPECT_EQ(server.activeGoalCount(), 0U);
}

TEST(ActionProtocol, CompletedGoalsDoNotAccumulate) {
  Server server;
  for (uint64_t version = 1; version <= 10000; ++version) {
    auto task = server.acceptGoal({});
    ASSERT_TRUE(task.valid());
    ayanami::actions::GetPathResult result;
    result.session_id = version;
    ASSERT_TRUE(server.publishResult(
        task.goal_id(), ayanami::actions::GoalStatus::SUCCEEDED, result));
  }
  EXPECT_EQ(server.activeGoalCount(), 0U);
}

// ---- PlannerAction ----

TEST(PlannerAction, AcceptGoalReturnsTaskHandleAndSucceedsWithPath) {
  auto planner = std::make_shared<FillingPlanner>();
  ayanami::components::PlannerAction action(planner);

  ayanami::actions::GetPathGoal goal;
  goal.start = PoseStamped::fromIsometry("map", Eigen::Isometry3d::Identity());
  goal.goal = PoseStamped::fromIsometry("map", Eigen::Isometry3d::Identity());
  goal.session_id = 7;
  goal.plan_version = 11;

  auto task = action.acceptGoal(goal);
  ASSERT_TRUE(task.valid());
  ASSERT_TRUE(waitReady(task));
  EXPECT_EQ(task.status(), ayanami::actions::GoalStatus::SUCCEEDED);
  EXPECT_FALSE(task.result().path.poses.empty());
  EXPECT_EQ(task.result().path.header.frame_id, "map");
  EXPECT_EQ(task.result().session_id, 7U);
  EXPECT_EQ(task.result().plan_version, 11U);
  EXPECT_GE(planner->calls_, 1);
}

TEST(PlannerAction, CancelSlotInterruptsRunningExecution) {
  auto planner = std::make_shared<BlockingPlanner>();
  ayanami::components::PlannerAction action(planner);

  auto task = action.acceptGoal({});
  ASSERT_TRUE(task.valid());
  {
    std::unique_lock<std::mutex> lock(planner->mutex_);
    ASSERT_TRUE(planner->entered_.wait_for(
        lock, std::chrono::milliseconds(1000),
        [&] { return planner->calls_.load() >= 1; }));
  }

  action.cancel(0);
  ASSERT_TRUE(waitReady(task));
  EXPECT_EQ(task.status(), ayanami::actions::GoalStatus::CANCELED);
}

TEST(PlannerAction, TaskHandleCancelForwardsToExecution) {
  auto planner = std::make_shared<BlockingPlanner>();
  ayanami::components::PlannerAction action(planner);

  auto task = action.acceptGoal({});
  ASSERT_TRUE(task.valid());
  {
    std::unique_lock<std::mutex> lock(planner->mutex_);
    ASSERT_TRUE(planner->entered_.wait_for(
        lock, std::chrono::milliseconds(1000),
        [&] { return planner->calls_.load() >= 1; }));
  }

  ASSERT_TRUE(task.cancel());
  ASSERT_TRUE(waitReady(task));
  EXPECT_EQ(task.status(), ayanami::actions::GoalStatus::CANCELED);
}

TEST(PlannerAction, PreemptSameSlotJoinsOldGoalBeforeNewStarts) {
  auto planner = std::make_shared<BlockingPlanner>();
  ayanami::components::PlannerAction action(planner);

  auto first = action.acceptGoal({});
  ASSERT_TRUE(first.valid());
  {
    std::unique_lock<std::mutex> lock(planner->mutex_);
    ASSERT_TRUE(planner->entered_.wait_for(
        lock, std::chrono::milliseconds(1000),
        [&] { return planner->calls_.load() >= 1; }));
  }

  auto second = action.acceptGoal({});
  ASSERT_TRUE(second.valid());

  // 旧 goal 被 preempt → CANCELED; 新 goal 跑完 → SUCCEEDED
  ASSERT_TRUE(waitReady(first));
  EXPECT_EQ(first.status(), ayanami::actions::GoalStatus::CANCELED);
  ASSERT_TRUE(waitReady(second));
  EXPECT_EQ(second.status(), ayanami::actions::GoalStatus::SUCCEEDED);
  // 串行: 第二个在第一个 join 后启动 (插件从不并发)
  EXPECT_EQ(planner->calls_.load(), 2);
}

TEST(PlannerAction, ResultEmittedOnBypassSlotWhileTaskHandleIsMain) {
  auto planner = std::make_shared<FillingPlanner>();
  ayanami::components::PlannerAction action(planner);

  std::atomic<int> bypass_hits{0};
  ayanami::actions::GoalStatus bypass_status =
      ayanami::actions::GoalStatus::PENDING;
  int cb_id = ayanami::interfaces::NavigationCallbacks::on_get_path_result.add(
      [&](const ayanami::actions::GoalResult<ayanami::actions::GetPathResult>& gr) {
        bypass_hits++;
        bypass_status = gr.status;
      });

  auto task = action.acceptGoal({});
  ASSERT_TRUE(waitReady(task));
  EXPECT_EQ(task.status(), ayanami::actions::GoalStatus::SUCCEEDED);
  EXPECT_GE(bypass_hits.load(), 1);
  EXPECT_EQ(bypass_status, ayanami::actions::GoalStatus::SUCCEEDED);

  ayanami::interfaces::NavigationCallbacks::on_get_path_result.remove(cb_id);
}

// ---- 其余 3 个 action 最小验收 ----

TEST(ControllerAction, RunsOneCycleAndSucceeds) {
  auto controller = std::make_shared<TestController>();
  TestControllerAction action(controller);
  action.initCheckers(
      std::make_shared<LambdaGoalChecker>(
          [&controller] { return controller->calls_ >= 3; }),
      std::make_shared<AlwaysTrueProgressChecker>());

  ayanami::actions::ExePathGoal goal;
  goal.path.poses.resize(4);
  auto task = action.acceptGoal(goal);
  ASSERT_TRUE(waitReady(task));
  EXPECT_EQ(task.status(), ayanami::actions::GoalStatus::SUCCEEDED);
  EXPECT_TRUE(task.result().reached);
  EXPECT_EQ(task.result().failure, FailureCode::None);
  EXPECT_GE(controller->calls_, 3);
}

TEST(SmootherAction, SmoothsPathAndSucceeds) {
  auto smoother = std::make_shared<FillingSmoother>();
  ayanami::components::SmootherAction action(smoother);

  ayanami::actions::SmoothPathGoal goal;
  goal.path.poses.resize(3);
  auto task = action.acceptGoal(goal);
  ASSERT_TRUE(waitReady(task));
  EXPECT_EQ(task.status(), ayanami::actions::GoalStatus::SUCCEEDED);
  EXPECT_EQ(task.result().smoothed_path.poses.size(), 3U);
}

TEST(RecoveryAction, SingleRunDeliversResult) {
  auto recovery = std::make_shared<TestRecovery>();
  ayanami::components::RecoveryAction action(recovery);

  ayanami::actions::RecoveryGoal goal;
  goal.recovery_id = "spin";
  auto task = action.acceptGoal(goal);
  ASSERT_TRUE(waitReady(task));
  EXPECT_EQ(task.status(), ayanami::actions::GoalStatus::SUCCEEDED);
  EXPECT_EQ(task.result().failure, FailureCode::None);
}
