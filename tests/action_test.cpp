#include <gtest/gtest.h>

#include <ayanami/abstract/components/action_server.hpp>
#include <ayanami/abstract/components/controller_action.hpp>
#include <ayanami/abstract/components/planner_action.hpp>
#include <ayanami/abstract/components/recovery_action.hpp>
#include <ayanami/abstract/components/smoother_action.hpp>
#include <ayanami/abstract/core/abstract_goal_checker.hpp>
#include <ayanami/abstract/core/abstract_progress_checker.hpp>
#include <ayanami/interfaces/callbacks.hpp>
#include <ayanami/abstract/components/robot_state_controller.hpp>
#include <ayanami/messages/failure.hpp>
#include <ayanami/types.hpp>

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

using ayanami::geometry_msgs::TwistStamped;
using ayanami::nav_msgs::Path;
using ayanami::ErrorCode;
using ayanami::ResultT;
using ayanami::RecoveryCycleData;
using ayanami::RecoveryStatus;

/// Blocks until cancelled: verifies cancel/preemption interrupts a RUNNING planner
class BlockingPlanner : public ayanami::abstract::AbstractPlanner {
public:
  ResultT<void> makePlan(const PoseStamped& start, const PoseStamped& goal,
                          double, ayanami::nav_msgs::Path& plan_out) override {
    calls++;
    {
      std::unique_lock<std::mutex> lock(mutex);
      entered.notify_all();
      release.wait_for(lock, std::chrono::milliseconds(2000),
                        [this] { return canceled.load(); });
    }
    // The cancelled path also returns a success path: the execution detects cancel_requested and yields CANCELED.
    (void)canceled.exchange(false);
    ayanami::nav_msgs::Path plan;
    plan.poses.push_back(start);
    plan.poses.push_back(goal);
    plan_out = std::move(plan);
    return ResultT<void>::success();
  }
  bool cancel() override {
    canceled = true;
    release.notify_all();
    return true;
  }
  std::atomic<int> calls{0};
  std::mutex mutex;
  std::condition_variable entered;
  std::condition_variable release;
  std::atomic<bool> canceled{false};
};

/// Instant-success planner: fills the path; verifies SUCCEEDED + non-empty path
class FillingPlanner : public ayanami::abstract::AbstractPlanner {
public:
  ResultT<void> makePlan(const PoseStamped&, const PoseStamped&,
                          double, ayanami::nav_msgs::Path& plan_out) override {
    calls++;
    ayanami::nav_msgs::Path plan;
    plan.header.frame_id = "map";
    plan.poses.resize(2);
    plan_out = std::move(plan);
    return ResultT<void>::success();
  }
  bool cancel() override { return true; }
  std::atomic<int> calls{0};
};

/// Controller: the arrival decision is delegated to the goal_checker (reached after N compute frames).
class TestController : public ayanami::abstract::AbstractController {
public:
  ResultT<void> computeVelocityCommand(const PoseStamped&,
                                           const TwistStamped&, ayanami::geometry_msgs::TwistStamped& cmd) override {
    calls++;
    cmd = TwistStamped{};
    cmd.twist.linear.x() = 0.5;
    return ResultT<void>::success();
  }
  bool setPlan(const Path&) override { return true; }
  bool cancel() override { return true; }
  std::atomic<int> calls{0};
};

// ---- checker test stubs ----

class LambdaGoalChecker : public ayanami::abstract::AbstractGoalChecker {
 public:
  explicit LambdaGoalChecker(std::function<bool()> reached)
      : reached(std::move(reached)) {}
  void reset() override {}
  bool isGoalReached(const PoseStamped&, const PoseStamped&,
                     const TwistStamped&) override {
    return reached();
  }
  ayanami::GoalCheckerTolerances getTolerances() const override {
    return {};
  }

 private:
  std::function<bool()> reached;
};

class AlwaysTrueProgressChecker
    : public ayanami::abstract::AbstractProgressChecker {
 public:
  void reset() override {}
  bool check(const PoseStamped&) override { return true; }
};

/// ControllerAction derivative: injects the checker inside createExecution (the server is
/// injected by NavigationServerBase; direct action tests use this subclass).
class TestControllerAction : public ayanami::components::ControllerAction {
 public:
  using ControllerAction::ControllerAction;
  void initCheckers(ayanami::abstract::AbstractGoalChecker::Ptr goal,
                    ayanami::abstract::AbstractProgressChecker::Ptr progress) {
    this->goal = std::move(goal);
    this->progress = std::move(progress);
  }

 protected:
  ayanami::components::ControllerExecution::Ptr createExecution() override {
    auto exec = ControllerAction::createExecution();
    exec->setGoalChecker(goal);
    exec->setProgressChecker(progress);
    return exec;
  }

 private:
  ayanami::abstract::AbstractGoalChecker::Ptr goal;
  ayanami::abstract::AbstractProgressChecker::Ptr progress;
};

class FillingSmoother : public ayanami::abstract::AbstractSmoother {
public:
  ResultT<void> smoothPath(Path& path, double) override {
    path.header.frame_id = "map";
    return ResultT<void>::success();
  }
  bool cancel() override { return true; }
};

class TestRecovery : public ayanami::abstract::AbstractRecovery {
public:
  ResultT<void> tick(RecoveryCycleData& cycle) override {
    RecoveryCycleData data;
    data.status = RecoveryStatus::SUCCESS;
    cycle = std::move(data);
    return ResultT<void>::success();
  }
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

// ---- ActionServer protocol (re-creates legacy action_protocol_test, all in the ACTIVE state) ----

TEST(ActionProtocol, completionCarriesResultAndCleansGoal) {
  Server server;
  auto task = server.acceptGoal({});
  ASSERT_TRUE(task.valid());
  EXPECT_EQ(server.activeGoalCount(), 1U);

  ayanami::actions::GetPathResult result;
  result.error = ErrorCode::NONE;
  result.path.poses.resize(2);
  ASSERT_TRUE(server.publishResult(
      task.goal_id(), ayanami::actions::GoalStatus::SUCCEEDED, result));

  ASSERT_TRUE(waitReady(task));
  EXPECT_EQ(task.status(), ayanami::actions::GoalStatus::SUCCEEDED);
  EXPECT_EQ(task.result().error, ErrorCode::NONE);
  EXPECT_FALSE(task.result().path.poses.empty());
  EXPECT_EQ(server.activeGoalCount(), 0U);
  // The completed goal has been cleaned up and cannot finish again
  EXPECT_FALSE(server.publishResult(
      task.goal_id(), ayanami::actions::GoalStatus::SUCCEEDED, result));
}

TEST(ActionProtocol, cancelIsARequestUntilExecutionFinishes) {
  Server server;
  auto task = server.acceptGoal({});
  ASSERT_TRUE(task.cancel());
  EXPECT_TRUE(server.isCancelRequested(task.goal_id()));
  EXPECT_EQ(server.activeGoalCount(), 1U);

  ayanami::actions::GetPathResult result;
  result.error = ErrorCode::NONE;
  ASSERT_TRUE(server.publishResult(
      task.goal_id(), ayanami::actions::GoalStatus::CANCELED, result));
  ASSERT_TRUE(waitReady(task));
  EXPECT_EQ(task.status(), ayanami::actions::GoalStatus::CANCELED);
  EXPECT_EQ(server.activeGoalCount(), 0U);
}

TEST(ActionProtocol, completedGoalsDoNotAccumulate) {
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

TEST(PlannerAction, acceptGoalReturnsTaskHandleAndSucceedsWithPath) {
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
  EXPECT_GE(planner->calls, 1);
}

TEST(PlannerAction, cancelSlotInterruptsRunningExecution) {
  auto planner = std::make_shared<BlockingPlanner>();
  ayanami::components::PlannerAction action(planner);

  auto task = action.acceptGoal({});
  ASSERT_TRUE(task.valid());
  {
    std::unique_lock<std::mutex> lock(planner->mutex);
    ASSERT_TRUE(planner->entered.wait_for(
        lock, std::chrono::milliseconds(1000),
        [&] { return planner->calls.load() >= 1; }));
  }

  action.cancel();
  ASSERT_TRUE(waitReady(task));
  EXPECT_EQ(task.status(), ayanami::actions::GoalStatus::CANCELED);
}

TEST(PlannerAction, taskHandleCancelForwardsToExecution) {
  auto planner = std::make_shared<BlockingPlanner>();
  ayanami::components::PlannerAction action(planner);

  auto task = action.acceptGoal({});
  ASSERT_TRUE(task.valid());
  {
    std::unique_lock<std::mutex> lock(planner->mutex);
    ASSERT_TRUE(planner->entered.wait_for(
        lock, std::chrono::milliseconds(1000),
        [&] { return planner->calls.load() >= 1; }));
  }

  ASSERT_TRUE(task.cancel());
  ASSERT_TRUE(waitReady(task));
  EXPECT_EQ(task.status(), ayanami::actions::GoalStatus::CANCELED);
}

TEST(PlannerAction, preemptSameSlotJoinsOldGoalBeforeNewStarts) {
  auto planner = std::make_shared<BlockingPlanner>();
  ayanami::components::PlannerAction action(planner);

  auto first = action.acceptGoal({});
  ASSERT_TRUE(first.valid());
  {
    std::unique_lock<std::mutex> lock(planner->mutex);
    ASSERT_TRUE(planner->entered.wait_for(
        lock, std::chrono::milliseconds(1000),
        [&] { return planner->calls.load() >= 1; }));
  }

  auto second = action.acceptGoal({});
  ASSERT_TRUE(second.valid());

  // Old goal preempted -> CANCELED; new goal runs to completion -> SUCCEEDED
  ASSERT_TRUE(waitReady(first));
  EXPECT_EQ(first.status(), ayanami::actions::GoalStatus::CANCELED);
  ASSERT_TRUE(waitReady(second));
  EXPECT_EQ(second.status(), ayanami::actions::GoalStatus::SUCCEEDED);
  // Serial: the second starts only after the first is joined (modules never run concurrently)
  EXPECT_EQ(planner->calls.load(), 2);
}

TEST(PlannerAction, resultEmittedOnBypassSlotWhileTaskHandleIsMain) {
  auto planner = std::make_shared<FillingPlanner>();
  ayanami::components::PlannerAction action(planner);

  std::atomic<int> bypass_hits{0};
  ayanami::actions::GoalStatus bypass_status =
      ayanami::actions::GoalStatus::PENDING;
  ayanami::CallbackSubscription cb = ayanami::interfaces::NavigationCallbacks::on_get_path_result.subscribe(
      [&](const ayanami::actions::GoalResult<ayanami::actions::GetPathResult>& gr) {
        bypass_hits++;
        bypass_status = gr.status;
      });

  auto task = action.acceptGoal({});
  ASSERT_TRUE(waitReady(task));
  EXPECT_EQ(task.status(), ayanami::actions::GoalStatus::SUCCEEDED);
  EXPECT_GE(bypass_hits.load(), 1);
  EXPECT_EQ(bypass_status, ayanami::actions::GoalStatus::SUCCEEDED);

  // cb destructor unsubscribes automatically
}

// ---- Minimal acceptance for the remaining 3 actions ----

TEST(ControllerAction, runsOneCycleAndSucceeds) {
  auto& robot_state = ayanami::components::RobotStateController::instance();
  robot_state.initialize("map", "odom", "base_link", 10.0);
  robot_state.setTransform("map", "base_link", ayanami::Timestamp::latest(),
                           Eigen::Isometry3d::Identity());
  auto controller = std::make_shared<TestController>();
  TestControllerAction action(controller);
  action.initCheckers(
      std::make_shared<LambdaGoalChecker>(
          [&controller] { return controller->calls >= 3; }),
      std::make_shared<AlwaysTrueProgressChecker>());

  ayanami::actions::ExePathGoal goal;
  goal.path.poses.resize(4);
  auto task = action.acceptGoal(goal);
  ASSERT_TRUE(waitReady(task));
  EXPECT_EQ(task.status(), ayanami::actions::GoalStatus::SUCCEEDED);
  EXPECT_TRUE(task.result().reached);
  EXPECT_EQ(task.result().error, ErrorCode::NONE);
  EXPECT_GE(controller->calls, 3);
}

TEST(SmootherAction, smoothsPathAndSucceeds) {
  auto smoother = std::make_shared<FillingSmoother>();
  ayanami::components::SmootherAction action(smoother);

  ayanami::actions::SmoothPathGoal goal;
  goal.path.poses.resize(3);
  auto task = action.acceptGoal(goal);
  ASSERT_TRUE(waitReady(task));
  EXPECT_EQ(task.status(), ayanami::actions::GoalStatus::SUCCEEDED);
  EXPECT_EQ(task.result().smoothed_path.poses.size(), 3U);
}

TEST(RecoveryAction, singleRunDeliversResult) {
  auto recovery = std::make_shared<TestRecovery>();
  ayanami::components::RecoveryAction action(recovery);

  ayanami::actions::RecoveryGoal goal;
  goal.recovery_id = "spin";
  auto task = action.acceptGoal(goal);
  ASSERT_TRUE(waitReady(task));
  EXPECT_EQ(task.status(), ayanami::actions::GoalStatus::SUCCEEDED);
  EXPECT_EQ(task.result().error, ErrorCode::NONE);
}
