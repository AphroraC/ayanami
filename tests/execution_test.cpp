#include <gtest/gtest.h>

#include <ayanami/abstract/components/controller_execution.hpp>
#include <ayanami/abstract/components/planner_execution.hpp>
#include <ayanami/abstract/components/recovery_execution.hpp>
#include <ayanami/abstract/components/smoother_execution.hpp>
#include <ayanami/abstract/core/abstract_goal_checker.hpp>
#include <ayanami/abstract/core/abstract_progress_checker.hpp>
#include <ayanami/interfaces/callbacks.hpp>
#include <ayanami/messages/failure.hpp>

#include <Eigen/Geometry>

#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>

namespace {

using ayanami::geometry_msgs::PoseStamped;
using ayanami::FailureCode;

using ayanami::geometry_msgs::TwistStamped;
using ayanami::nav_msgs::Path;

// ---- 测试插件: 记录调用次数, 输出确定结果, 记录 cancel 透传 ----

class TestPlanner : public ayanami::abstract::AbstractPlanner {
public:
  FailureCode makePlan(const PoseStamped&, const PoseStamped&, double,
                       Path&, double&, std::string&) override {
    calls_++;
    return FailureCode::None;
  }
  bool cancel() override {
    cancel_called_ = true;
    return true;
  }
  std::atomic<int> calls_{0};
  std::atomic<bool> cancel_called_{false};
};

class TestController : public ayanami::abstract::AbstractController {
public:
  FailureCode computeVelocityCommands(
      const PoseStamped&, const TwistStamped&, TwistStamped& cmd_vel,
      std::string&,
      const ayanami::abstract::AbstractGoalChecker::Ptr&) override {
    calls_++;
    // 插件只写 linear.x, 其余字段必须由 execution 在发布前 `= {}` 清零保证已初始化
    cmd_vel.twist.linear.x() = 0.5;
    return FailureCode::None;
  }
  bool setPlan(const Path&) override { return true; }
  bool cancel() override {
    cancel_called_ = true;
    return true;
  }
  std::atomic<int> calls_{0};
  std::atomic<bool> cancel_called_{false};
};

// 永不达目标 (goal_checker 永不判到达): 用于观察 cancel → CANCELED
// (而非 STOPPED) 的退出语义
class NeverReachingController : public ayanami::abstract::AbstractController {
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
  bool cancel() override {
    cancel_called_ = true;
    return true;
  }
  std::atomic<int> calls_{0};
  std::atomic<bool> cancel_called_{false};
};

// ---- checker 测试桩 ----

// 目标检查: 自身第 N 次调用后才判到达 (check 先于 compute, N=4 ⇒ 至少 3 帧 compute)。
class ReachAfterNChecksGoalChecker : public ayanami::abstract::AbstractGoalChecker {
public:
  explicit ReachAfterNChecksGoalChecker(int threshold = 4)
      : threshold_(threshold) {}
  void reset() override { calls_ = 0; }
  bool isGoalReached(const PoseStamped&, const PoseStamped&,
                     const TwistStamped&) override {
    return ++calls_ >= threshold_;
  }
  ayanami::abstract::GoalCheckerTolerances getTolerances() const override {
    return {};
  }

private:
  int threshold_;
  int calls_{0};
};

// 永不到达。
class NeverGoalChecker : public ayanami::abstract::AbstractGoalChecker {
public:
  void reset() override {}
  bool isGoalReached(const PoseStamped&, const PoseStamped&,
                     const TwistStamped&) override {
    return false;
  }
  ayanami::abstract::GoalCheckerTolerances getTolerances() const override {
    return {};
  }
};

// 恒有进展。
class TrueProgressChecker : public ayanami::abstract::AbstractProgressChecker {
public:
  void reset() override {}
  bool check(const PoseStamped&) override { return true; }
};

// 恒无进展。
class FalseProgressChecker : public ayanami::abstract::AbstractProgressChecker {
public:
  void reset() override {}
  bool check(const PoseStamped&) override { return false; }
};

class TestSmoother : public ayanami::abstract::AbstractSmoother {
public:
  FailureCode smoothPath(const Path&, Path&, double, bool,
                         std::string&) override {
    return FailureCode::None;
  }
  bool cancel() override {
    cancel_called_ = true;
    return true;
  }
  std::atomic<bool> cancel_called_{false};
};

class TestRecovery : public ayanami::abstract::AbstractRecovery {
public:
  FailureCode runRecovery(std::string&) override { return FailureCode::None; }
  bool cancel() override {
    cancel_called_ = true;
    return true;
  }
  std::atomic<bool> cancel_called_{false};
};

// 阻塞直到被 cancel: 验证恢复执行取消后进入 CANCELED 且 cancel 透传到插件
class BlockingRecovery : public ayanami::abstract::AbstractRecovery {
public:
  FailureCode runRecovery(std::string&) override {
    while (!cancel_requested_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return FailureCode::None;
  }
  bool cancel() override {
    cancel_called_ = true;
    cancel_requested_ = true;
    return true;
  }
  std::atomic<bool> cancel_called_{false};
  std::atomic<bool> cancel_requested_{false};
};

// 轮询直至 terminal state 或超时 (waitForStateUpdate 返回无 timeout 即状态已更新)
template <typename Exec>
bool waitForState(Exec& e, int target, int max_iters = 100) {
  for (int i = 0; i < max_iters; ++i) {
    if (e.getState() == target) return true;
    e.waitForStateUpdate(std::chrono::milliseconds(100));
  }
  return e.getState() == target;
}

} // namespace

TEST(ExecutionPlanner, StartGoalReachesFoundPlanAndOutcomeZero) {
  auto planner = std::make_shared<TestPlanner>();
  ayanami::components::PlannerExecution exec(planner);
  PoseStamped start, goal;
  exec.start(start, goal, 0.1);
  ASSERT_TRUE(waitForState(exec, ayanami::components::PlannerExecution::FOUND_PLAN));
  EXPECT_EQ(exec.getState(), ayanami::components::PlannerExecution::FOUND_PLAN);
  EXPECT_EQ(exec.getFailure(), FailureCode::None);
  EXPECT_GE(planner->calls_, 1);
  exec.join();
}

TEST(ExecutionController, LoopsAtFrequencyReachesGoalAndEmitsCmdVel) {
  auto controller = std::make_shared<TestController>();
  ayanami::components::ControllerExecution exec(controller);
  exec.setControllerFrequency(500.0);
  // 到达判定由 goal_checker 完成 (唯一到达判定, 不再经 controller)。
  exec.setGoalChecker(std::make_shared<ReachAfterNChecksGoalChecker>());
  exec.setProgressChecker(std::make_shared<TrueProgressChecker>());

  bool received_cmd = false;
  int cb_id = ayanami::interfaces::NavigationCallbacks::on_velocity_command.add(
      [&received_cmd](const TwistStamped& cmd) {
        received_cmd = true;
        // Global Constraint: 发布前清零 ⇒ linear.x 等字段已初始化 (非 NaN/Inf)
        EXPECT_TRUE(std::isfinite(cmd.twist.linear.x()));
        EXPECT_TRUE(std::isfinite(cmd.twist.linear.y()));
        EXPECT_TRUE(std::isfinite(cmd.twist.linear.z()));
        EXPECT_TRUE(std::isfinite(cmd.twist.angular.z()));
      });

  Path plan;
  plan.poses.resize(4);
  exec.setNewPlan(plan);
  exec.start();

  ASSERT_TRUE(waitForState(exec, ayanami::components::ControllerExecution::ARRIVED_GOAL));
  EXPECT_EQ(exec.getState(), ayanami::components::ControllerExecution::ARRIVED_GOAL);
  EXPECT_EQ(exec.getFailure(), FailureCode::None);
  EXPECT_GE(controller->calls_, 3);
  EXPECT_TRUE(received_cmd);

  exec.join();
  ayanami::interfaces::NavigationCallbacks::on_velocity_command.remove(cb_id);
}

TEST(ExecutionController, PublishStopCommandEmitsZeroCmdVel) {
  auto controller = std::make_shared<TestController>();
  ayanami::components::ControllerExecution exec(controller);

  bool received = false;
  int cb_id = ayanami::interfaces::NavigationCallbacks::on_velocity_command.add(
      [&received](const TwistStamped& cmd) {
        received = true;
        EXPECT_EQ(cmd.twist.linear.x(), 0.0);
        EXPECT_EQ(cmd.twist.linear.y(), 0.0);
        EXPECT_EQ(cmd.twist.linear.z(), 0.0);
        EXPECT_EQ(cmd.twist.angular.x(), 0.0);
        EXPECT_EQ(cmd.twist.angular.y(), 0.0);
        EXPECT_EQ(cmd.twist.angular.z(), 0.0);
      });

  exec.cancel();  // 触发 publishStopCommand (safety cancel)

  EXPECT_TRUE(received);
  ayanami::interfaces::NavigationCallbacks::on_velocity_command.remove(cb_id);
}

TEST(ExecutionRecovery, RunRecoveryReachesDone) {
  auto recovery = std::make_shared<TestRecovery>();
  ayanami::components::RecoveryExecution exec(recovery);
  exec.start();
  ASSERT_TRUE(waitForState(exec, ayanami::components::RecoveryExecution::RECOVERY_DONE));
  EXPECT_EQ(exec.getState(), ayanami::components::RecoveryExecution::RECOVERY_DONE);
  EXPECT_EQ(exec.getFailure(), FailureCode::None);
  exec.join();
}

TEST(ExecutionSmoother, SmoothPathReachesSmoothed) {
  auto smoother = std::make_shared<TestSmoother>();
  ayanami::components::SmootherExecution exec(smoother);
  Path path;
  exec.smoothPath(path, 2.0, false);
  ASSERT_TRUE(waitForState(exec, ayanami::components::SmootherExecution::SMOOTHED_PATH));
  EXPECT_EQ(exec.getState(), ayanami::components::SmootherExecution::SMOOTHED_PATH);
  EXPECT_EQ(exec.getFailure(), FailureCode::None);
  exec.join();
}

TEST(ExecutionPlanner, CancelForwardsToPlannerPlugin) {
  auto planner = std::make_shared<TestPlanner>();
  ayanami::components::PlannerExecution exec(planner);
  exec.cancel();
  EXPECT_TRUE(planner->cancel_called_);
}

TEST(ExecutionSmoother, CancelForwardsToSmootherPlugin) {
  auto smoother = std::make_shared<TestSmoother>();
  ayanami::components::SmootherExecution exec(smoother);
  exec.cancel();
  EXPECT_TRUE(smoother->cancel_called_);
}

TEST(ExecutionRecovery, CancelForwardsToRecoveryPlugin) {
  auto recovery = std::make_shared<TestRecovery>();
  ayanami::components::RecoveryExecution exec(recovery);
  exec.cancel();
  EXPECT_TRUE(recovery->cancel_called_);
}

TEST(ExecutionRecovery, CancelDuringBlockingRecoveryEndsInCanceledAndForwards) {
  auto recovery = std::make_shared<BlockingRecovery>();
  ayanami::components::RecoveryExecution exec(recovery);
  exec.start();
  // 等 runRecovery 进入阻塞 (cancel_requested_ 仍 false)
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  exec.cancel();
  EXPECT_TRUE(recovery->cancel_called_);
  ASSERT_TRUE(waitForState(exec, ayanami::components::RecoveryExecution::CANCELED));
  EXPECT_EQ(exec.getState(), ayanami::components::RecoveryExecution::CANCELED);
  exec.join();
}

TEST(ExecutionController, CancelEndsInCanceledNotStoppedAndForwardsToPlugin) {
  auto controller = std::make_shared<NeverReachingController>();
  ayanami::components::ControllerExecution exec(controller);
  exec.setControllerFrequency(500.0);
  exec.setGoalChecker(std::make_shared<NeverGoalChecker>());
  exec.setProgressChecker(std::make_shared<TrueProgressChecker>());

  Path plan;
  plan.poses.resize(4);
  exec.setNewPlan(plan);
  exec.start();

  // 等至少一次 computeVelocityCommands 跑过, 再 cancel
  for (int i = 0; i < 100 && controller->calls_ == 0; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  EXPECT_GE(controller->calls_, 1);

  exec.cancel();
  EXPECT_TRUE(controller->cancel_called_);

  // 取消 → CANCELED (而非 stop 的 STOPPED)
  ASSERT_TRUE(waitForState(exec, ayanami::components::ControllerExecution::CANCELED));
  EXPECT_EQ(exec.getState(), ayanami::components::ControllerExecution::CANCELED);
  EXPECT_NE(exec.getState(), ayanami::components::ControllerExecution::STOPPED);
  exec.join();
}

TEST(ExecutionController, NoProgressMapsToNoProgressStateAndFailure) {
  auto controller = std::make_shared<NeverReachingController>();
  ayanami::components::ControllerExecution exec(controller);
  exec.setControllerFrequency(500.0);
  exec.setGoalChecker(std::make_shared<NeverGoalChecker>());
  exec.setProgressChecker(std::make_shared<FalseProgressChecker>());

  Path plan;
  plan.poses.resize(4);
  exec.setNewPlan(plan);
  exec.start();

  ASSERT_TRUE(waitForState(exec, ayanami::components::ControllerExecution::NO_PROGRESS));
  EXPECT_EQ(exec.getState(), ayanami::components::ControllerExecution::NO_PROGRESS);
  EXPECT_EQ(exec.getFailure(), FailureCode::NoProgress);
  EXPECT_EQ(controller->calls_, 0);  // 无进展即终止, controller 不被调用
  exec.join();
}

TEST(ExecutionController, MissingCheckersIsInternalError) {
  auto controller = std::make_shared<NeverReachingController>();
  ayanami::components::ControllerExecution exec(controller);
  exec.setControllerFrequency(500.0);

  Path plan;
  plan.poses.resize(4);
  exec.setNewPlan(plan);
  exec.start();

  ASSERT_TRUE(waitForState(exec, ayanami::components::ControllerExecution::INTERNAL_ERROR));
  EXPECT_EQ(exec.getState(), ayanami::components::ControllerExecution::INTERNAL_ERROR);
  EXPECT_EQ(exec.getFailure(), FailureCode::InternalError);
  EXPECT_EQ(controller->calls_, 0);
  exec.join();
}
