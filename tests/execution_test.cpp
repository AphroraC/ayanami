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
    calls++;
    return FailureCode::None;
  }
  bool cancel() override {
    cancel_called = true;
    return true;
  }
  std::atomic<int> calls{0};
  std::atomic<bool> cancel_called{false};
};

class TestController : public ayanami::abstract::AbstractController {
public:
  FailureCode computeVelocityCommands(
      const PoseStamped&, const TwistStamped&, TwistStamped& cmd_vel,
      std::string&) override {
    calls++;
    // 插件只写 linear.x, 其余字段必须由 execution 在发布前 `= {}` 清零保证已初始化
    cmd_vel.twist.linear.x() = 0.5;
    return FailureCode::None;
  }
  bool setPlan(const Path&) override { return true; }
  bool cancel() override {
    cancel_called = true;
    return true;
  }
  std::atomic<int> calls{0};
  std::atomic<bool> cancel_called{false};
};

// 永不达目标 (goal_checker 永不判到达): 用于观察 cancel → CANCELED
// (而非 STOPPED) 的退出语义
class NeverReachingController : public ayanami::abstract::AbstractController {
public:
  FailureCode computeVelocityCommands(
      const PoseStamped&, const TwistStamped&, TwistStamped& cmd_vel,
      std::string&) override {
    calls++;
    cmd_vel.twist.linear.x() = 0.5;
    return FailureCode::None;
  }
  bool setPlan(const Path&) override { return true; }
  bool cancel() override {
    cancel_called = true;
    return true;
  }
  std::atomic<int> calls{0};
  std::atomic<bool> cancel_called{false};
};

// ---- checker 测试桩 ----

// 目标检查: 自身第 N 次调用后才判到达 (check 先于 compute, N=4 ⇒ 至少 3 帧 compute)。
class ReachAfterNChecksGoalChecker : public ayanami::abstract::AbstractGoalChecker {
public:
  explicit ReachAfterNChecksGoalChecker(int threshold = 4)
      : threshold(threshold) {}
  void reset() override { calls = 0; }
  bool isGoalReached(const PoseStamped&, const PoseStamped&,
                     const TwistStamped&) override {
    return ++calls >= threshold;
  }
  ayanami::abstract::GoalCheckerTolerances getTolerances() const override {
    return {};
  }

private:
  int threshold;
  int calls{0};
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
    cancel_called = true;
    return true;
  }
  std::atomic<bool> cancel_called{false};
};

class TestRecovery : public ayanami::abstract::AbstractRecovery {
public:
  FailureCode runRecovery(std::string&) override { return FailureCode::None; }
  bool cancel() override {
    cancel_called = true;
    return true;
  }
  std::atomic<bool> cancel_called{false};
};

// 阻塞直到被 cancel: 验证恢复执行取消后进入 CANCELED 且 cancel 透传到插件
class BlockingRecovery : public ayanami::abstract::AbstractRecovery {
public:
  FailureCode runRecovery(std::string&) override {
    while (!cancel_requested) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return FailureCode::None;
  }
  bool cancel() override {
    cancel_called = true;
    cancel_requested = true;
    return true;
  }
  std::atomic<bool> cancel_called{false};
  std::atomic<bool> cancel_requested{false};
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

TEST(ExecutionPlanner, startGoalReachesFoundPlanAndOutcomeZero) {
  auto planner = std::make_shared<TestPlanner>();
  ayanami::components::PlannerExecution exec(planner);
  PoseStamped start, goal;
  exec.start(start, goal, 0.1);
  ASSERT_TRUE(waitForState(exec, ayanami::components::PlannerExecution::FOUND_PLAN));
  EXPECT_EQ(exec.getState(), ayanami::components::PlannerExecution::FOUND_PLAN);
  EXPECT_EQ(exec.getFailure(), FailureCode::None);
  EXPECT_GE(planner->calls, 1);
  exec.join();
}

TEST(ExecutionController, loopsAtFrequencyReachesGoalAndEmitsCmdVel) {
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
  EXPECT_GE(controller->calls, 3);
  EXPECT_TRUE(received_cmd);

  exec.join();
  ayanami::interfaces::NavigationCallbacks::on_velocity_command.remove(cb_id);
}

TEST(ExecutionController, publishStopCommandEmitsZeroCmdVel) {
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

TEST(ExecutionRecovery, runRecoveryReachesDone) {
  auto recovery = std::make_shared<TestRecovery>();
  ayanami::components::RecoveryExecution exec(recovery);
  exec.start();
  ASSERT_TRUE(waitForState(exec, ayanami::components::RecoveryExecution::RECOVERY_DONE));
  EXPECT_EQ(exec.getState(), ayanami::components::RecoveryExecution::RECOVERY_DONE);
  EXPECT_EQ(exec.getFailure(), FailureCode::None);
  exec.join();
}

TEST(ExecutionSmoother, smoothPathReachesSmoothed) {
  auto smoother = std::make_shared<TestSmoother>();
  ayanami::components::SmootherExecution exec(smoother);
  Path path;
  exec.smoothPath(path, 2.0, false);
  ASSERT_TRUE(waitForState(exec, ayanami::components::SmootherExecution::SMOOTHED_PATH));
  EXPECT_EQ(exec.getState(), ayanami::components::SmootherExecution::SMOOTHED_PATH);
  EXPECT_EQ(exec.getFailure(), FailureCode::None);
  exec.join();
}

TEST(ExecutionPlanner, cancelForwardsToPlannerPlugin) {
  auto planner = std::make_shared<TestPlanner>();
  ayanami::components::PlannerExecution exec(planner);
  exec.cancel();
  EXPECT_TRUE(planner->cancel_called);
}

TEST(ExecutionSmoother, cancelForwardsToSmootherPlugin) {
  auto smoother = std::make_shared<TestSmoother>();
  ayanami::components::SmootherExecution exec(smoother);
  exec.cancel();
  EXPECT_TRUE(smoother->cancel_called);
}

TEST(ExecutionRecovery, cancelForwardsToRecoveryPlugin) {
  auto recovery = std::make_shared<TestRecovery>();
  ayanami::components::RecoveryExecution exec(recovery);
  exec.cancel();
  EXPECT_TRUE(recovery->cancel_called);
}

TEST(ExecutionRecovery, cancelDuringBlockingRecoveryEndsInCanceledAndForwards) {
  auto recovery = std::make_shared<BlockingRecovery>();
  ayanami::components::RecoveryExecution exec(recovery);
  exec.start();
  // 等 runRecovery 进入阻塞 (cancel_requested 仍 false)
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  exec.cancel();
  EXPECT_TRUE(recovery->cancel_called);
  ASSERT_TRUE(waitForState(exec, ayanami::components::RecoveryExecution::CANCELED));
  EXPECT_EQ(exec.getState(), ayanami::components::RecoveryExecution::CANCELED);
  exec.join();
}

TEST(ExecutionController, cancelEndsInCanceledNotStoppedAndForwardsToPlugin) {
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
  for (int i = 0; i < 100 && controller->calls == 0; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  EXPECT_GE(controller->calls, 1);

  exec.cancel();
  EXPECT_TRUE(controller->cancel_called);

  // 取消 → CANCELED (而非 stop 的 STOPPED)
  ASSERT_TRUE(waitForState(exec, ayanami::components::ControllerExecution::CANCELED));
  EXPECT_EQ(exec.getState(), ayanami::components::ControllerExecution::CANCELED);
  EXPECT_NE(exec.getState(), ayanami::components::ControllerExecution::STOPPED);
  exec.join();
}

TEST(ExecutionController, noProgressMapsToNoProgressStateAndFailure) {
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
  EXPECT_EQ(controller->calls, 0);  // 无进展即终止, controller 不被调用
  exec.join();
}

TEST(ExecutionController, missingCheckersIsInternalError) {
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
  EXPECT_EQ(controller->calls, 0);
  exec.join();
}

// 控制器输出超过 setMaximumVelocity 上限 → validVelocityCommand 拒绝 →
// NO_LOCAL_CMD (不发非零超限速度); max_vel_y=0 额外禁止横移 (|linear.y| 必须为 0)。
TEST(ExecutionController, velocityAboveMaxRejectedAndNotPublished) {
  class OverspeedController : public ayanami::abstract::AbstractController {
   public:
    FailureCode computeVelocityCommands(
        const PoseStamped&, const TwistStamped&, TwistStamped& cmd_vel,
        std::string&) override {
      calls++;
      cmd_vel.twist.linear.x() = 5.0;
      cmd_vel.twist.linear.y() = 2.0;
      cmd_vel.twist.angular.z() = 3.0;
      return FailureCode::None;
    }
    bool setPlan(const Path&) override { return true; }
    bool cancel() override { return true; }
    std::atomic<int> calls{0};
  };
  auto controller = std::make_shared<OverspeedController>();
  ayanami::components::ControllerExecution exec(controller);
  exec.setControllerFrequency(500.0);
  exec.setGoalChecker(std::make_shared<NeverGoalChecker>());
  exec.setProgressChecker(std::make_shared<TrueProgressChecker>());
  // 速度上限: x<=1.0, y==0 (禁止横移), theta<=1.0。
  exec.setMaximumVelocity(1.0, 0.0, 1.0);

  std::atomic<int> overspeed_cmds{0};
  int cb_id = ayanami::interfaces::NavigationCallbacks::on_velocity_command.add(
      [&](const TwistStamped& cmd) {
        if (cmd.twist.linear.x() > 1.0 ||
            std::abs(cmd.twist.linear.y()) > 1e-9 ||
            std::abs(cmd.twist.angular.z()) > 1.0) {
          overspeed_cmds.fetch_add(1);
        }
      });

  Path plan;
  plan.poses.resize(4);
  exec.setNewPlan(plan);
  exec.start();

  // 每周期都命中超限 → NO_LOCAL_CMD (非终态, 循环继续), 超限速度从未发布。
  for (int i = 0; i < 100 &&
                  exec.getState() !=
                      ayanami::components::ControllerExecution::NO_LOCAL_CMD;
       ++i) {
    exec.waitForStateUpdate(std::chrono::milliseconds(10));
  }
  EXPECT_EQ(exec.getState(),
            ayanami::components::ControllerExecution::NO_LOCAL_CMD);
  EXPECT_EQ(exec.getFailure(), FailureCode::NoValidCommand);
  EXPECT_GE(controller->calls, 1);
  EXPECT_EQ(overspeed_cmds.load(), 0);

  exec.cancel();
  exec.join();
  ayanami::interfaces::NavigationCallbacks::on_velocity_command.remove(cb_id);
}

// setMaximumVelocity 上限宽松时超限被放行 (对照: 拒绝路径真因上限而非其他)。
TEST(ExecutionController, velocityWithinMaxIsPublished) {
  class FastController : public ayanami::abstract::AbstractController {
   public:
    FailureCode computeVelocityCommands(
        const PoseStamped&, const TwistStamped&, TwistStamped& cmd_vel,
        std::string&) override {
      cmd_vel.twist.linear.x() = 0.4;
      cmd_vel.twist.angular.z() = 0.5;
      return FailureCode::None;
    }
    bool setPlan(const Path&) override { return true; }
    bool cancel() override { return true; }
  };
  auto controller = std::make_shared<FastController>();
  ayanami::components::ControllerExecution exec(controller);
  exec.setControllerFrequency(500.0);
  exec.setGoalChecker(std::make_shared<ReachAfterNChecksGoalChecker>(2));
  exec.setProgressChecker(std::make_shared<TrueProgressChecker>());
  exec.setMaximumVelocity(1.0, 0.0, 1.0);

  bool received = false;
  int cb_id = ayanami::interfaces::NavigationCallbacks::on_velocity_command.add(
      [&](const TwistStamped&) { received = true; });

  Path plan;
  plan.poses.resize(4);
  exec.setNewPlan(plan);
  exec.start();

  ASSERT_TRUE(waitForState(exec, ayanami::components::ControllerExecution::ARRIVED_GOAL));
  EXPECT_EQ(exec.getFailure(), FailureCode::None);
  EXPECT_TRUE(received);

  exec.join();
  ayanami::interfaces::NavigationCallbacks::on_velocity_command.remove(cb_id);
}
