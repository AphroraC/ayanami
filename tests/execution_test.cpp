#include <gtest/gtest.h>

#include <ayanami/components/controller_execution.hpp>
#include <ayanami/components/planner_execution.hpp>
#include <ayanami/components/recovery_execution.hpp>
#include <ayanami/components/smoother_execution.hpp>
#include <ayanami/interfaces/callbacks.hpp>

#include <Eigen/Geometry>

#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

namespace {

using ayanami::geometry_msgs::PoseStamped;
using ayanami::geometry_msgs::TwistStamped;
using ayanami::nav_msgs::Path;

// ---- 测试插件: 记录调用次数, 输出确定结果 ----

class TestPlanner : public ayanami::abstract::AbstractPlanner {
public:
  uint32_t makePlan(const PoseStamped&, const PoseStamped&, double,
                    Path&, double&, std::string&) override {
    calls_++;
    return 0;
  }
  std::atomic<int> calls_{0};
};

class TestController : public ayanami::abstract::AbstractController {
public:
  uint32_t computeVelocityCommands(const PoseStamped&, const TwistStamped&,
                                   TwistStamped& cmd_vel, std::string&) override {
    calls_++;
    // 插件只写 linear.x, 其余字段必须由 execution 在发布前 `= {}` 清零保证已初始化
    cmd_vel.twist.linear.x() = 0.5;
    return 0;
  }
  bool isGoalReached(double, double) override { return calls_ >= 3; }
  bool setPlan(const Path&) override { return true; }
  std::atomic<int> calls_{0};
};

class TestSmoother : public ayanami::abstract::AbstractSmoother {
public:
  uint32_t smoothPath(const Path&, Path&, double, bool, std::string&) override {
    return 0;
  }
};

class TestRecovery : public ayanami::abstract::AbstractRecovery {
public:
  uint32_t runRecovery(std::string&) override { return 0; }
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
  EXPECT_EQ(exec.getOutcome(), 0u);
  EXPECT_GE(planner->calls_, 1);
  exec.join();
}

TEST(ExecutionController, LoopsAtFrequencyReachesGoalAndEmitsCmdVel) {
  auto controller = std::make_shared<TestController>();
  ayanami::components::ControllerExecution exec(controller);
  exec.setControllerFrequency(500.0);

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
  EXPECT_EQ(exec.getOutcome(), 0u);
  exec.join();
}

TEST(ExecutionSmoother, SmoothPathReachesSmoothed) {
  auto smoother = std::make_shared<TestSmoother>();
  ayanami::components::SmootherExecution exec(smoother);
  Path path;
  exec.smoothPath(path, 2.0, false);
  ASSERT_TRUE(waitForState(exec, ayanami::components::SmootherExecution::SMOOTHED_PATH));
  EXPECT_EQ(exec.getState(), ayanami::components::SmootherExecution::SMOOTHED_PATH);
  EXPECT_EQ(exec.getOutcome(), 0u);
  exec.join();
}
