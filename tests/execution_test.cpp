#include <gtest/gtest.h>

#include "test_transform_source.hpp"

#include <ayanami/abstract/components/controller_execution.hpp>
#include <ayanami/abstract/components/planner_execution.hpp>
#include <ayanami/abstract/components/recovery_execution.hpp>
#include <ayanami/abstract/components/smoother_execution.hpp>
#include <ayanami/abstract/core/abstract_goal_checker.hpp>
#include <ayanami/abstract/core/abstract_progress_checker.hpp>
#include <ayanami/interfaces/callbacks.hpp>
#include <ayanami/abstract/components/robot_state_controller.hpp>
#include <ayanami/messages/failure.hpp>
#include <ayanami/types.hpp>

#include <Eigen/Geometry>

#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>

using ayanami::testing::testTransforms;
namespace {

using ayanami::geometry_msgs::PoseStamped;

using ayanami::geometry_msgs::TwistStamped;
using ayanami::nav_msgs::Path;
using ayanami::ErrorCode;
using ayanami::ResultT;
using ayanami::RecoveryCycleData;
using ayanami::RecoveryStatus;

// ---- Test modules: record call counts, produce deterministic results, record cancel pass-through ----

class TestPlanner : public ayanami::abstract::AbstractPlanner {
public:
  ResultT<void> makePlan(const PoseStamped&, const PoseStamped&,
                          double, ayanami::nav_msgs::Path& plan_out) override {
    calls++;
    plan_out = Path{};
    return ResultT<void>::success();
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
  ResultT<void> computeVelocityCommand(const PoseStamped&,
                                           const TwistStamped&, ayanami::geometry_msgs::TwistStamped& cmd) override {
    calls++;
    // The module only writes linear.x; the remaining fields must be guaranteed initialized by the execution via its pre-publish = {} zeroing
    cmd = TwistStamped{};
    cmd.twist.linear.x() = 0.5;
    return ResultT<void>::success();
  }
  bool setPlan(const Path&) override { return true; }
  bool cancel() override {
    cancel_called = true;
    return true;
  }
  std::atomic<int> calls{0};
  std::atomic<bool> cancel_called{false};
};

/// Never reaches the goal (the goal_checker never declares arrival): used to observe the cancel -> CANCELED
/// (rather than STOPPED) exit semantics.
class NeverReachingController : public ayanami::abstract::AbstractController {
public:
  ResultT<void> computeVelocityCommand(const PoseStamped&,
                                           const TwistStamped&, ayanami::geometry_msgs::TwistStamped& cmd) override {
    calls++;
    cmd = TwistStamped{};
    cmd.twist.linear.x() = 0.5;
    return ResultT<void>::success();
  }
  bool setPlan(const Path&) override { return true; }
  bool cancel() override {
    cancel_called = true;
    return true;
  }
  std::atomic<int> calls{0};
  std::atomic<bool> cancel_called{false};
};

// ---- checker test stubs ----

/// Goal check: declares arrival only on its own Nth call (check runs before compute, N=4 => at least 3 compute frames).
class ReachAfterNChecksGoalChecker : public ayanami::abstract::AbstractGoalChecker {
public:
  explicit ReachAfterNChecksGoalChecker(int threshold = 4)
      : threshold(threshold) {}
  void reset() override { calls = 0; }
  bool isGoalReached(const PoseStamped&, const PoseStamped&,
                     const TwistStamped&) override {
    return ++calls >= threshold;
  }
  ayanami::GoalCheckerTolerances getTolerances() const override {
    return {};
  }

private:
  int threshold;
  int calls{0};
};

/// Never reached.
class NeverGoalChecker : public ayanami::abstract::AbstractGoalChecker {
public:
  void reset() override {}
  bool isGoalReached(const PoseStamped&, const PoseStamped&,
                     const TwistStamped&) override {
    return false;
  }
  ayanami::GoalCheckerTolerances getTolerances() const override {
    return {};
  }
};

/// Always progressing.
class TrueProgressChecker : public ayanami::abstract::AbstractProgressChecker {
public:
  void reset() override {}
  bool check(const PoseStamped&) override { return true; }
};

/// Never progressing.
class FalseProgressChecker : public ayanami::abstract::AbstractProgressChecker {
public:
  void reset() override {}
  bool check(const PoseStamped&) override { return false; }
};

class TestSmoother : public ayanami::abstract::AbstractSmoother {
public:
  ResultT<void> smoothPath(Path&, double) override {
    return ResultT<void>::success();
  }
  bool cancel() override {
    cancel_called = true;
    return true;
  }
  std::atomic<bool> cancel_called{false};
};

class TestRecovery : public ayanami::abstract::AbstractRecovery {
public:
  ResultT<void> tick(RecoveryCycleData& cycle) override {
    RecoveryCycleData data;
    data.status = RecoveryStatus::SUCCESS;
    cycle = std::move(data);
    return ResultT<void>::success();
  }
  bool cancel() override {
    cancel_called = true;
    return true;
  }
  std::atomic<bool> cancel_called{false};
};

/// Returns RUNNING on every cycle while running, until cancelled: verifies the recovery execution enters CANCELED
/// after cancel and that cancel passes through to the module (a tick-state-machine equivalent of the old
/// BlockingRecovery::runRecovery blocking loop: blocking yields to execution cycle driving, semantics unchanged).
class BlockingRecovery : public ayanami::abstract::AbstractRecovery {
public:
  ResultT<void> tick(RecoveryCycleData& cycle) override {
    if (cancel_requested) {
      return ResultT<void>::failure(ErrorCode::CANCELED, "canceled");
    }
    RecoveryCycleData data;  // RUNNING by default
    cycle = std::move(data);
    return ResultT<void>::success();
  }
  bool cancel() override {
    cancel_called = true;
    cancel_requested = true;
    return true;
  }
  std::atomic<bool> cancel_called{false};
  std::atomic<bool> cancel_requested{false};
};

/// Polls until a terminal state or timeout (waitForStateUpdate returning without timeout means the state was updated)
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
  exec.prepareGoal(start, goal, 0.1);
  std::thread exec_th([&] { exec.runBlocking(); });
  ASSERT_TRUE(waitForState(exec, ayanami::components::PlannerExecution::FOUND_PLAN));
  EXPECT_EQ(exec.getState(), ayanami::components::PlannerExecution::FOUND_PLAN);
  EXPECT_EQ(exec.getError(), ErrorCode::NONE);
  EXPECT_GE(planner->calls, 1);
  exec_th.join();
}

TEST(ExecutionController, loopsAtFrequencyReachesGoalAndEmitsCmdVel) {
  auto& robot_state = ayanami::components::RobotStateController::instance();
  robot_state.initialize("map", "odom", "base_link", 10.0);
  testTransforms().set("map", "base_link", ayanami::Timestamp::latest(),
                           Eigen::Isometry3d::Identity());
  auto controller = std::make_shared<TestController>();
  ayanami::components::ControllerExecution exec(controller);
  exec.setControllerFrequency(500.0);
  // Arrival is decided by the goal_checker (the only arrival decision, no longer via the controller).
  exec.setGoalChecker(std::make_shared<ReachAfterNChecksGoalChecker>());
  exec.setProgressChecker(std::make_shared<TrueProgressChecker>());

  bool received_cmd = false;
  ayanami::CallbackSubscription cmd_cb =
      ayanami::interfaces::NavigationCallbacks::on_velocity_command.subscribe(
      [&received_cmd](const TwistStamped& cmd) {
        received_cmd = true;
        // Global constraint: zeroed before publishing => linear.x and other fields are initialized (non-NaN/Inf)
        EXPECT_TRUE(std::isfinite(cmd.twist.linear.x()));
        EXPECT_TRUE(std::isfinite(cmd.twist.linear.y()));
        EXPECT_TRUE(std::isfinite(cmd.twist.linear.z()));
        EXPECT_TRUE(std::isfinite(cmd.twist.angular.z()));
      });

  Path plan;
  plan.poses.resize(4);
  exec.setNewPlan(plan);
  std::thread exec_th([&] { exec.runBlocking(); });

  ASSERT_TRUE(waitForState(exec, ayanami::components::ControllerExecution::ARRIVED_GOAL));
  EXPECT_EQ(exec.getState(), ayanami::components::ControllerExecution::ARRIVED_GOAL);
  EXPECT_EQ(exec.getError(), ErrorCode::NONE);
  EXPECT_GE(controller->calls, 3);
  EXPECT_TRUE(received_cmd);

  exec_th.join();
}  // cmd_cb destructor unsubscribes automatically

TEST(ExecutionController, publishStopCommandEmitsZeroCmdVel) {
  auto& robot_state = ayanami::components::RobotStateController::instance();
  robot_state.initialize("map", "odom", "base_link", 10.0);
  testTransforms().set("map", "base_link", ayanami::Timestamp::latest(),
                           Eigen::Isometry3d::Identity());
  auto controller = std::make_shared<TestController>();
  ayanami::components::ControllerExecution exec(controller);

  bool received = false;
  ayanami::CallbackSubscription cb = ayanami::interfaces::NavigationCallbacks::on_velocity_command.subscribe(
      [&received](const TwistStamped& cmd) {
        received = true;
        EXPECT_EQ(cmd.twist.linear.x(), 0.0);
        EXPECT_EQ(cmd.twist.linear.y(), 0.0);
        EXPECT_EQ(cmd.twist.linear.z(), 0.0);
        EXPECT_EQ(cmd.twist.angular.x(), 0.0);
        EXPECT_EQ(cmd.twist.angular.y(), 0.0);
        EXPECT_EQ(cmd.twist.angular.z(), 0.0);
      });

  exec.cancel();  // Triggers publishStopCommand (safety cancel)

  EXPECT_TRUE(received);
  // cb destructor unsubscribes automatically
}

TEST(ExecutionRecovery, runRecoveryReachesDone) {
  auto recovery = std::make_shared<TestRecovery>();
  ayanami::components::RecoveryExecution exec(recovery);
  std::thread exec_th([&] { exec.runBlocking(); });
  ASSERT_TRUE(waitForState(exec, ayanami::components::RecoveryExecution::RECOVERY_DONE));
  EXPECT_EQ(exec.getState(), ayanami::components::RecoveryExecution::RECOVERY_DONE);
  EXPECT_EQ(exec.getError(), ErrorCode::NONE);
  exec_th.join();
}

TEST(ExecutionSmoother, smoothPathReachesSmoothed) {
  auto smoother = std::make_shared<TestSmoother>();
  ayanami::components::SmootherExecution exec(smoother);
  Path path;
  std::thread exec_th([&] { exec.smoothPath(path, 2.0); });
  ASSERT_TRUE(waitForState(exec, ayanami::components::SmootherExecution::SMOOTHED_PATH));
  EXPECT_EQ(exec.getState(), ayanami::components::SmootherExecution::SMOOTHED_PATH);
  EXPECT_EQ(exec.getError(), ErrorCode::NONE);
  exec_th.join();
}

TEST(ExecutionPlanner, cancelForwardsToPlannerModule) {
  auto planner = std::make_shared<TestPlanner>();
  ayanami::components::PlannerExecution exec(planner);
  exec.cancel();
  EXPECT_TRUE(planner->cancel_called);
}

TEST(ExecutionSmoother, cancelForwardsToSmootherModule) {
  auto smoother = std::make_shared<TestSmoother>();
  ayanami::components::SmootherExecution exec(smoother);
  exec.cancel();
  EXPECT_TRUE(smoother->cancel_called);
}

TEST(ExecutionRecovery, cancelForwardsToRecoveryModule) {
  auto recovery = std::make_shared<TestRecovery>();
  ayanami::components::RecoveryExecution exec(recovery);
  exec.cancel();
  EXPECT_TRUE(recovery->cancel_called);
}

TEST(ExecutionRecovery, cancelDuringBlockingRecoveryEndsInCanceledAndForwards) {
  auto recovery = std::make_shared<BlockingRecovery>();
  ayanami::components::RecoveryExecution exec(recovery);
  std::thread exec_th([&] { exec.runBlocking(); });
  // Wait for the recovery to enter its cycle loop (tick returns RUNNING, cancel_requested still false)
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  exec.cancel();
  EXPECT_TRUE(recovery->cancel_called);
  ASSERT_TRUE(waitForState(exec, ayanami::components::RecoveryExecution::CANCELED));
  EXPECT_EQ(exec.getState(), ayanami::components::RecoveryExecution::CANCELED);
  exec_th.join();
}

TEST(ExecutionController, cancelEndsInCanceledNotStoppedAndForwardsToModule) {
  auto controller = std::make_shared<NeverReachingController>();
  ayanami::components::ControllerExecution exec(controller);
  exec.setControllerFrequency(500.0);
  exec.setGoalChecker(std::make_shared<NeverGoalChecker>());
  exec.setProgressChecker(std::make_shared<TrueProgressChecker>());

  Path plan;
  plan.poses.resize(4);
  exec.setNewPlan(plan);
  std::thread exec_th([&] { exec.runBlocking(); });

  // Wait for at least one computeVelocityCommand to run, then cancel
  for (int i = 0; i < 100 && controller->calls == 0; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  EXPECT_GE(controller->calls, 1);

  exec.cancel();
  EXPECT_TRUE(controller->cancel_called);

  // Cancel -> CANCELED (rather than the STOPPED of a stop)
  ASSERT_TRUE(waitForState(exec, ayanami::components::ControllerExecution::CANCELED));
  EXPECT_EQ(exec.getState(), ayanami::components::ControllerExecution::CANCELED);
  EXPECT_NE(exec.getState(), ayanami::components::ControllerExecution::STOPPED);
  exec_th.join();
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
  std::thread exec_th([&] { exec.runBlocking(); });

  ASSERT_TRUE(waitForState(exec, ayanami::components::ControllerExecution::NO_PROGRESS));
  EXPECT_EQ(exec.getState(), ayanami::components::ControllerExecution::NO_PROGRESS);
  EXPECT_EQ(exec.getError(), ErrorCode::NO_PROGRESS);
  EXPECT_EQ(controller->calls, 0);  // Terminates on no progress; the controller is never called
  exec_th.join();
}

TEST(ExecutionController, missingCheckersIsInternalError) {
  auto controller = std::make_shared<NeverReachingController>();
  ayanami::components::ControllerExecution exec(controller);
  exec.setControllerFrequency(500.0);

  Path plan;
  plan.poses.resize(4);
  exec.setNewPlan(plan);
  std::thread exec_th([&] { exec.runBlocking(); });

  ASSERT_TRUE(waitForState(exec, ayanami::components::ControllerExecution::INTERNAL_ERROR));
  EXPECT_EQ(exec.getState(), ayanami::components::ControllerExecution::INTERNAL_ERROR);
  EXPECT_EQ(exec.getError(), ErrorCode::INTERNAL_ERROR);
  EXPECT_EQ(controller->calls, 0);
  exec_th.join();
}

// Controller output above the setMaximumVelocity cap -> clamped into the caps and
// published (policy: over-limit commands are clamped + warned, never stopped on);
// max_vel_y=0 additionally forbids lateral motion (|linear.y| must be 0 after clamp).
TEST(ExecutionController, velocityAboveMaxClampedAndPublished) {
  class OverspeedController : public ayanami::abstract::AbstractController {
   public:
    ResultT<void> computeVelocityCommand(const PoseStamped&,
                                             const TwistStamped&, ayanami::geometry_msgs::TwistStamped& cmd) override {
      calls++;
      cmd = TwistStamped{};
      cmd.twist.linear.x() = 5.0;
      cmd.twist.linear.y() = 2.0;
      cmd.twist.angular.z() = 3.0;
      return ResultT<void>::success();
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
  // Velocity caps: x<=1.0, y==0 (lateral motion forbidden), theta<=1.0.
  exec.setMaximumVelocity(1.0, 0.0, 1.0);

  std::atomic<int> overspeed_cmds{0};
  std::atomic<int> published{0};
  TwistStamped last_cmd;
  ayanami::CallbackSubscription cb = ayanami::interfaces::NavigationCallbacks::on_velocity_command.subscribe(
      [&](const TwistStamped& cmd) {
        last_cmd = cmd;
        published.fetch_add(1);
        if (cmd.twist.linear.x() > 1.0 ||
            std::abs(cmd.twist.linear.y()) > 1e-9 ||
            std::abs(cmd.twist.angular.z()) > 1.0) {
          overspeed_cmds.fetch_add(1);
        }
      });

  Path plan;
  plan.poses.resize(4);
  exec.setNewPlan(plan);
  std::thread exec_th([&] { exec.runBlocking(); });

  // Every cycle publishes a clamped command -> GOT_LOCAL_CMD (loop keeps running);
  // the over-limit velocity is clamped, never published above the caps.
  for (int i = 0; i < 100 &&
                  exec.getState() !=
                      ayanami::components::ControllerExecution::GOT_LOCAL_CMD;
       ++i) {
    exec.waitForStateUpdate(std::chrono::milliseconds(10));
  }
  EXPECT_EQ(exec.getState(),
            ayanami::components::ControllerExecution::GOT_LOCAL_CMD);
  EXPECT_EQ(exec.getError(), ErrorCode::NONE);
  EXPECT_GE(controller->calls, 1);
  for (int i = 0; i < 100 && published.load() == 0; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_GE(published.load(), 1);
  EXPECT_EQ(overspeed_cmds.load(), 0);
  EXPECT_NEAR(last_cmd.twist.linear.x(), 1.0, 1e-9);
  EXPECT_NEAR(last_cmd.twist.linear.y(), 0.0, 1e-9);
  EXPECT_NEAR(last_cmd.twist.angular.z(), 1.0, 1e-9);

  exec.cancel();
  exec_th.join();
}  // cmd_cb destructor unsubscribes automatically

// With a loose setMaximumVelocity cap the over-limit value is admitted (control: the rejection was truly caused by the cap, nothing else).
TEST(ExecutionController, velocityWithinMaxIsPublished) {
  class FastController : public ayanami::abstract::AbstractController {
   public:
    ResultT<void> computeVelocityCommand(const PoseStamped&,
                                             const TwistStamped&, ayanami::geometry_msgs::TwistStamped& cmd) override {
      cmd = TwistStamped{};
      cmd.twist.linear.x() = 0.4;
      cmd.twist.angular.z() = 0.5;
      return ResultT<void>::success();
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
  ayanami::CallbackSubscription cb = ayanami::interfaces::NavigationCallbacks::on_velocity_command.subscribe(
      [&](const TwistStamped&) { received = true; });

  Path plan;
  plan.poses.resize(4);
  exec.setNewPlan(plan);
  std::thread exec_th([&] { exec.runBlocking(); });

  ASSERT_TRUE(waitForState(exec, ayanami::components::ControllerExecution::ARRIVED_GOAL));
  EXPECT_EQ(exec.getError(), ErrorCode::NONE);
  EXPECT_TRUE(received);

  exec_th.join();
}  // cmd_cb destructor unsubscribes automatically

TEST(ExecutionController, updatePlanDedupsContentEquivalentPath) {
  // Content-signature dedup (task: version-advance responsibility in the BT
  // layer + framework-level dedup): a version bump whose path content is
  // equivalent (a converged smoother republishing its own output) advances
  // only the version bookkeeping -- the controller setPlan consumption flag
  // stays clear, so critic resets never run for a no-op update.
  auto controller = std::make_shared<TestController>();
  ayanami::components::ControllerExecution exec(controller);
  exec.setGoalChecker(std::make_shared<NeverGoalChecker>());
  exec.setProgressChecker(std::make_shared<TrueProgressChecker>());

  Path plan_a;
  plan_a.poses.resize(3);
  Path plan_a_equiv = plan_a;
  plan_a_equiv.poses[1].pose.position.x() = 5e-4;  // 0.5 mm: equivalent
  Path plan_b = plan_a;
  plan_b.poses[1].pose.position.x() = 0.5;         // 0.5 m: different

  // Initial plan lands (the run loop would consume it; updatePlan itself
  // must not bypass a pending one).
  EXPECT_TRUE(exec.updatePlan(plan_a, 1u));
  EXPECT_TRUE(exec.hasNewPlan());
  auto consumed = exec.takeNewPlan();
  EXPECT_EQ(consumed.second, 1u);
  EXPECT_FALSE(exec.hasNewPlan());

  // Equivalent content, new version: version advances, no new setPlan.
  EXPECT_TRUE(exec.updatePlan(plan_a_equiv, 2u));
  EXPECT_FALSE(exec.hasNewPlan()) << "equivalent path must not re-trigger "
                                    "controller->setPlan";
  EXPECT_EQ(exec.getPlanVersion(), 2u);

  // Different content: consumed by the run loop on the next cycle.
  EXPECT_TRUE(exec.updatePlan(plan_b, 3u));
  EXPECT_TRUE(exec.hasNewPlan());
  consumed = exec.takeNewPlan();
  EXPECT_EQ(consumed.second, 3u);
}
