// FsmDriver unit tests: the BT's semantic carry-over verified scenario by
// scenario (forward pipeline, in-place goal revision, recovery rotation +
// budget, non-recoverable short-circuit, DegradeToRawPath, cancellation).
//
// All module roles are in-test mocks; the robot pose is driven through a
// test-owned transform source so the goal checker can observe motion.

#include <ayanami/abstract/components/navigation_fsm.hpp>

#include <ayanami/abstract/components/controller_execution.hpp>
#include <ayanami/abstract/components/planner_execution.hpp>
#include <ayanami/abstract/components/recovery_execution.hpp>
#include <ayanami/abstract/components/robot_state_controller.hpp>
#include <ayanami/abstract/components/smoother_execution.hpp>
#include <ayanami/abstract/core/abstract_controller.hpp>
#include <ayanami/abstract/core/abstract_goal_checker.hpp>
#include <ayanami/abstract/core/abstract_planner.hpp>
#include <ayanami/abstract/core/abstract_progress_checker.hpp>
#include <ayanami/abstract/core/abstract_recovery.hpp>
#include <ayanami/abstract/core/abstract_smoother.hpp>
#include <ayanami/interfaces/callbacks.hpp>
#include <ayanami/messages/timestamp.hpp>
#include <ayanami/types.hpp>

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace {

using namespace ayanami;

// ---- mocks ----

class FsmTestPlanner : public abstract::AbstractPlanner {
 public:
  enum class Mode { kPlan, kFail };

  ResultT<void> makePlan(const geometry_msgs::PoseStamped& start,
                         const geometry_msgs::PoseStamped& goal,
                         double tolerance,
                         nav_msgs::Path& plan) override {
    (void)tolerance;
    ++plan_calls;
    if (mode == Mode::kFail) {
      return ResultT<void>::failure(failure_code, "planner mock failure");
    }
    // Straight line start -> goal, one pose every 0.2 m, stamps = relative
    // seconds at 1 m/s (the family time-parameterization contract).
    plan = nav_msgs::Path{};
    plan.header.frame_id = "map";
    const Eigen::Vector3d delta = goal.pose.position - start.pose.position;
    const double length = delta.head<2>().norm();
    const int steps = std::max(1, static_cast<int>(std::ceil(length / 0.2)));
    double stamp = 0.0;
    for (int i = 0; i <= steps; ++i) {
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = "map";
      pose.pose.position =
          start.pose.position + delta * (static_cast<double>(i) / steps);
      pose.header.stamp = Timestamp(
          std::chrono::duration_cast<std::chrono::nanoseconds>(
              std::chrono::duration<double>(stamp)));
      plan.poses.push_back(std::move(pose));
      stamp += length / steps;
    }
    return ResultT<void>::success();
  }

  std::atomic<int> plan_calls{0};
  std::atomic<Mode> mode{Mode::kPlan};
  std::atomic<ErrorCode> failure_code{ErrorCode::PATH_INVALID};
};

class FsmTestSmoother : public abstract::AbstractSmoother {
 public:
  ResultT<void> smoothPath(nav_msgs::Path& path, double max_duration) override {
    (void)max_duration;
    ++smooth_calls;
    if (fail) {
      return ResultT<void>::failure(ErrorCode::SMOOTHING_FAILED,
                                    "smoother mock failure");
    }
    // Pass-through success (in-place contract: the path is already the
    // smoothed result).
    return ResultT<void>::success();
  }

  std::atomic<int> smooth_calls{0};
  std::atomic<bool> fail{false};
};

class FsmTestController : public abstract::AbstractController {
 public:
  enum class Mode { kDrive, kFailNoProgress };

  ResultT<void> computeVelocityCommand(
      const geometry_msgs::PoseStamped& pose,
      const geometry_msgs::TwistStamped& velocity,
      geometry_msgs::TwistStamped& cmd) override {
    (void)pose;
    (void)velocity;
    ++compute_calls;
    if (mode == Mode::kFailNoProgress) {
      return ResultT<void>::failure(ErrorCode::NO_PROGRESS,
                                    "controller mock failure");
    }
    cmd = geometry_msgs::TwistStamped{};
    cmd.header.frame_id = "base_link";
    cmd.twist.linear.x() = 0.5;
    return ResultT<void>::success();
  }

  std::atomic<Mode> mode{Mode::kDrive};

  bool setPlan(const nav_msgs::Path& plan) override {
    ++set_plan_calls;
    current_goal = plan.poses.empty()
                       ? geometry_msgs::PoseStamped{}
                       : plan.poses.back();
    return true;
  }

  std::atomic<int> compute_calls{0};
  std::atomic<int> set_plan_calls{0};
  geometry_msgs::PoseStamped current_goal;
};

class FsmTestGoalChecker : public abstract::AbstractGoalChecker {
 public:
  void reset() override { ++resets; }
  bool isGoalReached(
      const geometry_msgs::PoseStamped& current_pose,
      const geometry_msgs::PoseStamped& goal_pose,
      const geometry_msgs::TwistStamped& current_velocity) override {
    (void)current_velocity;
    return (current_pose.pose.position - goal_pose.pose.position)
               .head<2>()
               .norm() < 0.1;
  }
  GoalCheckerTolerances getTolerances() const override { return {}; }

  std::atomic<int> resets{0};
};

class FsmTestProgressChecker : public abstract::AbstractProgressChecker {
 public:
  void reset() override {}
  bool check(const geometry_msgs::PoseStamped&) override { return true; }
};

class FsmTestRecovery : public abstract::AbstractRecovery {
 public:
  ResultT<void> tick(RecoveryCycleData& cycle) override {
    ++ticks;
    if (long_running.load()) {
      // A spin/backup-shaped leg: RUNNING until preempted.
      cycle.status = RecoveryStatus::RUNNING;
      return ResultT<void>::success();
    }
    cycle.status = RecoveryStatus::SUCCESS;
    ++runs;
    return ResultT<void>::success();
  }

  std::atomic<int> runs{0};
  std::atomic<int> ticks{0};
  std::atomic<bool> long_running{false};
};

// ---- harness ----

class FsmTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Robot pose: a test-owned transform source (map <- base_link), driven
    // by the atomic x position. Velocity comes from the odometry seam.
    components::RobotStateController::instance().setTransformSource(
        [this](const std::string& target, const std::string& source,
               const Timestamp&, Eigen::Isometry3d& transform, double) {
          if (target != "map" || source != "base_link") {
            return false;
          }
          transform = Eigen::Translation3d(robot_x.load(), robot_y.load(), 0.0);
          return true;
        });
    components::RobotStateController::instance().setOdometry(
        geometry_msgs::TwistStamped{});

    result_subscription =
        interfaces::NavigationCallbacks::on_navigation_result.subscribe(
            [this](const actions::NavigationResult& result) {
              std::lock_guard<std::mutex> lock(result_mutex);
              results.push_back(result.status);
            });
  }

  /// @brief Assemble a driver over the harness mocks (fresh per scenario).
  void buildDriver(uint32_t max_retries = 6,
                   double controller_patience = -1.0,
                   double replan_grace = 0.0,
                   double fsm_frequency = 50.0) {
    planner = std::make_shared<FsmTestPlanner>();
    smoother = std::make_shared<FsmTestSmoother>();
    controller = std::make_shared<FsmTestController>();
    goal_checker = std::make_shared<FsmTestGoalChecker>();
    progress_checker = std::make_shared<FsmTestProgressChecker>();

    auto planner_execution =
        std::make_shared<components::PlannerExecution>(planner);
    auto smoother_execution =
        std::make_shared<components::SmootherExecution>(smoother);
    auto controller_execution =
        std::make_shared<components::ControllerExecution>(controller);
    controller_execution->setGoalChecker(goal_checker);
    controller_execution->setProgressChecker(progress_checker);
    controller_execution->setControllerFrequency(50.0);
    controller_execution->setMaximumVelocity(2.0, 2.0, 2.0);
    controller_execution->setPatience(controller_patience);
    controller_execution->setPoseGrace(0.25);

    std::vector<components::RecoveryExecution::Ptr> recoveries;
    for (int i = 0; i < 2; ++i) {
      auto recovery = std::make_shared<FsmTestRecovery>();
      recovery_mocks.push_back(recovery);
      auto execution =
          std::make_shared<components::RecoveryExecution>(recovery);
      execution->setCyclePeriod(std::chrono::milliseconds(5));
      recoveries.push_back(std::move(execution));
    }

    components::FsmDriverConfig config;
    config.fsm_frequency = fsm_frequency;
    config.planner_frequency = -1.0;  // replan on invalidity only
    config.smoother_frequency = -1.0; // smooth once per raw path
    config.max_nav_retries = max_retries;
    config.replan_grace = replan_grace;

    driver = std::make_unique<components::FsmDriver>(
        planner_execution, smoother_execution, controller_execution,
        std::move(recoveries), config);
    driver->setPathValidator([this](const nav_msgs::Path& path) {
      return path_valid.load() && !path.poses.empty();
    });
  }

  /// @brief Advance the robot toward x_target while the attempt runs.
  template <typename Pred>
  bool driveUntil(const Pred& done, double x_target,
                  std::chrono::milliseconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      if (done()) {
        return true;
      }
      double x = robot_x.load();
      if (x < x_target) {
        robot_x.store(x + 0.05);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return done();
  }

  bool waitFor(const std::function<bool()>& condition,
               std::chrono::milliseconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      if (condition()) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return condition();
  }

  std::vector<actions::GoalStatus> observedResults() {
    std::lock_guard<std::mutex> lock(result_mutex);
    return results;
  }

  void TearDown() override {
    if (driver) {
      driver->stop();
      driver.reset();
    }
  }

  std::shared_ptr<FsmTestPlanner> planner;
  std::shared_ptr<FsmTestSmoother> smoother;
  std::shared_ptr<FsmTestController> controller;
  std::shared_ptr<FsmTestGoalChecker> goal_checker;
  std::shared_ptr<FsmTestProgressChecker> progress_checker;
  std::vector<std::shared_ptr<FsmTestRecovery>> recovery_mocks;
  std::unique_ptr<components::FsmDriver> driver;
  /// @brief Mutable path validity (the plan-once grace scenarios flip
  /// it while following).
  std::atomic<bool> path_valid{true};

  std::atomic<double> robot_x{0.0};
  std::atomic<double> robot_y{0.0};
  CallbackSubscription result_subscription;
  std::mutex result_mutex;
  std::vector<actions::GoalStatus> results;
};

geometry_msgs::PoseStamped goalAt(double x, double y) {
  geometry_msgs::PoseStamped goal;
  goal.header.frame_id = "map";
  goal.pose.position = Eigen::Vector3d(x, y, 0.0);
  goal.pose.orientation = Eigen::Quaterniond::Identity();
  return goal;
}

// ---- scenarios ----

TEST_F(FsmTest, forwardPipelineReachesTheGoal) {
  buildDriver();
  driver->start();
  driver->beginNavigation(goalAt(2.0, 0.0));

  ASSERT_TRUE(driveUntil([this] { return driver->state() ==
                                         components::FsmDriver::State::kSucceeded; },
                         2.0, std::chrono::seconds(20)));
  EXPECT_GE(planner->plan_calls.load(), 1);
  EXPECT_GE(smoother->smooth_calls.load(), 1);
  EXPECT_GT(controller->compute_calls.load(), 0);

  const auto results = observedResults();
  ASSERT_EQ(results.size(), 1u);
  EXPECT_EQ(results.back(), actions::GoalStatus::SUCCEEDED);
}

TEST_F(FsmTest, goalRevisionReplansInPlaceAndSucceeds) {
  buildDriver();
  driver->start();
  driver->beginNavigation(goalAt(2.0, 0.0));
  ASSERT_TRUE(waitFor([this] {
    return driver->state() == components::FsmDriver::State::kFollow;
  }, std::chrono::seconds(10)));
  const auto revision_before = driver->goalRevision();

  // In-flight revision toward a closer goal: no cancel, no zero stream, the
  // budget is refreshed and the attempt continues to SUCCESS.
  driver->beginNavigation(goalAt(1.0, 0.0));
  // The revision is consumed at the plan thread's next tick boundary.
  ASSERT_TRUE(waitFor([this, revision_before] {
    return driver->goalRevision() == revision_before + 1;
  }, std::chrono::seconds(10)));

  ASSERT_TRUE(driveUntil([this] { return driver->state() ==
                                         components::FsmDriver::State::kSucceeded; },
                         1.0, std::chrono::seconds(20)));
  const auto results = observedResults();
  ASSERT_EQ(results.size(), 1u);
  EXPECT_EQ(results.back(), actions::GoalStatus::SUCCEEDED);
}

TEST_F(FsmTest, recoveryRotationBurnsTheBudgetThenAborts) {
  buildDriver(3);
  planner->mode = FsmTestPlanner::Mode::kFail;
  driver->start();
  driver->beginNavigation(goalAt(2.0, 0.0));

  ASSERT_TRUE(waitFor([this] {
    return driver->state() == components::FsmDriver::State::kAborted;
  }, std::chrono::seconds(20)));
  // One recovery per failure until the budget (3) is exhausted; the
  // rotation alternates across the two configured recoveries.
  EXPECT_EQ(recovery_mocks[0]->runs.load() + recovery_mocks[1]->runs.load(),
            3);
  EXPECT_EQ(recovery_mocks[0]->runs.load(), 2);
  EXPECT_EQ(recovery_mocks[1]->runs.load(), 1);

  const auto results = observedResults();
  ASSERT_EQ(results.size(), 1u);
  EXPECT_EQ(results.back(), actions::GoalStatus::ABORTED);
}

TEST_F(FsmTest, newGoalInterruptsRecoveryAndReplansForIt) {
  // nav2 GoalUpdated semantics (the ReactiveFallback short-circuit): a
  // fresh goal delivered while the recovery branch is running preempts it
  // -- the driver consumes the in-place revision (budget reset + forced
  // replan) and re-enters planning for the new goal instead of settling
  // the attempt.
  buildDriver(6);
  planner->mode = FsmTestPlanner::Mode::kFail;
  recovery_mocks[0]->long_running = true;  // first recovery runs long
  driver->start();
  driver->beginNavigation(goalAt(2.0, 0.0));

  ASSERT_TRUE(waitFor([this] {
    return recovery_mocks[0]->ticks.load() >= 2;
  }, std::chrono::seconds(10)));

  // Heal the planner and deliver the fresh goal mid-recovery.
  planner->mode = FsmTestPlanner::Mode::kPlan;
  const auto revision_before = driver->goalRevision();
  driver->beginNavigation(goalAt(1.0, 0.0));

  ASSERT_TRUE(waitFor([this, revision_before] {
    return driver->goalRevision() == revision_before + 1;
  }, std::chrono::seconds(10)));
  ASSERT_TRUE(waitFor([this] {
    return driver->state() == components::FsmDriver::State::kPlan ||
           driver->state() == components::FsmDriver::State::kFollow;
  }, std::chrono::seconds(10)));
  const int ticks_at_interrupt = recovery_mocks[0]->ticks.load();

  // The preempted attempt continues to SUCCESS for the new goal...
  ASSERT_TRUE(driveUntil([this] { return driver->state() ==
                                         components::FsmDriver::State::kSucceeded; },
                         1.0, std::chrono::seconds(20)));
  // ...exactly one navigation result, and the preempted recovery stopped
  // ticking shortly after the interrupt (no zombie cycles).
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_LE(recovery_mocks[0]->ticks.load() - ticks_at_interrupt, 3);
  const auto results = observedResults();
  ASSERT_EQ(results.size(), 1u);
  EXPECT_EQ(results.back(), actions::GoalStatus::SUCCEEDED);
}

TEST_F(FsmTest, nonRecoverableFailureShortCircuitsRecovery) {
  buildDriver(6);
  planner->mode = FsmTestPlanner::Mode::kFail;
  planner->failure_code = ErrorCode::GOAL_INVALID;
  driver->start();
  driver->beginNavigation(goalAt(2.0, 0.0));

  ASSERT_TRUE(waitFor([this] {
    return driver->state() == components::FsmDriver::State::kAborted;
  }, std::chrono::seconds(10)));
  EXPECT_EQ(recovery_mocks[0]->runs.load(), 0);
  EXPECT_EQ(recovery_mocks[1]->runs.load(), 0);
  const auto results = observedResults();
  ASSERT_EQ(results.size(), 1u);
  EXPECT_EQ(results.back(), actions::GoalStatus::ABORTED);
}

TEST_F(FsmTest, stationaryFollowFailuresAbortAfterTwoRecoveryCycles) {
  // The futility guard: a recovery cycle that ends in another follow
  // failure WITHOUT the robot having moved is a stuck loop the (test-phase
  // placeholder) recovery cannot break. Two such cycles abort with the
  // diagnosis instead of burning the whole budget.
  buildDriver(6, /*controller_patience=*/0.05);
  controller->mode = FsmTestController::Mode::kFailNoProgress;
  driver->start();
  driver->beginNavigation(goalAt(2.0, 0.0));
  // The robot is never driven: every recovery cycle is stationary.
  ASSERT_TRUE(waitFor([this] {
    return driver->state() == components::FsmDriver::State::kAborted;
  }, std::chrono::seconds(20)));
  EXPECT_EQ(recovery_mocks[0]->runs.load() + recovery_mocks[1]->runs.load(),
            2);
  const auto results = observedResults();
  ASSERT_EQ(results.size(), 1u);
  EXPECT_EQ(results.back(), actions::GoalStatus::ABORTED);
}

TEST_F(FsmTest, invalidPathGraceDefersToSmootherThenReplans) {
  // The plan-once contract: planner_frequency < 0 (no periodic replan) and
  // a positive replan_grace. While the path is invalid INSIDE the grace
  // window the planner must NOT be called again (the smoother owns the
  // repair); once the invalidity outlives the window the planner re-plans.
  buildDriver(6, -1.0, /*replan_grace=*/0.5);
  driver->start();
  driver->beginNavigation(goalAt(2.0, 0.0));
  ASSERT_TRUE(waitFor([this] {
    return driver->state() == components::FsmDriver::State::kFollow;
  }, std::chrono::seconds(10)));
  EXPECT_EQ(planner->plan_calls.load(), 1);

  // Invalidate the path; 0.3 s is inside the 0.5 s window.
  path_valid.store(false);
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  EXPECT_EQ(planner->plan_calls.load(), 1)
      << "the planner re-planned inside the grace window";

  // Outlive the window: the planner takes over.
  ASSERT_TRUE(waitFor(
      [this] { return planner->plan_calls.load() >= 2; },
      std::chrono::seconds(5)))
      << "the planner never re-planned past the grace window";

  // Recover validity and finish the attempt.
  path_valid.store(true);
  ASSERT_TRUE(driveUntil([this] {
    return driver->state() == components::FsmDriver::State::kSucceeded;
  }, 2.0, std::chrono::seconds(20)));
}

TEST_F(FsmTest, smootherFailureDegradesToTheRawPath) {
  buildDriver();
  smoother->fail = true;
  driver->start();
  driver->beginNavigation(goalAt(2.0, 0.0));

  // Smoothing is an enhancement: the raw path still follows through.
  ASSERT_TRUE(driveUntil([this] { return driver->state() ==
                                         components::FsmDriver::State::kSucceeded; },
                         2.0, std::chrono::seconds(20)));
  EXPECT_GE(smoother->smooth_calls.load(), 1);
  const auto results = observedResults();
  ASSERT_EQ(results.size(), 1u);
  EXPECT_EQ(results.back(), actions::GoalStatus::SUCCEEDED);
}

TEST_F(FsmTest, cancelNavigationSettlesCanceled) {
  buildDriver();
  driver->start();
  driver->beginNavigation(goalAt(20.0, 0.0));
  ASSERT_TRUE(waitFor([this] {
    return driver->state() == components::FsmDriver::State::kFollow;
  }, std::chrono::seconds(10)));

  driver->cancelNavigation();
  ASSERT_TRUE(waitFor([this] {
    return driver->state() == components::FsmDriver::State::kCanceled;
  }, std::chrono::seconds(10)));
  const auto results = observedResults();
  ASSERT_EQ(results.size(), 1u);
  EXPECT_EQ(results.back(), actions::GoalStatus::CANCELED);
}

TEST_F(FsmTest, secondAttemptAfterSuccessStartsClean) {
  buildDriver();
  driver->start();
  driver->beginNavigation(goalAt(0.5, 0.0));
  ASSERT_TRUE(driveUntil([this] {
    return driver->state() == components::FsmDriver::State::kSucceeded;
  }, 0.5, std::chrono::seconds(20)));

  // Second attempt toward a fresh goal (same as the first): the checker
  // baselines must be reset by the new attempt, not inherited.
  driver->beginNavigation(goalAt(1.5, 0.0));
  // The goal is consumed at the plan thread's next tick boundary; wait for
  // the new attempt to actually start before driving the robot forward.
  ASSERT_TRUE(waitFor([this] { return driver->attemptActive(); },
                      std::chrono::seconds(10)));
  ASSERT_TRUE(driveUntil([this] { return driver->state() ==
                                         components::FsmDriver::State::kSucceeded; },
                         1.5, std::chrono::seconds(20)));
  const auto results = observedResults();
  ASSERT_EQ(results.size(), 2u);
  EXPECT_EQ(results[0], actions::GoalStatus::SUCCEEDED);
  EXPECT_EQ(results[1], actions::GoalStatus::SUCCEEDED);
}

// ---- 2026-10-06 optimization regression locks ----

// The inter-tick wait is a goal-mailbox cv wait, not a blind sleep: a goal
// delivered mid-cycle wakes the plan loop immediately. With a 2 Hz tick
// (500 ms period) a blind inter-tick sleep cannot make the goal land inside
// 200 ms, so this deadline separates the two behaviours deterministically.
TEST_F(FsmTest, goalMailboxWakesThePlanLoopImmediately) {
  buildDriver(6, -1.0, 0.0, 2.0);
  driver->start();
  ASSERT_TRUE(waitFor([&] {
    return driver->state() == components::FsmDriver::State::kWaitGoal;
  }, std::chrono::milliseconds(1000)));
  // Park the loop firmly mid-period inside its inter-tick wait, then
  // deliver the goal.
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  driver->beginNavigation(goalAt(3.0, 0.0));
  EXPECT_TRUE(waitFor([&] { return driver->attemptActive(); },
                      std::chrono::milliseconds(200)))
      << "goal delivery waited out an FSM period (the cv wakeup is dead)";
  driver->cancelNavigation();
}

// A kFollow tick must not stack a second wait on top of the follow_cv wait:
// supervision (the per-tick validity probe) runs at fsm_frequency, not at
// half of it. Old behaviour: follow-wait(20 ms) + inter-tick sleep(20 ms)
// = ~25 Hz at a 50 Hz config; fixed: ~50 Hz. 35 probes/s separates them
// with scheduler-jitter margin.
TEST_F(FsmTest, followSupervisionRunsAtTheConfiguredTickRate) {
  buildDriver(6, -1.0, 0.0, 50.0);
  std::atomic<int> validator_calls{0};
  driver->setPathValidator([&](const nav_msgs::Path& path) {
    ++validator_calls;
    return !path.poses.empty();
  });
  driver->start();
  driver->beginNavigation(goalAt(3.0, 0.0));
  ASSERT_TRUE(waitFor([&] {
    return driver->state() == components::FsmDriver::State::kFollow;
  }, std::chrono::milliseconds(1000)));
  validator_calls.store(0);
  // The robot never moves (goal checker never fires, progress checker
  // always ok): the attempt stays in kFollow and probes once per tick.
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  EXPECT_GE(validator_calls.load(), 35)
      << "supervision rate halved (the follow wait and the inter-tick wait "
         "stacked)";
  driver->cancelNavigation();
}

}  // namespace
