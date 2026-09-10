// Re-creates the legacy cancel_responsiveness_test.cpp contract (equivalent rewrite on the new interface).
//
// Contract: calling cancel on a RUNNING planner / recovery action must settle the task within a bounded
// time (ready) with final GoalStatus CANCELED. Slow modules (makePlan /
// runRecovery would normally take 200ms) must be released immediately on cancel so the task is ready within 100ms.

#include <gtest/gtest.h>

#include <ayanami/abstract/components/planner_action.hpp>
#include <ayanami/abstract/components/recovery_action.hpp>
#include <ayanami/abstract/core/abstract_planner.hpp>
#include <ayanami/abstract/core/abstract_recovery.hpp>
#include <ayanami/messages/actions.hpp>
#include <ayanami/messages/failure.hpp>
#include <ayanami/types.hpp>

#include <Eigen/Geometry>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace {

using ayanami::ErrorCode;
using ayanami::RecoveryCycleData;
using ayanami::ResultT;
using ayanami::actions::GetPathGoal;
using ayanami::actions::GoalStatus;
using ayanami::actions::RecoveryGoal;
using ayanami::geometry_msgs::PoseStamped;
using ayanami::nav_msgs::Path;

/// Slow planner: makePlan blocks for up to 200ms (simulating a long plan); cancel releases it immediately so it
/// returns quickly. When cancelled it still returns a success path; the execution layer yields CANCELED via cancel_requested.
class SlowPlanner : public ayanami::abstract::AbstractPlanner {
public:
  ResultT<void> makePlan(const PoseStamped&, const PoseStamped&,
                          double, ayanami::nav_msgs::Path& plan_out) override {
    {
      std::lock_guard<std::mutex> lock(mutex);
      started.store(true);
    }
    started_cv.notify_all();
    std::unique_lock<std::mutex> lock(release_mutex);
    release.wait_for(lock, std::chrono::milliseconds(200),
                     [this] { return canceled.load(); });
    (void)canceled.exchange(false);
    // The cancelled path also returns a success path: the execution detects cancel_requested and yields CANCELED.
    plan_out = Path{};
    return ResultT<void>::success();
  }

  bool cancel() override {
    canceled.store(true);
    release.notify_all();
    return true;
  }

  std::atomic<bool> canceled{false};
  std::atomic<bool> started{false};
  std::mutex mutex;
  std::condition_variable started_cv;
  std::mutex release_mutex;
  std::condition_variable release;
};

/// Slow recovery: the first tick cycle blocks for up to 200ms; cancel releases it immediately and returns the cancelled terminal state.
class SlowRecovery : public ayanami::abstract::AbstractRecovery {
public:
  ResultT<void> tick(RecoveryCycleData& cycle) override {
    {
      std::lock_guard<std::mutex> lock(mutex);
      started.store(true);
    }
    started_cv.notify_all();
    std::unique_lock<std::mutex> lock(release_mutex);
    release.wait_for(lock, std::chrono::milliseconds(200),
                     [this] { return canceled.load(); });
    if (canceled.exchange(false)) {
      return ResultT<void>::failure(ErrorCode::CANCELED, "canceled");
    }
    return ResultT<void>::failure(ErrorCode::RECOVERY_FAILED,
                                    "unexpected completion");
  }

  bool cancel() override {
    canceled.store(true);
    release.notify_all();
    return true;
  }

  std::atomic<bool> canceled{false};
  std::atomic<bool> started{false};
  std::mutex mutex;
  std::condition_variable started_cv;
  std::mutex release_mutex;
  std::condition_variable release;
};

PoseStamped makePose(double x = 0.0, double y = 0.0) {
  auto p = PoseStamped::fromIsometry("map", Eigen::Isometry3d::Identity());
  p.pose.position = Eigen::Vector3d(x, y, 0.0);
  return p;
}

/// Asserts the task becomes ready within the deadline (core cancel-responsiveness contract).
template <typename Task>
bool readyWithin(Task& t, int max_ms) {
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(max_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    if (t.ready()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  return t.ready();
}

}  // namespace

TEST(CancelResponsiveness, cancelDuringPlanningReadyWithinBudget) {
  auto planner = std::make_shared<SlowPlanner>();
  ayanami::components::PlannerAction action(planner);

  GetPathGoal goal;
  goal.start = makePose(0.0, 0.0);
  goal.goal = makePose(1.0, 1.0);
  auto task = action.acceptGoal(goal);
  ASSERT_TRUE(task.valid());

  // Wait for the planner to enter makePlan (RUNNING)
  {
    std::unique_lock<std::mutex> lock(planner->mutex);
    ASSERT_TRUE(planner->started_cv.wait_for(
        lock, std::chrono::milliseconds(1000),
        [&] { return planner->started.load(); }));
  }

  // After cancel it must become ready within 100ms (the slow 200ms module is interrupted immediately)
  ASSERT_TRUE(task.cancel());
  ASSERT_TRUE(readyWithin(task, 100));
  EXPECT_EQ(task.status(), GoalStatus::CANCELED);
  // The failure code depends on when the module returns; finish races with the module thread, so the stable contract is
  // status==CANCELED.
}

TEST(CancelResponsiveness, cancelDuringRecoveryReadyWithinBudget) {
  auto recovery = std::make_shared<SlowRecovery>();
  ayanami::components::RecoveryAction action(recovery);

  RecoveryGoal goal;
  goal.recovery_id = "slow_recovery";
  auto task = action.acceptGoal(goal);
  ASSERT_TRUE(task.valid());

  {
    std::unique_lock<std::mutex> lock(recovery->mutex);
    ASSERT_TRUE(recovery->started_cv.wait_for(
        lock, std::chrono::milliseconds(1000),
        [&] { return recovery->started.load(); }));
  }

  ASSERT_TRUE(task.cancel());
  ASSERT_TRUE(readyWithin(task, 100));
  EXPECT_EQ(task.status(), GoalStatus::CANCELED);
}
