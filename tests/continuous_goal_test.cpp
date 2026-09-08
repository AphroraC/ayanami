// Re-creates the legacy continuous_goal_test.cpp contract (equivalent rewrite on the new interface).
//
// Contract: rapidly repeated acceptGoal of the same goal (tiny intervals) must not spin the system forever. On the
// new interface, the same goal on the same slot goes preempt+join+start: the old goal is preempted (CANCELED) and
// the new goal runs normally from RUNNING to SUCCEEDED; everything completes in bounded time (no hang), with compatible TaskHandle completion order.

#include <gtest/gtest.h>

#include <ayanami/abstract/components/planner_action.hpp>
#include <ayanami/abstract/core/abstract_planner.hpp>
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
#include <vector>

namespace {

using ayanami::ResultT;
using ayanami::actions::GetPathGoal;
using ayanami::actions::GoalStatus;
using ayanami::geometry_msgs::PoseStamped;
using ayanami::nav_msgs::Path;

/// Blocking planner: makePlan waits for release or cancel before returning.
/// cancel sets canceled and wakes; it is cleared after each call, making preemption and release semantics predictable.
class GatedPlanner : public ayanami::abstract::AbstractPlanner {
public:
  ResultT<void> makePlan(const PoseStamped& start, const PoseStamped& goal,
                          double, ayanami::nav_msgs::Path& plan_out) override {
    ++calls;
    {
      std::unique_lock<std::mutex> lock(mutex);
      entered.notify_all();
      release_cv.wait_for(lock, std::chrono::milliseconds(500),
                       [this] { return canceled.load() || released.load(); });
    }
    (void)canceled.exchange(false);
    // The cancelled path also returns a success path: the execution detects cancel_requested and yields CANCELED.
    Path plan;
    plan.poses.push_back(start);
    plan.poses.push_back(goal);
    plan_out = std::move(plan);
    return ResultT<void>::success();
  }

  bool cancel() override {
    canceled.store(true);
    release_cv.notify_all();
    return true;
  }

  void release() {
    released.store(true);
    release_cv.notify_all();
  }

  std::atomic<int> calls{0};
  std::atomic<bool> canceled{false};
  std::atomic<bool> released{false};
  std::mutex mutex;
  std::condition_variable entered;
  std::condition_variable release_cv;
};

PoseStamped makePose(double x = 0.0, double y = 0.0) {
  auto p = PoseStamped::fromIsometry("map", Eigen::Isometry3d::Identity());
  p.pose.position = Eigen::Vector3d(x, y, 0.0);
  return p;
}

GetPathGoal makeGoal() {
  GetPathGoal goal;
  goal.start = makePose(0.0, 0.0);
  goal.goal = makePose(1.0, 1.0);
  return goal;
}

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

/// Two rapid acceptGoal calls with the same goal: the second is ACCEPTED; the first is preempted CANCELED,
/// the second eventually SUCCEEDED; everything completes in bounded time (no hang).
TEST(ContinuousGoalReplacement, sameGoalRapidResubmissionIsStable) {
  auto planner = std::make_shared<GatedPlanner>();
  ayanami::components::PlannerAction action(planner);

  const auto goal = makeGoal();
  auto first = action.acceptGoal(goal);
  ASSERT_TRUE(first.valid());

  // Wait for the first to enter makePlan
  {
    std::unique_lock<std::mutex> lock(planner->mutex);
    ASSERT_TRUE(planner->entered.wait_for(
        lock, std::chrono::milliseconds(1000),
        [&] { return planner->calls.load() >= 1; }));
  }

  // Rapidly resubmit the same goal (tiny interval): the second must be ACCEPTED
  auto second = action.acceptGoal(goal);
  ASSERT_TRUE(second.valid());

  // The first was preempted -> CANCELED; the second is RUNNING (accepted once it enters makePlan)
  ASSERT_TRUE(readyWithin(first, 1000));
  EXPECT_EQ(first.status(), GoalStatus::CANCELED);

  // Release the second -> SUCCEEDED
  planner->release();
  ASSERT_TRUE(readyWithin(second, 2000));
  EXPECT_EQ(second.status(), GoalStatus::SUCCEEDED);
  EXPECT_GE(planner->calls.load(), 2);
}

/// Rapidly submit the same goal many times in a row: only the last one SUCCEEDED, all others preempted CANCELED,
/// all TaskHandles complete within the deadline (no hang / no endless spin).
TEST(ContinuousGoalReplacement, rapidSameGoalSubmissionsOnlyLastWins) {
  auto planner = std::make_shared<GatedPlanner>();
  ayanami::components::PlannerAction action(planner);

  const auto goal = makeGoal();
  std::vector<decltype(action.acceptGoal(goal))> tasks;
  for (int i = 0; i < 5; ++i) {
    tasks.push_back(action.acceptGoal(goal));
    ASSERT_TRUE(tasks.back().valid());
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  // Release the last one (the rest were already preempted)
  planner->release();

  int succeeded = 0;
  int canceled = 0;
  for (auto& task : tasks) {
    ASSERT_TRUE(readyWithin(task, 2000)) << "task 未在时限内完成 (hang?)";
    if (task.status() == GoalStatus::SUCCEEDED) {
      ++succeeded;
    } else if (task.status() == GoalStatus::CANCELED) {
      ++canceled;
    }
  }

  EXPECT_EQ(succeeded, 1);
  EXPECT_EQ(canceled, 4);
  // No endless spin: the call count is bounded (linear in the number of submissions; the shared cancel-flag preemption race can
  // at most retry one makePlan once more, still far from unbounded retries).
  EXPECT_GE(planner->calls.load(), 1);
  EXPECT_LE(planner->calls.load(), 10);
}
