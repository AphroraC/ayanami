// Re-creates the legacy latest_wins_test.cpp contract (equivalent rewrite on the new interface).
//
// Contract (latest_wins / strong preemption): on the same slot (PlannerAction always uses slot 0), first acceptGoal A
// (blocking, never completes); a second acceptGoal B on the same slot preempts A -> A is preempted immediately, its TaskHandle
// turns CANCELED (within bounded time), B becomes ACTIVE RUNNING and finally SUCCEEDED. Modules never
// run concurrently (max_running==1), and B makePlan enters only after A makePlan has exited
// (strictly start-after-join, deterministic, no reentrancy). The planning call count is bounded (no spin).

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

namespace {

using ayanami::ErrorCode;
using ayanami::ResultT;
using ayanami::actions::GetPathGoal;
using ayanami::actions::GoalStatus;
using ayanami::geometry_msgs::PoseStamped;
using ayanami::nav_msgs::Path;

/// Blocking planner: records the concurrent run count and entry/exit order; makePlan waits for entered, then blocks
/// until cancel or release. When cancelled it returns NO_PATH_FOUND -> the execution layer yields CANCELED.
class BlockingPlanner : public ayanami::abstract::AbstractPlanner {
public:
  ResultT<void> makePlan(const PoseStamped& start, const PoseStamped& goal,
                          double, ayanami::nav_msgs::Path& plan_out) override {
    ++calls;
    const int seq = ++entered_calls;
    const int running_now = ++running;
    int observed = max_running.load();
    while (running_now > observed &&
           !max_running.compare_exchange_weak(observed, running_now)) {}
    if (seq == 2) {
      // Data for the deterministic assertion: when the second makePlan enters, the first must have already exited the module.
      exit_before_second_entry.store(exited_calls.load());
    }
    {
      std::unique_lock<std::mutex> lock(mutex);
      entered.notify_all();
      release_cv.wait_for(lock, std::chrono::milliseconds(2000),
                       [this] { return canceled.load() || released.load(); });
    }
    --running;
    ++exited_calls;
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
  std::atomic<int> entered_calls{0};
  std::atomic<int> exited_calls{0};
  std::atomic<int> running{0};
  std::atomic<int> max_running{0};
  std::atomic<int> exit_before_second_entry{-1};
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

GetPathGoal makeGoal(uint64_t version) {
  GetPathGoal goal;
  goal.start = makePose(0.0, 0.0);
  goal.goal = makePose(1.0, 1.0);
  goal.plan_version = version;
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

/// Planner supporting setNewGoal in-place updates: setNewGoal returns true immediately so PlannerExecution
/// adopts latest-wins (no cancel/join/new execution); makePlan blocks until report releases it,
/// returns PathNotFound when cancelled, otherwise encodes the latest goal it received into the path.
class LatestWinsPlanner : public ayanami::abstract::AbstractPlanner {
 public:
  ResultT<void> makePlan(const PoseStamped& start, const PoseStamped& goal,
                          double, ayanami::nav_msgs::Path& plan_out) override {
    ++calls;
    ++entered_calls;
    const int running_now = ++running;
    int observed = max_running.load();
    while (running_now > observed &&
           !max_running.compare_exchange_weak(observed, running_now)) {}
    {
      std::unique_lock<std::mutex> lock(mutex);
      release_cv.wait_for(lock, std::chrono::milliseconds(3000),
                          [this] { return canceled.load() || released.load(); });
    }
    --running;
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

  bool setNewGoal(const PoseStamped&, const PoseStamped&, double) override {
    ++set_goal_calls;
    return true;
  }

  void release() {
    released.store(true);
    release_cv.notify_all();
  }

  std::atomic<int> calls{0};
  std::atomic<int> entered_calls{0};
  std::atomic<int> running{0};
  std::atomic<int> max_running{0};
  std::atomic<int> set_goal_calls{0};
  std::atomic<bool> canceled{false};
  std::atomic<bool> released{false};
  std::mutex mutex;
  std::condition_variable release_cv;
};

/// Counts createExecution calls: proves that consecutive goals create only 1 execution thread
/// (created by the first acceptGoal; afterwards everything takes the in-place setNewGoal path).
class CountingPlannerAction : public ayanami::components::PlannerAction {
 public:
  using PlannerAction::PlannerAction;
  std::atomic<int> executions_created{0};

 protected:
  ayanami::components::PlannerExecution::Ptr createExecution() override {
    executions_created.fetch_add(1);
    return PlannerAction::createExecution();
  }
};

}  // namespace

TEST(LatestWins, sameSlotSecondGoalEjectsFirstPromptly) {
  auto planner = std::make_shared<BlockingPlanner>();
  ayanami::components::PlannerAction action(planner);

  // A: blocks, never completes
  auto first = action.acceptGoal(makeGoal(1));
  ASSERT_TRUE(first.valid());
  {
    std::unique_lock<std::mutex> lock(planner->mutex);
    ASSERT_TRUE(planner->entered.wait_for(
        lock, std::chrono::milliseconds(1000),
        [&] { return planner->calls.load() >= 1; }));
  }

  // B: second acceptGoal on the same slot -> preempts A
  auto second = action.acceptGoal(makeGoal(2));
  ASSERT_TRUE(second.valid());

  // A is preempted immediately: ready within bounded time and CANCELED (measured ~2ms, far below the 2s block)
  ASSERT_TRUE(readyWithin(first, 200));
  EXPECT_EQ(first.status(), GoalStatus::CANCELED);

  // B becomes ACTIVE RUNNING (blocked in makePlan, not finished). setActive runs asynchronously on the slot thread,
  // so poll until it reaches ACTIVE.
  EXPECT_FALSE(second.ready());
  {
    bool saw_active = false;
    for (int i = 0; i < 200 && !saw_active; ++i) {
      auto gh = action.server()->getGoal(second.goal_id());
      if (gh && gh->status == GoalStatus::ACTIVE) {
        saw_active = true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    EXPECT_TRUE(saw_active) << "B did not enter ACTIVE RUNNING";
  }

  // Release B -> SUCCEEDED
  planner->release();
  ASSERT_TRUE(readyWithin(second, 2000));
  EXPECT_EQ(second.status(), GoalStatus::SUCCEEDED);

  // Strong serialization (latest_wins / MBF preemption contract): modules never reenter concurrently.
  // ActionBase::start strictly cancels+joins the old slot thread first and only starts the new slot thread after the old
  // execution has fully exited, so same-slot module runs are deterministically serial: max_running must be 1.
  EXPECT_EQ(planner->max_running.load(), 1);
  EXPECT_EQ(planner->running.load(), 0);
  // Deterministic entry order: when the second makePlan (B) enters, the first (A) must have already exited the module.
  EXPECT_GE(planner->exit_before_second_entry.load(), 1);

  // No spin: the call count is bounded (linear in the number of submissions, far below unbounded retries).
  EXPECT_LE(planner->calls.load(), 3);
}

TEST(LatestWins, setNewGoalPlannerHundredGoalsSingleExecutionLatestWins) {
  constexpr int kGoals = 100;
  auto planner = std::make_shared<LatestWinsPlanner>();
  CountingPlannerAction action(planner);

  std::vector<ayanami::components::TaskHandle<ayanami::actions::GetPathResult>>
      tasks;
  tasks.reserve(kGoals);

  // The first goal creates the single execution and blocks in makePlan.
  GetPathGoal first = makeGoal(1);
  first.goal = makePose(1.0, 7.0);
  tasks.push_back(action.acceptGoal(first));
  ASSERT_TRUE(tasks.front().valid());

  // Wait for the first execution thread to enter the module (goals 2..100 afterwards all take the in-place setNewGoal path).
  {
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(2000);
    while (planner->running.load() == 0 &&
           std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    ASSERT_GE(planner->running.load(), 1);
  }

  // 99 consecutive new goals preempt in place: the goal id is encoded into the goal pose x.
  for (int i = 2; i <= kGoals; ++i) {
    GetPathGoal g = makeGoal(i);
    g.goal = makePose(static_cast<double>(i), 7.0);
    tasks.push_back(action.acceptGoal(g));
  }
  ASSERT_EQ(tasks.size(), kGoals);
  for (const auto& t : tasks) {
    ASSERT_TRUE(t.valid());
  }

  // The first 99 tasks are preempted by the newest goal -> CANCELED.
  for (int i = 0; i < kGoals - 1; ++i) {
    ASSERT_TRUE(readyWithin(tasks[i], 2000));
    EXPECT_EQ(tasks[i].status(), GoalStatus::CANCELED);
  }

  // Only one execution thread (created by the first; no rebuild/cancel/join/new thread).
  EXPECT_EQ(action.executions_created.load(), 1);
  // The module setNewGoal was called 99 times (in-place preemption of goals 2..100).
  EXPECT_EQ(planner->set_goal_calls.load(), kGoals - 1);

  // Module max concurrency 1 (driven by a single execution thread, never reentered).
  EXPECT_EQ(planner->max_running.load(), 1);

  // Release: the execution thread adopts the latest (goal 100) and succeeds.
  planner->release();
  ASSERT_TRUE(readyWithin(tasks[kGoals - 1], 3000));
  EXPECT_EQ(tasks[kGoals - 1].status(), GoalStatus::SUCCEEDED);
  EXPECT_EQ(tasks[kGoals - 1].result().error, ErrorCode::NONE);

  // The result corresponds to the newest goal (goal id encoded x = 100, y = 7).
  const auto& result = tasks[kGoals - 1].result();
  ASSERT_GE(result.path.poses.size(), 1u);
  EXPECT_DOUBLE_EQ(result.path.poses.back().pose.position.x(),
                   static_cast<double>(kGoals));
  EXPECT_DOUBLE_EQ(result.path.poses.back().pose.position.y(), 7.0);

  // No concurrency residue.
  EXPECT_EQ(planner->running.load(), 0);
}
