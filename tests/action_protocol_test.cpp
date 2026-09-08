// Re-creates the legacy action_protocol_test.cpp contract (equivalent rewrite on the new interface, not a copy).
//
// Goal lifecycle: Pending -> Active -> Succeeded/Aborted/Canceled legal transitions.
// - ActionServer protocol layer: PENDING -> ACTIVE (setActive/setExecuting same state) -> terminal;
// terminal is an absorbing state (subsequent setActive/publishResult/cancelGoal are all rejected).
// - PlannerAction acceptGoal main flow: the goal is ACTIVE in the server first; once the result is ready it goes
// to SUCCEEDED/ABORTED/CANCELED and is removed from the server when done (no accumulation).
// - Illegal transitions (repeated finish / unknown goal_id / further transitions after terminal) must be rejected.

#include <gtest/gtest.h>

#include <ayanami/abstract/components/action_server.hpp>
#include <ayanami/abstract/components/planner_action.hpp>
#include <ayanami/abstract/core/abstract_planner.hpp>
#include <ayanami/messages/actions.hpp>
#include <ayanami/types.hpp>

#include <Eigen/Geometry>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

namespace {

using ayanami::ErrorCode;
using ayanami::ResultT;
using ayanami::actions::GetPathFeedback;
using ayanami::actions::GetPathGoal;
using ayanami::actions::GetPathResult;
using ayanami::actions::GoalStatus;
using ayanami::components::ActionServer;
using ayanami::components::PlannerAction;
using ayanami::geometry_msgs::PoseStamped;
using ayanami::nav_msgs::Path;

using Server = ActionServer<GetPathGoal, GetPathResult, GetPathFeedback>;

template <typename Task>
bool waitReady(Task& t, int max_ms = 3000) {
  for (int i = 0; i < max_ms / 10 && !t.ready(); ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return t.ready();
}

/// Instant-success planner: fills the path; verifies SUCCEEDED + non-empty path.
class FillingPlanner : public ayanami::abstract::AbstractPlanner {
public:
  ResultT<void> makePlan(const PoseStamped&, const PoseStamped&,
                          double, ayanami::nav_msgs::Path& plan_out) override {
    calls++;
    Path plan;
    plan.header.frame_id = "map";
    plan.poses.resize(2);
    plan_out = std::move(plan);
    return ResultT<void>::success();
  }
  bool cancel() override { return true; }
  std::atomic<int> calls{0};
};

/// Slow-success planner: blocks until release before filling the path (used to observe the ACTIVE intermediate state).
class GatedPlanner : public ayanami::abstract::AbstractPlanner {
public:
  ResultT<void> makePlan(const PoseStamped& start, const PoseStamped& goal,
                          double, ayanami::nav_msgs::Path& plan_out) override {
    {
      std::lock_guard<std::mutex> lock(mutex);
      started.store(true);
    }
    started_cv.notify_all();
    std::unique_lock<std::mutex> lock(release_mutex);
    release_cv.wait_for(lock, std::chrono::seconds(2),
                     [this] { return release_flag.load(); });
    release_flag.store(false);
    Path plan;
    plan.poses.push_back(start);
    plan.poses.push_back(goal);
    plan_out = std::move(plan);
    return ResultT<void>::success();
  }
  bool cancel() override { return true; }
  void release() {
    release_flag.store(true);
    release_cv.notify_all();
  }
  std::atomic<bool> release_flag{false};
  std::atomic<bool> started{false};
  std::mutex mutex;
  std::condition_variable started_cv;
  std::mutex release_mutex;
  std::condition_variable release_cv;
};

/// Exception-throwing planner: PlannerExecution catches it -> INTERNAL_ERROR -> ABORTED mapping.
class ThrowingPlanner : public ayanami::abstract::AbstractPlanner {
public:
  ResultT<void> makePlan(const PoseStamped&, const PoseStamped&,
                          double, ayanami::nav_msgs::Path& plan_out) override {
    throw std::runtime_error("planner internal error");
  }
  bool cancel() override { return true; }
};

PoseStamped makePose(double x = 0.0, double y = 0.0) {
  auto p = PoseStamped::fromIsometry("map", Eigen::Isometry3d::Identity());
  p.pose.position = Eigen::Vector3d(x, y, 0.0);
  return p;
}

GetPathGoal makeGoal(uint64_t session, uint64_t version) {
  GetPathGoal goal;
  goal.start = makePose(0.0, 0.0);
  goal.goal = makePose(1.0, 1.0);
  goal.session_id = session;
  goal.plan_version = version;
  return goal;
}

}  // namespace

// ---- ActionServer protocol layer: PENDING -> ACTIVE -> terminal legal transitions ----

TEST(ActionProtocol, lifecyclePendingActiveSucceeded) {
  Server server;
  auto task = server.acceptGoal(makeGoal(1, 1));
  ASSERT_TRUE(task.valid());

  auto gh = server.getGoal(task.goal_id());
  ASSERT_TRUE(gh.has_value());
  EXPECT_EQ(gh->status, GoalStatus::PENDING);

  // Legal transition: PENDING -> ACTIVE (Accepted/Executing same state)
  EXPECT_TRUE(server.setActive(task.goal_id()));
  gh = server.getGoal(task.goal_id());
  ASSERT_TRUE(gh.has_value());
  EXPECT_EQ(gh->status, GoalStatus::ACTIVE);
  EXPECT_TRUE(server.setExecuting(task.goal_id()));
  gh = server.getGoal(task.goal_id());
  ASSERT_TRUE(gh.has_value());
  EXPECT_EQ(gh->status, GoalStatus::ACTIVE);

  // -> SUCCEEDED
  GetPathResult result;
  result.error = ErrorCode::NONE;
  EXPECT_TRUE(
      server.publishResult(task.goal_id(), GoalStatus::SUCCEEDED, result));
  ASSERT_TRUE(waitReady(task));
  EXPECT_EQ(task.status(), GoalStatus::SUCCEEDED);
  EXPECT_EQ(server.activeGoalCount(), 0u);
}

TEST(ActionProtocol, lifecycleActiveAborted) {
  Server server;
  auto task = server.acceptGoal(makeGoal(1, 2));
  ASSERT_TRUE(task.valid());
  ASSERT_TRUE(server.setActive(task.goal_id()));

  GetPathResult result;
  result.error = ErrorCode::PATH_NOT_FOUND;
  EXPECT_TRUE(server.publishResult(task.goal_id(), GoalStatus::ABORTED, result));
  ASSERT_TRUE(waitReady(task));
  EXPECT_EQ(task.status(), GoalStatus::ABORTED);
  EXPECT_EQ(task.result().error, ErrorCode::PATH_NOT_FOUND);
  EXPECT_EQ(server.activeGoalCount(), 0u);
}

TEST(ActionProtocol, lifecycleActiveCanceled) {
  Server server;
  auto task = server.acceptGoal(makeGoal(1, 3));
  ASSERT_TRUE(task.valid());
  ASSERT_TRUE(server.setActive(task.goal_id()));

  GetPathResult result;
  result.error = ErrorCode::NONE;
  EXPECT_TRUE(
      server.publishResult(task.goal_id(), GoalStatus::CANCELED, result));
  ASSERT_TRUE(waitReady(task));
  EXPECT_EQ(task.status(), GoalStatus::CANCELED);
  EXPECT_EQ(server.activeGoalCount(), 0u);
}

TEST(ActionProtocol, terminalIsAbsorbingRejectsFurtherTransitions) {
  Server server;
  auto task = server.acceptGoal(makeGoal(1, 4));
  ASSERT_TRUE(task.valid());
  ASSERT_TRUE(server.setActive(task.goal_id()));
  GetPathResult result;
  result.error = ErrorCode::NONE;
  ASSERT_TRUE(
      server.publishResult(task.goal_id(), GoalStatus::SUCCEEDED, result));
  ASSERT_TRUE(waitReady(task));

  // After terminal: setActive / setExecuting / finishing again / cancelGoal are all rejected
  EXPECT_FALSE(server.setActive(task.goal_id()));
  EXPECT_FALSE(server.setExecuting(task.goal_id()));
  EXPECT_FALSE(
      server.publishResult(task.goal_id(), GoalStatus::ABORTED, result));
  EXPECT_FALSE(server.cancelGoal(task.goal_id()));
  // Completed goal has been removed from the map
  EXPECT_FALSE(server.getGoal(task.goal_id()).has_value());
  EXPECT_EQ(server.activeGoalCount(), 0u);
}

TEST(ActionProtocol, unknownGoalRejectsEveryOperation) {
  Server server;
  GetPathResult result;
  result.error = ErrorCode::NONE;
  EXPECT_FALSE(server.setActive(9999));
  EXPECT_FALSE(server.setExecuting(9999));
  EXPECT_FALSE(server.publishResult(9999, GoalStatus::SUCCEEDED, result));
  EXPECT_FALSE(server.cancelGoal(9999));
  EXPECT_FALSE(server.getGoal(9999).has_value());
}

// ---- PlannerAction main flow: acceptGoal -> ACTIVE -> terminal ----

TEST(ActionProtocol, plannerAcceptGoalReachesActiveThenSucceeded) {
  auto planner = std::make_shared<GatedPlanner>();
  ayanami::components::PlannerAction action(planner);

  auto task = action.acceptGoal(makeGoal(7, 11));
  ASSERT_TRUE(task.valid());

  // runImpl calls setActive before exec.start: ACTIVE is observable while the planner blocks
  {
    std::unique_lock<std::mutex> lock(planner->mutex);
    ASSERT_TRUE(planner->started_cv.wait_for(
        lock, std::chrono::milliseconds(1000),
        [&] { return planner->started.load(); }));
  }
  auto gh = action.server()->getGoal(task.goal_id());
  ASSERT_TRUE(gh.has_value());
  EXPECT_EQ(gh->status, GoalStatus::ACTIVE);

  planner->release();
  ASSERT_TRUE(waitReady(task));
  EXPECT_EQ(task.status(), GoalStatus::SUCCEEDED);
  EXPECT_FALSE(task.result().path.poses.empty());
  EXPECT_EQ(task.result().session_id, 7u);
  EXPECT_EQ(task.result().plan_version, 11u);
  EXPECT_EQ(action.server()->activeGoalCount(), 0u);
}

TEST(ActionProtocol, plannerAcceptGoalAbortsOnInternalError) {
  auto planner = std::make_shared<ThrowingPlanner>();
  ayanami::components::PlannerAction action(planner);

  auto task = action.acceptGoal(makeGoal(7, 12));
  ASSERT_TRUE(task.valid());
  ASSERT_TRUE(waitReady(task));
  EXPECT_EQ(task.status(), GoalStatus::ABORTED);
  EXPECT_EQ(action.server()->activeGoalCount(), 0u);
}
