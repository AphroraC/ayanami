// Task 27: 复刻 legacy action_protocol_test.cpp 契约 (新接口等价改写, 不 copy)。
//
// Goal lifecycle: Pending→Active→Succeeded/Aborted/Canceled 法定转换。
// - ActionServer 协议层: PENDING→ACTIVE (setActive/setExecuting 同态)→terminal;
//   terminal 是吸收态 (后续 setActive/publishResult/cancelGoal 均被拒)。
// - PlannerAction acceptGoal 主流: 目标在 server 中先 ACTIVE, 结果 ready 后
//   SUCCEEDED/ABORTED/CANCELED, 完成后从 server 移除 (不累积)。
// - 非法转换 (重复 finish / 未知 goal_id / terminal 后再转换) 必须被拒。

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
using ayanami::PlannerResultT;
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

// 即时成功 planner: 填 path, 验 SUCCEEDED + path 非空。
class FillingPlanner : public ayanami::abstract::AbstractPlanner {
public:
  PlannerResultT makePlan(const PoseStamped&, const PoseStamped&,
                          double) override {
    calls++;
    Path plan;
    plan.header.frame_id = "map";
    plan.poses.resize(2);
    return PlannerResultT::success(std::move(plan));
  }
  bool cancel() override { return true; }
  std::atomic<int> calls{0};
};

// 慢速成功 planner: 阻塞到 release() 才填 path (观察 ACTIVE 中间态用)。
class GatedPlanner : public ayanami::abstract::AbstractPlanner {
public:
  PlannerResultT makePlan(const PoseStamped& start, const PoseStamped& goal,
                          double) override {
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
    return PlannerResultT::success(std::move(plan));
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

// 抛异常 planner: PlannerExecution 捕获 → INTERNAL_ERROR → ABORTED 映射。
class ThrowingPlanner : public ayanami::abstract::AbstractPlanner {
public:
  PlannerResultT makePlan(const PoseStamped&, const PoseStamped&,
                          double) override {
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

// ---- ActionServer 协议层: PENDING→ACTIVE→terminal 法定转换 ----

TEST(ActionProtocol, lifecyclePendingActiveSucceeded) {
  Server server;
  auto task = server.acceptGoal(makeGoal(1, 1));
  ASSERT_TRUE(task.valid());

  auto gh = server.getGoal(task.goal_id());
  ASSERT_TRUE(gh.has_value());
  EXPECT_EQ(gh->status, GoalStatus::PENDING);

  // 合法转换: PENDING → ACTIVE (Accepted/Executing 同态, Ruling #2)
  EXPECT_TRUE(server.setActive(task.goal_id()));
  gh = server.getGoal(task.goal_id());
  ASSERT_TRUE(gh.has_value());
  EXPECT_EQ(gh->status, GoalStatus::ACTIVE);
  EXPECT_TRUE(server.setExecuting(task.goal_id()));
  gh = server.getGoal(task.goal_id());
  ASSERT_TRUE(gh.has_value());
  EXPECT_EQ(gh->status, GoalStatus::ACTIVE);

  // → SUCCEEDED
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

  // terminal 后: setActive / setExecuting / 再 finish / cancelGoal 全被拒
  EXPECT_FALSE(server.setActive(task.goal_id()));
  EXPECT_FALSE(server.setExecuting(task.goal_id()));
  EXPECT_FALSE(
      server.publishResult(task.goal_id(), GoalStatus::ABORTED, result));
  EXPECT_FALSE(server.cancelGoal(task.goal_id()));
  // 完成的 goal 已从 map 移出
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

// ---- PlannerAction 主流: acceptGoal → ACTIVE → terminal ----

TEST(ActionProtocol, plannerAcceptGoalReachesActiveThenSucceeded) {
  auto planner = std::make_shared<GatedPlanner>();
  ayanami::components::PlannerAction action(planner);

  auto task = action.acceptGoal(makeGoal(7, 11));
  ASSERT_TRUE(task.valid());

  // runImpl 先 setActive 再 exec.start(): 阻塞 planner 期可观察到 ACTIVE
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
