// Task 27: 复刻 legacy continuous_goal_test.cpp 契约 (新接口等价改写)。
//
// 契约: 同 goal 连续 acceptGoal (间隔极小) 不应导致系统无限 spin。新接口下
// 同 goal 同 slot 走 preempt+join+start, 旧 goal 被抢占 (CANCELED), 新 goal
// 正常 RUNNING 至 SUCCEEDED; 全程有界完成 (无 hang), TaskHandle 完成顺序兼容。

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

using ayanami::PlannerResultT;
using ayanami::actions::GetPathGoal;
using ayanami::actions::GoalStatus;
using ayanami::geometry_msgs::PoseStamped;
using ayanami::nav_msgs::Path;

// 阻塞型 planner: makePlan 等待 release() 或 cancel() 才返回。
// cancel() 置 canceled 并唤醒; 每次调用后清零, 使抢占与释放语义可预测。
class GatedPlanner : public ayanami::abstract::AbstractPlanner {
public:
  PlannerResultT makePlan(const PoseStamped& start, const PoseStamped& goal,
                          double) override {
    ++calls;
    {
      std::unique_lock<std::mutex> lock(mutex);
      entered.notify_all();
      release_cv.wait_for(lock, std::chrono::milliseconds(500),
                       [this] { return canceled.load() || released.load(); });
    }
    (void)canceled.exchange(false);
    // 取消路径也返回成功路径: execution 检测 cancel_requested 判 CANCELED。
    Path plan;
    plan.poses.push_back(start);
    plan.poses.push_back(goal);
    return PlannerResultT::success(std::move(plan));
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

// 同 goal 两次快速 acceptGoal: 第二次 ACCEPTED; 第一个被抢占 CANCELED,
// 第二个最终 SUCCEEDED; 全程有界完成 (无 hang)。
TEST(ContinuousGoalReplacement, sameGoalRapidResubmissionIsStable) {
  auto planner = std::make_shared<GatedPlanner>();
  ayanami::components::PlannerAction action(planner);

  const auto goal = makeGoal();
  auto first = action.acceptGoal(goal);
  ASSERT_TRUE(first.valid());

  // 等第一个进入 makePlan
  {
    std::unique_lock<std::mutex> lock(planner->mutex);
    ASSERT_TRUE(planner->entered.wait_for(
        lock, std::chrono::milliseconds(1000),
        [&] { return planner->calls.load() >= 1; }));
  }

  // 同 goal 快速再发 (间隔极小): 第二次必须 ACCEPTED
  auto second = action.acceptGoal(goal);
  ASSERT_TRUE(second.valid());

  // 第一个被抢占 → CANCELED; 第二个 RUNNING (等其进入 makePlan 即被 accept)
  ASSERT_TRUE(readyWithin(first, 1000));
  EXPECT_EQ(first.status(), GoalStatus::CANCELED);

  // 释放第二个 → SUCCEEDED
  planner->release();
  ASSERT_TRUE(readyWithin(second, 2000));
  EXPECT_EQ(second.status(), GoalStatus::SUCCEEDED);
  EXPECT_GE(planner->calls.load(), 2);
}

// 快速连续提交多个同 goal: 只有最后一个 SUCCEEDED, 其余全部被抢占 CANCELED,
// 所有 TaskHandle 在时限内完成 (无 hang / 无无限 spin)。
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

  // 释放最后一个 (其余已被抢占)
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
  // 无无限 spin: 调用次数有界 (线性于提交数; 共享 cancel 标志的抢占竞态最多
  // 让某个 makePlan 多重试一次, 仍远小于无限重试)。
  EXPECT_GE(planner->calls.load(), 1);
  EXPECT_LE(planner->calls.load(), 10);
}
