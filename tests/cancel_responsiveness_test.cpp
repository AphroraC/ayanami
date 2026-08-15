// Task 27: 复刻 legacy cancel_responsiveness_test.cpp 契约 (新接口等价改写)。
//
// 契约: 对 RUNNING 的 planner / recovery action 调 cancel(), 任务必须在有界
// 时间内完成 (ready), 且最终 GoalStatus 为 CANCELED。慢插件 (makePlan /
// runRecovery 本需 200ms) 必须在 cancel() 后立即释放, 使任务 100ms 内 ready。

#include <gtest/gtest.h>

#include <ayanami/components/planner_action.hpp>
#include <ayanami/components/recovery_action.hpp>
#include <ayanami/core/abstract_planner.hpp>
#include <ayanami/core/abstract_recovery.hpp>
#include <ayanami/messages/actions.hpp>

#include <Eigen/Geometry>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace {

using ayanami::actions::GetPathGoal;
using ayanami::actions::GetPathResult;
using ayanami::actions::GoalStatus;
using ayanami::actions::RecoveryGoal;
using ayanami::actions::RecoveryResult;
using ayanami::geometry_msgs::PoseStamped;
using ayanami::nav_msgs::Path;

// 慢 planner: makePlan 阻塞最多 200ms (模拟长规划); cancel() 立即释放使其
// 快速返回。被取消时返回 NO_PATH_FOUND (planner 错误段 50+), 执行层经
// cancel_ 置 CANCELED (outcome 保留插件返回码)。
class SlowPlanner : public ayanami::abstract::AbstractPlanner {
public:
  uint32_t makePlan(const PoseStamped&, const PoseStamped&, double, Path&,
                    double&, std::string&) override {
    {
      std::lock_guard<std::mutex> lock(mutex);
      started.store(true);
    }
    started_cv.notify_all();
    std::unique_lock<std::mutex> lock(release_mutex);
    release.wait_for(lock, std::chrono::milliseconds(200),
                     [this] { return canceled.load(); });
    if (canceled.exchange(false)) {
      return GetPathResult::NO_PATH_FOUND;
    }
    return GetPathResult::SUCCESS;
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

// 慢 recovery: runRecovery 阻塞最多 200ms; cancel() 立即释放返回 CANCELED 码。
class SlowRecovery : public ayanami::abstract::AbstractRecovery {
public:
  uint32_t runRecovery(std::string&) override {
    {
      std::lock_guard<std::mutex> lock(mutex);
      started.store(true);
    }
    started_cv.notify_all();
    std::unique_lock<std::mutex> lock(release_mutex);
    release.wait_for(lock, std::chrono::milliseconds(200),
                     [this] { return canceled.load(); });
    if (canceled.exchange(false)) {
      return RecoveryResult::CANCELED;
    }
    return RecoveryResult::SUCCESS;
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
  p.pose.translation() = Eigen::Vector3d(x, y, 0.0);
  return p;
}

// 断言 task 在 deadline 内 ready (cancel 响应性核心契约)。
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

TEST(CancelResponsiveness, CancelDuringPlanningReadyWithinBudget) {
  auto planner = std::make_shared<SlowPlanner>();
  ayanami::components::PlannerAction action(planner);

  GetPathGoal goal;
  goal.start = makePose(0.0, 0.0);
  goal.goal = makePose(1.0, 1.0);
  auto task = action.acceptGoal(goal);
  ASSERT_TRUE(task.valid());

  // 等 planner 进入 makePlan (RUNNING)
  {
    std::unique_lock<std::mutex> lock(planner->mutex);
    ASSERT_TRUE(planner->started_cv.wait_for(
        lock, std::chrono::milliseconds(1000),
        [&] { return planner->started.load(); }));
  }

  // cancel 后必须在 100ms 内 ready (慢插件 200ms 被立即打断)
  ASSERT_TRUE(task.cancel());
  ASSERT_TRUE(readyWithin(task, 100));
  EXPECT_EQ(task.status(), GoalStatus::CANCELED);
  // outcome 是插件返回码, finish 与插件线程存在竞态 (cancel 通知到插件线程
  // 落地 outcome 之间 finish 已读); 稳定契约为 status==CANCELED。
}

TEST(CancelResponsiveness, CancelDuringRecoveryReadyWithinBudget) {
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
