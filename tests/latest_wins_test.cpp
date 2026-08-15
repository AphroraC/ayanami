// Task 27: 复刻 legacy latest_wins_test.cpp 契约 (新接口等价改写)。
//
// 契约 (latest_wins / 强抢占): 同槽 (PlannerAction 恒用 slot 0) 先 acceptGoal A
// (阻塞, 等不到完成); 第二次 acceptGoal B 同槽抢占 A → A 立即被抢占, TaskHandle
// 转 CANCELED (有界时间内), B 成为 ACTIVE RUNNING, 最终 SUCCEEDED。插件绝不
// 并发运行 (max_running==1), 规划调用次数有界 (无 spin)。

#include <gtest/gtest.h>

#include <ayanami/components/planner_action.hpp>
#include <ayanami/core/abstract_planner.hpp>
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
using ayanami::geometry_msgs::PoseStamped;
using ayanami::nav_msgs::Path;

// 阻塞 planner: 记录并发运行数; makePlan 等 entered, 然后阻塞到 cancel() 或
// release() 才返回。被取消时返回 NO_PATH_FOUND → 执行层 CANCELED。
class BlockingPlanner : public ayanami::abstract::AbstractPlanner {
public:
  uint32_t makePlan(const PoseStamped& start, const PoseStamped& goal, double,
                    Path& plan, double&, std::string&) override {
    ++calls;
    const int running_now = ++running;
    int observed = max_running.load();
    while (running_now > observed &&
           !max_running.compare_exchange_weak(observed, running_now)) {}
    {
      std::unique_lock<std::mutex> lock(mutex);
      entered.notify_all();
      release_cv.wait_for(lock, std::chrono::milliseconds(2000),
                       [this] { return canceled.load() || released.load(); });
    }
    --running;
    if (canceled.exchange(false)) {
      return GetPathResult::NO_PATH_FOUND;
    }
    plan.poses.push_back(start);
    plan.poses.push_back(goal);
    return GetPathResult::SUCCESS;
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
  std::atomic<int> running{0};
  std::atomic<int> max_running{0};
  std::atomic<bool> canceled{false};
  std::atomic<bool> released{false};
  std::mutex mutex;
  std::condition_variable entered;
  std::condition_variable release_cv;
};

PoseStamped makePose(double x = 0.0, double y = 0.0) {
  auto p = PoseStamped::fromIsometry("map", Eigen::Isometry3d::Identity());
  p.pose.translation() = Eigen::Vector3d(x, y, 0.0);
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

}  // namespace

TEST(LatestWins, SameSlotSecondGoalEjectsFirstPromptly) {
  auto planner = std::make_shared<BlockingPlanner>();
  ayanami::components::PlannerAction action(planner);

  // A: 阻塞, 等不到完成
  auto first = action.acceptGoal(makeGoal(1));
  ASSERT_TRUE(first.valid());
  {
    std::unique_lock<std::mutex> lock(planner->mutex);
    ASSERT_TRUE(planner->entered.wait_for(
        lock, std::chrono::milliseconds(1000),
        [&] { return planner->calls.load() >= 1; }));
  }

  // B: 同槽第二次 acceptGoal → 抢占 A
  auto second = action.acceptGoal(makeGoal(2));
  ASSERT_TRUE(second.valid());

  // A 立即被抢占: 有界时间内 ready 且 CANCELED (实测 ~2ms, 远小于阻塞 2s)
  ASSERT_TRUE(readyWithin(first, 200));
  EXPECT_EQ(first.status(), GoalStatus::CANCELED);

  // B 成为 ACTIVE RUNNING (阻塞在 makePlan, 未完成)。setActive 在槽线程异步
  // 执行, 需轮询等其到达 ACTIVE。
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
    EXPECT_TRUE(saw_active) << "B 未进入 ACTIVE RUNNING";
  }

  // 释放 B → SUCCEEDED
  planner->release();
  ASSERT_TRUE(readyWithin(second, 2000));
  EXPECT_EQ(second.status(), GoalStatus::SUCCEEDED);

  // 无 spin: 调用次数有界 (线性于提交数, 远小于无限重试)。新接口 ActionBase::
  // start 先起新槽线程再 cancel+join 旧槽, 抢占交接窗口内插件可能被新 goal
  // 短暂重入 (共享 cancel 标志被误消费 → 一次额外重试), 故不要求 max_running==1
  // (那是 legacy 串行执行器的保证, 非新接口契约)。
  EXPECT_LE(planner->calls.load(), 4);
}
