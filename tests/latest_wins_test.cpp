// Task 27: 复刻 legacy latest_wins_test.cpp 契约 (新接口等价改写)。
//
// 契约 (latest_wins / 强抢占): 同槽 (PlannerAction 恒用 slot 0) 先 acceptGoal A
// (阻塞, 等不到完成); 第二次 acceptGoal B 同槽抢占 A → A 立即被抢占, TaskHandle
// 转 CANCELED (有界时间内), B 成为 ACTIVE RUNNING, 最终 SUCCEEDED。插件绝不
// 并发运行 (max_running==1), 且 B 的 makePlan 一定在 A 的 makePlan 退出之后才
// 进入 (严格 join 后 start, 确定性无重入)。规划调用次数有界 (无 spin)。

#include <gtest/gtest.h>

#include <ayanami/abstract/components/planner_action.hpp>
#include <ayanami/abstract/core/abstract_planner.hpp>
#include <ayanami/messages/actions.hpp>
#include <ayanami/messages/failure.hpp>

#include <Eigen/Geometry>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace {

using ayanami::FailureCode;
using ayanami::actions::GetPathGoal;
using ayanami::actions::GoalStatus;
using ayanami::geometry_msgs::PoseStamped;
using ayanami::nav_msgs::Path;

// 阻塞 planner: 记录并发运行数与进出场顺序; makePlan 等 entered, 然后阻塞到
// cancel() 或 release() 才返回。被取消时返回 NO_PATH_FOUND → 执行层 CANCELED。
class BlockingPlanner : public ayanami::abstract::AbstractPlanner {
public:
  FailureCode makePlan(const PoseStamped& start, const PoseStamped& goal, double,
                       Path& plan, double&, std::string&) override {
    ++calls;
    const int seq = ++entered_calls;
    const int running_now = ++running;
    int observed = max_running.load();
    while (running_now > observed &&
           !max_running.compare_exchange_weak(observed, running_now)) {}
    if (seq == 2) {
      // 确定性断言的数据: 第二次 makePlan 进入时, 第一次必须已经退出插件。
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
    if (canceled.exchange(false)) {
      return FailureCode::PathNotFound;
    }
    plan.poses.push_back(start);
    plan.poses.push_back(goal);
    return FailureCode::None;
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

// 支持 setNewGoal in-place 更新的 planner: setNewGoal 立即返回 true 让
// PlannerExecution 走 latest-wins 采纳 (不 cancel/join/新建 execution); makePlan
// 阻塞到 report 释放, 被取消返回 PathNotFound, 否则把传入的最新 goal 编码进路径。
class LatestWinsPlanner : public ayanami::abstract::AbstractPlanner {
 public:
  FailureCode makePlan(const PoseStamped& start, const PoseStamped& goal, double,
                       Path& plan, double&, std::string&) override {
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
    if (canceled.exchange(false)) {
      return FailureCode::PathNotFound;
    }
    plan.poses.push_back(start);
    plan.poses.push_back(goal);
    return FailureCode::None;
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

// 计数 createExecution: 证明连续 goal 只创建过 1 个 execution 线程
// (首个 acceptGoal 创建, 之后全部走 in-place setNewGoal 路径)。
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

  // 强串行 (latest_wins / MBF 抢占契约): 插件从不并发重入。
  // ActionBase::start 严格先 cancel+join 旧槽线程, 等旧 execution 完全退出后才
  // 启动新槽线程, 故同槽插件运行是确定性的串行: max_running 必为 1。
  EXPECT_EQ(planner->max_running.load(), 1);
  EXPECT_EQ(planner->running.load(), 0);
  // 确定性进场顺序: 第二次 makePlan (B) 进入时, 第一次 (A) 必须已退出插件。
  EXPECT_GE(planner->exit_before_second_entry.load(), 1);

  // 无 spin: 调用次数有界 (线性于提交数, 远小于无限重试)。
  EXPECT_LE(planner->calls.load(), 3);
}

TEST(LatestWins, setNewGoalPlannerHundredGoalsSingleExecutionLatestWins) {
  constexpr int kGoals = 100;
  auto planner = std::make_shared<LatestWinsPlanner>();
  CountingPlannerAction action(planner);

  std::vector<ayanami::components::TaskHandle<ayanami::actions::GetPathResult>>
      tasks;
  tasks.reserve(kGoals);

  // 首个 goal 创建唯一 execution 并阻塞在 makePlan。
  GetPathGoal first = makeGoal(1);
  first.goal = makePose(1.0, 7.0);
  tasks.push_back(action.acceptGoal(first));
  ASSERT_TRUE(tasks.front().valid());

  // 等首个 execution 线程进入插件 (之后 2..100 全部走 in-place setNewGoal)。
  {
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(2000);
    while (planner->running.load() == 0 &&
           std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    ASSERT_GE(planner->running.load(), 1);
  }

  // 连续 99 个新 goal in-place 抢占: goal id 编码进 goal 位姿 x。
  for (int i = 2; i <= kGoals; ++i) {
    GetPathGoal g = makeGoal(i);
    g.goal = makePose(static_cast<double>(i), 7.0);
    tasks.push_back(action.acceptGoal(g));
  }
  ASSERT_EQ(tasks.size(), kGoals);
  for (const auto& t : tasks) {
    ASSERT_TRUE(t.valid());
  }

  // 前 99 个 Task 被最新 goal 抢占 → CANCELED。
  for (int i = 0; i < kGoals - 1; ++i) {
    ASSERT_TRUE(readyWithin(tasks[i], 2000));
    EXPECT_EQ(tasks[i].status(), GoalStatus::CANCELED);
  }

  // 只有一个 execution 线程 (首个创建, 无重建/cancel/join/new thread)。
  EXPECT_EQ(action.executions_created.load(), 1);
  // 插件 setNewGoal 被调用 99 次 (goal 2..100 的 in-place 抢占)。
  EXPECT_EQ(planner->set_goal_calls.load(), kGoals - 1);

  // 插件最大并发 1 (单 execution 线程驱动, 绝无重入)。
  EXPECT_EQ(planner->max_running.load(), 1);

  // 释放: 执行线程采纳最新 (goal 100) 并成功。
  planner->release();
  ASSERT_TRUE(readyWithin(tasks[kGoals - 1], 3000));
  EXPECT_EQ(tasks[kGoals - 1].status(), GoalStatus::SUCCEEDED);
  EXPECT_EQ(tasks[kGoals - 1].result().failure, FailureCode::None);

  // 结果对应最新 goal (goal id 编码 x = 100, y = 7)。
  const auto& result = tasks[kGoals - 1].result();
  ASSERT_GE(result.path.poses.size(), 1u);
  EXPECT_DOUBLE_EQ(result.path.poses.back().pose.position.x(),
                   static_cast<double>(kGoals));
  EXPECT_DOUBLE_EQ(result.path.poses.back().pose.position.y(), 7.0);

  // 无并发残留。
  EXPECT_EQ(planner->running.load(), 0);
}
