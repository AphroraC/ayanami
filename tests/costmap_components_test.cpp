#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <costmap_2d/costmap_2d.hpp>
#include <costmap_2d/layered_costmap.hpp>
#include <costmap_server/costmap_wrapper.hpp>

#include <ayanami/costmap/components/costmap_controller_action.hpp>
#include <ayanami/costmap/components/costmap_controller_execution.hpp>
#include <ayanami/costmap/components/costmap_planner_action.hpp>
#include <ayanami/costmap/components/costmap_planner_execution.hpp>
#include <ayanami/costmap/components/costmap_recovery_action.hpp>
#include <ayanami/costmap/components/costmap_recovery_execution.hpp>
#include <ayanami/interfaces/callbacks.hpp>

namespace {

using ayanami::geometry_msgs::PoseStamped;
using ayanami::geometry_msgs::TwistStamped;
using ayanami::nav_msgs::Path;

std::shared_ptr<costmap_2d::CostmapWrapper> makeWrapper() {
  auto lm = std::make_shared<costmap_2d::LayeredCostmap>(
      "map", /*rolling_window=*/true, /*track_unknown=*/false);
  lm->resizeMap(10, 10, 0.5, 0.0, 0.0);
  return std::make_shared<costmap_2d::CostmapWrapper>(lm, "map");
}

// 阻塞直到被 cancel: 让执行停在 run 内, 便于观察 preRun/postRun 的激活状态。
class BlockingPlanner : public ayanami::abstract::AbstractPlanner {
 public:
  uint32_t makePlan(const PoseStamped&, const PoseStamped&, double, Path& plan,
                    double&, std::string&) override {
    calls_++;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      entered_.notify_all();
      release_.wait_for(lock, std::chrono::milliseconds(2000),
                        [this] { return canceled_.load(); });
    }
    if (canceled_.exchange(false)) {
      return 0;
    }
    plan.poses.resize(2);
    return 0;
  }
  bool cancel() override {
    canceled_ = true;
    release_.notify_all();
    return true;
  }
  std::atomic<int> calls_{0};
  std::mutex mutex_;
  std::condition_variable entered_;
  std::condition_variable release_;
  std::atomic<bool> canceled_{false};
};

class BlockingRecovery : public ayanami::abstract::AbstractRecovery {
 public:
  uint32_t runRecovery(std::string&) override {
    calls_++;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      entered_.notify_all();
      release_.wait_for(lock, std::chrono::milliseconds(2000),
                        [this] { return canceled_.load(); });
    }
    return 0;
  }
  bool cancel() override {
    canceled_ = true;
    release_.notify_all();
    return true;
  }
  std::atomic<int> calls_{0};
  std::mutex mutex_;
  std::condition_variable entered_;
  std::condition_variable release_;
  std::atomic<bool> canceled_{false};
};

class TestController : public ayanami::abstract::AbstractController {
 public:
  uint32_t computeVelocityCommands(const PoseStamped&, const TwistStamped&,
                                   TwistStamped& cmd_vel, std::string&) override {
    calls_++;
    cmd_vel.twist.linear.x() = 0.5;
    return 0;
  }
  bool isGoalReached(double, double) override { return calls_ >= 3; }
  bool setPlan(const Path&) override { return true; }
  bool cancel() override { return true; }
  std::atomic<int> calls_{0};
};

class TestRecovery : public ayanami::abstract::AbstractRecovery {
 public:
  uint32_t runRecovery(std::string&) override { return 0; }
  bool cancel() override { return true; }
};

// 暴露 protected createExecution 工厂, 验证派生 Action 返回 costmap 版 execution。
class TestablePlannerAction : public ayanami::costmap::CostmapPlannerAction {
 public:
  using CostmapPlannerAction::CostmapPlannerAction;
  ayanami::components::PlannerExecution::Ptr make() { return createExecution(); }
};

class TestableControllerAction : public ayanami::costmap::CostmapControllerAction {
 public:
  using CostmapControllerAction::CostmapControllerAction;
  ayanami::components::ControllerExecution::Ptr make() { return createExecution(); }
};

class TestableRecoveryAction : public ayanami::costmap::CostmapRecoveryAction {
 public:
  using CostmapRecoveryAction::CostmapRecoveryAction;
  ayanami::components::RecoveryExecution::Ptr make() { return createExecution(); }
};

}  // namespace

// ---- CostmapPlannerExecution: preRun 激活 / postRun 停用 / run 持锁 ----

TEST(CostmapComponentsTest, PlannerExecutionActivatesBeforeAndDeactivatesAfter) {
  auto wrapper = makeWrapper();
  auto planner = std::make_shared<BlockingPlanner>();
  ayanami::costmap::CostmapPlannerExecution exec("test_planner", planner,
                                                 wrapper);

  EXPECT_FALSE(wrapper->isActive());
  exec.start(PoseStamped{}, PoseStamped{}, 0.1);
  {
    std::unique_lock<std::mutex> lock(planner->mutex_);
    ASSERT_TRUE(planner->entered_.wait_for(
        lock, std::chrono::milliseconds(1000),
        [&] { return planner->calls_.load() >= 1; }));
  }
  // 执行中: preRun 已 checkActivate
  EXPECT_TRUE(wrapper->isActive());

  exec.cancel();
  exec.join();
  // 执行后: postRun 已 checkDeactivate
  EXPECT_FALSE(wrapper->isActive());
}

TEST(CostmapComponentsTest, PlannerExecutionHoldsCostmapMutexDuringRun) {
  auto wrapper = makeWrapper();
  auto planner = std::make_shared<BlockingPlanner>();
  ayanami::costmap::CostmapPlannerExecution exec("test_planner", planner,
                                                 wrapper);

  exec.start(PoseStamped{}, PoseStamped{}, 0.1);
  {
    std::unique_lock<std::mutex> lock(planner->mutex_);
    ASSERT_TRUE(planner->entered_.wait_for(
        lock, std::chrono::milliseconds(1000),
        [&] { return planner->calls_.load() >= 1; }));
  }
  // run() 覆写持有 recursive_mutex: 其他线程拿不到锁
  const bool locked = wrapper->mutex().try_lock();
  if (locked) {
    wrapper->mutex().unlock();
  }
  EXPECT_FALSE(locked);

  exec.cancel();
  exec.join();
  // 执行结束释放锁
  ASSERT_TRUE(wrapper->mutex().try_lock());
  wrapper->mutex().unlock();
}

// ---- CostmapControllerExecution: safetyCheck 接 isCurrent, 非 current 不发速度 ----

TEST(CostmapComponentsTest, ControllerSafetyCheckStopsWhenCostmapNotCurrent) {
  auto wrapper = makeWrapper();
  wrapper->setCurrent(false);
  auto controller = std::make_shared<TestController>();
  ayanami::costmap::CostmapControllerExecution exec("test_controller", controller,
                                                    wrapper);

  std::atomic<int> stop_cmds{0};
  int cb_id = ayanami::interfaces::NavigationCallbacks::on_velocity_command.add(
      [&](const TwistStamped& cmd) {
        // 非 current: 只允许清零停止命令, 不允许任何非零速度
        EXPECT_DOUBLE_EQ(cmd.twist.linear.x(), 0.0);
        EXPECT_DOUBLE_EQ(cmd.twist.linear.y(), 0.0);
        EXPECT_DOUBLE_EQ(cmd.twist.angular.z(), 0.0);
        stop_cmds++;
      });

  Path plan;
  plan.poses.resize(4);
  exec.setNewPlan(plan);
  exec.setControllerFrequency(500.0);
  exec.start();

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  // safetyCheck false 每周期发布停止命令, computeVelocityCommands 不被调
  EXPECT_EQ(controller->calls_, 0);
  EXPECT_GE(stop_cmds.load(), 1);

  exec.cancel();
  exec.join();
  ayanami::interfaces::NavigationCallbacks::on_velocity_command.remove(cb_id);
}

// ---- CostmapRecoveryExecution: 激活/停用 global+local 两 costmap ----

TEST(CostmapComponentsTest, RecoveryExecutionActivatesDeactivatesBothCostmaps) {
  auto global = makeWrapper();
  auto local = makeWrapper();
  auto recovery = std::make_shared<BlockingRecovery>();
  ayanami::costmap::CostmapRecoveryExecution exec("test_recovery", recovery,
                                                  global, local);

  EXPECT_FALSE(global->isActive());
  EXPECT_FALSE(local->isActive());
  exec.start();
  {
    std::unique_lock<std::mutex> lock(recovery->mutex_);
    ASSERT_TRUE(recovery->entered_.wait_for(
        lock, std::chrono::milliseconds(1000),
        [&] { return recovery->calls_.load() >= 1; }));
  }
  EXPECT_TRUE(global->isActive());
  EXPECT_TRUE(local->isActive());

  exec.cancel();
  exec.join();
  EXPECT_FALSE(global->isActive());
  EXPECT_FALSE(local->isActive());
}

// ---- 派生 Action createExecution 工厂返回 costmap 版 execution ----

TEST(CostmapComponentsTest, PlannerActionFactoryReturnsCostmapPlannerExecution) {
  auto wrapper = makeWrapper();
  auto planner = std::make_shared<BlockingPlanner>();
  TestablePlannerAction action(planner, wrapper);

  auto exec = action.make();
  auto cast = std::dynamic_pointer_cast<ayanami::costmap::CostmapPlannerExecution>(exec);
  ASSERT_NE(cast, nullptr);
  EXPECT_EQ(cast.get(), exec.get());
}

TEST(CostmapComponentsTest, ControllerActionFactoryReturnsCostmapControllerExecution) {
  auto wrapper = makeWrapper();
  auto controller = std::make_shared<TestController>();
  TestableControllerAction action(controller, wrapper);

  auto exec = action.make();
  auto cast =
      std::dynamic_pointer_cast<ayanami::costmap::CostmapControllerExecution>(exec);
  ASSERT_NE(cast, nullptr);
  EXPECT_EQ(cast.get(), exec.get());
}

TEST(CostmapComponentsTest, RecoveryActionFactoryReturnsCostmapRecoveryExecution) {
  auto global = makeWrapper();
  auto local = makeWrapper();
  auto recovery = std::make_shared<TestRecovery>();
  TestableRecoveryAction action(recovery, global, local);

  auto exec = action.make();
  auto cast =
      std::dynamic_pointer_cast<ayanami::costmap::CostmapRecoveryExecution>(exec);
  ASSERT_NE(cast, nullptr);
  EXPECT_EQ(cast.get(), exec.get());
}
