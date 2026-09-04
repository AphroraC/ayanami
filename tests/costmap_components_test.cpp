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
#include <ayanami/abstract/core/abstract_goal_checker.hpp>
#include <ayanami/abstract/core/abstract_progress_checker.hpp>
#include <ayanami/interfaces/callbacks.hpp>
#include <ayanami/abstract/components/robot_state_controller.hpp>
#include <ayanami/messages/failure.hpp>
#include <ayanami/types.hpp>

namespace {

using ayanami::ErrorCode;
using ayanami::ControllerCommand;
using ayanami::ControllerResultT;
using ayanami::PlannerResultT;
using ayanami::RecoveryCycleData;
using ayanami::RecoveryResultT;
using ayanami::RecoveryStatus;
using ayanami::SmootherResultT;
using ayanami::costmap::CostmapContext;
using ayanami::geometry_msgs::PoseStamped;
using ayanami::geometry_msgs::TwistStamped;
using ayanami::nav_msgs::Path;

std::shared_ptr<costmap_2d::CostmapWrapper> makeWrapper() {
  auto lm = std::make_shared<costmap_2d::LayeredCostmap>(
      "map", /*rolling_window=*/true, /*track_unknown=*/false);
  lm->resizeMap(10, 10, 0.5, 0.0, 0.0);
  return std::make_shared<costmap_2d::CostmapWrapper>(lm, "map");
}

std::shared_ptr<ayanami::costmap::CostmapContext> makeContext(
    const std::shared_ptr<costmap_2d::CostmapWrapper>& global = nullptr,
    const std::shared_ptr<costmap_2d::CostmapWrapper>& local = nullptr) {
  auto ctx = std::make_shared<ayanami::costmap::CostmapContext>();
  ctx->setGlobalCostmap(global);
  ctx->setLocalCostmap(local);
  return ctx;
}

// 阻塞直到被 cancel: 让执行停在 run 内, 便于观察 preRun/postRun 的激活状态。
class BlockingPlanner : public ayanami::costmap::CostmapPlanner {
 public:
  void configure(const ayanami::PluginContext&, CostmapContext::ConstPtr) override {}
  PlannerResultT makePlan(const PoseStamped&, const PoseStamped&,
                          double) override {
    calls++;
    {
      std::unique_lock<std::mutex> lock(mutex);
      entered.notify_all();
      release.wait_for(lock, std::chrono::milliseconds(2000),
                        [this] { return canceled.load(); });
    }
    // 被取消也返回成功路径: execution 检测 cancel_requested 判 CANCELED。
    (void)canceled.exchange(false);
    ayanami::nav_msgs::Path plan;
    plan.poses.resize(2);
    return PlannerResultT::success(std::move(plan));
  }
  bool cancel() override {
    canceled = true;
    release.notify_all();
    return true;
  }
  std::atomic<int> calls{0};
  std::mutex mutex;
  std::condition_variable entered;
  std::condition_variable release;
  std::atomic<bool> canceled{false};
};

class BlockingRecovery : public ayanami::costmap::CostmapRecovery {
 public:
  void configure(const ayanami::PluginContext&, CostmapContext::ConstPtr) override {}
  RecoveryResultT tick() override {
    calls++;
    {
      std::unique_lock<std::mutex> lock(mutex);
      entered.notify_all();
      release.wait_for(lock, std::chrono::milliseconds(2000),
                        [this] { return canceled.load(); });
    }
    RecoveryCycleData data;
    data.status = RecoveryStatus::SUCCESS;
    return RecoveryResultT::success(std::move(data));
  }
  bool cancel() override {
    canceled = true;
    release.notify_all();
    return true;
  }
  std::atomic<int> calls{0};
  std::mutex mutex;
  std::condition_variable entered;
  std::condition_variable release;
  std::atomic<bool> canceled{false};
};

class TestController : public ayanami::costmap::CostmapController {
 public:
  void configure(const ayanami::PluginContext&, CostmapContext::ConstPtr) override {}
  ControllerResultT computeVelocityCommand(const PoseStamped&,
                                           const TwistStamped&) override {
    calls++;
    ControllerCommand cmd;
    cmd.velocity = TwistStamped{};
    cmd.velocity.twist.linear.x() = 0.5;
    return ControllerResultT::success(std::move(cmd));
  }
  bool setPlan(const Path&) override { return true; }
  bool cancel() override { return true; }
  std::atomic<int> calls{0};
};

// checker 测试桩: 永不到达 + 恒有进展 (controller 逻辑测试专用)。
class NeverGoalChecker : public ayanami::abstract::AbstractGoalChecker {
 public:
  void reset() override {}
  bool isGoalReached(const PoseStamped&, const PoseStamped&,
                     const TwistStamped&) override {
    return false;
  }
  ayanami::abstract::GoalCheckerTolerances getTolerances() const override {
    return {};
  }
};

class TrueProgressChecker : public ayanami::abstract::AbstractProgressChecker {
 public:
  void reset() override {}
  bool check(const PoseStamped&) override { return true; }
};

class FalseProgressChecker : public ayanami::abstract::AbstractProgressChecker {
 public:
  void reset() override {}
  bool check(const PoseStamped&) override { return false; }
};

class TestRecovery : public ayanami::costmap::CostmapRecovery {
 public:
  void configure(const ayanami::PluginContext&, CostmapContext::ConstPtr) override {}
  RecoveryResultT tick() override {
    RecoveryCycleData data;
    data.status = RecoveryStatus::SUCCESS;
    return RecoveryResultT::success(std::move(data));
  }
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

// ---- CostmapPlannerExecution: preRun 激活 / postRun 停用 / 不持写锁 ----

TEST(CostmapComponentsTest, plannerExecutionActivatesBeforeAndDeactivatesAfter) {
  auto wrapper = makeWrapper();
  auto planner = std::make_shared<BlockingPlanner>();
  ayanami::costmap::CostmapPlannerExecution exec(
      "test_planner", planner, makeContext(wrapper));

  EXPECT_FALSE(wrapper->isActive());
  exec.start(PoseStamped{}, PoseStamped{}, 0.1);
  {
    std::unique_lock<std::mutex> lock(planner->mutex);
    ASSERT_TRUE(planner->entered.wait_for(
        lock, std::chrono::milliseconds(1000),
        [&] { return planner->calls.load() >= 1; }));
  }
  // 执行中: preRun 已 checkActivate
  EXPECT_TRUE(wrapper->isActive());

  exec.cancel();
  exec.join();
  // 执行后: postRun 已 checkDeactivate
  EXPECT_FALSE(wrapper->isActive());
}

TEST(CostmapComponentsTest, plannerExecutionDoesNotHoldCostmapMutex) {
  auto wrapper = makeWrapper();
  auto planner = std::make_shared<BlockingPlanner>();
  ayanami::costmap::CostmapPlannerExecution exec(
      "test_planner", planner, makeContext(wrapper));

  exec.start(PoseStamped{}, PoseStamped{}, 0.1);
  {
    std::unique_lock<std::mutex> lock(planner->mutex);
    ASSERT_TRUE(planner->entered.wait_for(
        lock, std::chrono::milliseconds(1000),
        [&] { return planner->calls.load() >= 1; }));
  }
  // 快照纪律: planner 执行全程不持 wrapper 写锁 —— obstacle 写者可自由获取。
  const bool locked = wrapper->mutex().try_lock();
  ASSERT_TRUE(locked);
  wrapper->mutex().unlock();

  exec.cancel();
  exec.join();
}

// ---- CostmapControllerExecution: safetyCheck 接 isCurrent, 非 current 不发速度 ----

TEST(CostmapComponentsTest, controllerSafetyCheckStopsWhenCostmapNotCurrent) {
  auto wrapper = makeWrapper();
  wrapper->setCurrent(false);
  auto controller = std::make_shared<TestController>();
  ayanami::costmap::CostmapControllerExecution exec(
      "test_controller", controller, makeContext(nullptr, wrapper));
  // checker 注入 (execution 要求 checker 存在; 本场景只验证 safetyCheck 不发速度)。
  exec.setGoalChecker(std::make_shared<NeverGoalChecker>());
  exec.setProgressChecker(std::make_shared<TrueProgressChecker>());

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
  EXPECT_EQ(controller->calls, 0);
  EXPECT_GE(stop_cmds.load(), 1);

  exec.cancel();
  exec.join();
  ayanami::interfaces::NavigationCallbacks::on_velocity_command.remove(cb_id);
}

// ---- CostmapControllerExecution: 无进展 → NO_PROGRESS (ABORTED 映射) ----

TEST(CostmapComponentsTest, controllerNoProgressMapsToNoProgressState) {
  auto& robot_state = ayanami::components::RobotStateController::instance();
  robot_state.initialize("map", "odom", "base_link", 10.0);
  robot_state.setTransform("map", "base_link", ayanami::Timestamp::latest(),
                           Eigen::Isometry3d::Identity());
  auto wrapper = makeWrapper();
  auto controller = std::make_shared<TestController>();
  ayanami::costmap::CostmapControllerExecution exec(
      "test_controller", controller, makeContext(nullptr, wrapper));
  exec.setGoalChecker(std::make_shared<NeverGoalChecker>());
  exec.setProgressChecker(std::make_shared<FalseProgressChecker>());

  Path plan;
  plan.poses.resize(4);
  exec.setNewPlan(plan);
  exec.setControllerFrequency(500.0);
  exec.start();

  for (int i = 0; i < 100 &&
                  exec.getState() !=
                      ayanami::components::ControllerExecution::NO_PROGRESS;
       ++i) {
    exec.waitForStateUpdate(std::chrono::milliseconds(10));
  }
  EXPECT_EQ(exec.getState(),
            ayanami::components::ControllerExecution::NO_PROGRESS);
  EXPECT_EQ(exec.getError(), ErrorCode::NO_PROGRESS);
  EXPECT_EQ(controller->calls, 0);

  exec.join();
}

// ---- CostmapRecoveryExecution: 激活/停用 global+local 两 costmap ----

TEST(CostmapComponentsTest, recoveryExecutionActivatesDeactivatesBothCostmaps) {
  auto global = makeWrapper();
  auto local = makeWrapper();
  auto recovery = std::make_shared<BlockingRecovery>();
  ayanami::costmap::CostmapRecoveryExecution exec(
      "test_recovery", recovery, makeContext(global, local));

  EXPECT_FALSE(global->isActive());
  EXPECT_FALSE(local->isActive());
  exec.start();
  {
    std::unique_lock<std::mutex> lock(recovery->mutex);
    ASSERT_TRUE(recovery->entered.wait_for(
        lock, std::chrono::milliseconds(1000),
        [&] { return recovery->calls.load() >= 1; }));
  }
  EXPECT_TRUE(global->isActive());
  EXPECT_TRUE(local->isActive());

  exec.cancel();
  exec.join();
  EXPECT_FALSE(global->isActive());
  EXPECT_FALSE(local->isActive());
}

// ---- 派生 Action createExecution 工厂返回 costmap 版 execution ----

TEST(CostmapComponentsTest, plannerActionFactoryReturnsCostmapPlannerExecution) {
  auto wrapper = makeWrapper();
  auto planner = std::make_shared<BlockingPlanner>();
  TestablePlannerAction action(planner, makeContext(wrapper));

  auto exec = action.make();
  auto cast = std::dynamic_pointer_cast<ayanami::costmap::CostmapPlannerExecution>(exec);
  ASSERT_NE(cast, nullptr);
  EXPECT_EQ(cast.get(), exec.get());
}

TEST(CostmapComponentsTest, controllerActionFactoryReturnsCostmapControllerExecution) {
  auto wrapper = makeWrapper();
  auto controller = std::make_shared<TestController>();
  TestableControllerAction action(controller, makeContext(nullptr, wrapper));

  auto exec = action.make();
  auto cast =
      std::dynamic_pointer_cast<ayanami::costmap::CostmapControllerExecution>(exec);
  ASSERT_NE(cast, nullptr);
  EXPECT_EQ(cast.get(), exec.get());
}

TEST(CostmapComponentsTest, recoveryActionFactoryReturnsCostmapRecoveryExecution) {
  auto global = makeWrapper();
  auto local = makeWrapper();
  auto recovery = std::make_shared<TestRecovery>();
  TestableRecoveryAction action(recovery, makeContext(global, local));

  auto exec = action.make();
  auto cast =
      std::dynamic_pointer_cast<ayanami::costmap::CostmapRecoveryExecution>(exec);
  ASSERT_NE(cast, nullptr);
  EXPECT_EQ(cast.get(), exec.get());
}
