#include <gtest/gtest.h>

#include "test_transform_source.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <costmap/costmap_2d.hpp>
#include <costmap/layered_costmap.hpp>
#include <costmap/costmap_wrapper.hpp>

#include <ayanami/costmap/components/costmap_controller_execution.hpp>
#include <ayanami/costmap/components/costmap_planner_execution.hpp>
#include <ayanami/abstract/core/abstract_goal_checker.hpp>
#include <ayanami/abstract/core/abstract_progress_checker.hpp>
#include <ayanami/interfaces/callbacks.hpp>
#include <ayanami/abstract/components/robot_state_controller.hpp>
#include <ayanami/messages/failure.hpp>
#include <ayanami/types.hpp>

using ayanami::testing::testTransforms;
namespace {

using ayanami::ErrorCode;
using ayanami::ResultT;
using ayanami::RecoveryCycleData;
using ayanami::RecoveryStatus;
using ayanami::costmap::CostmapMapContext;
using ayanami::geometry_msgs::PoseStamped;
using ayanami::geometry_msgs::TwistStamped;
using ayanami::nav_msgs::Path;

std::shared_ptr<costmap_2d::CostmapWrapper> makeWrapper() {
  auto lm = std::make_shared<costmap_2d::LayeredCostmap>(
      "map", /*rolling_window=*/true, /*track_unknown=*/false);
  lm->resizeMap(10, 10, 0.5, 0.0, 0.0);
  return std::make_shared<costmap_2d::CostmapWrapper>(lm, "map");
}

std::shared_ptr<ayanami::costmap::CostmapMapContext> makeContext(
    const std::shared_ptr<costmap_2d::CostmapWrapper>& global = nullptr,
    const std::shared_ptr<costmap_2d::CostmapWrapper>& local = nullptr) {
  auto ctx = std::make_shared<ayanami::costmap::CostmapMapContext>();
  ctx->setGlobalCostmap(global);
  ctx->setLocalCostmap(local);
  return ctx;
}

/// Blocks until cancelled: keeps the execution inside run so the preRun/postRun activation state can be observed.
class BlockingPlanner : public ayanami::costmap::CostmapPlanner {
 public:
  void configure(const ayanami::ModuleContext&, CostmapMapContext::ConstPtr) override {}
  ResultT<void> makePlan(const PoseStamped&, const PoseStamped&,
                          double, ayanami::nav_msgs::Path& plan_out) override {
    calls++;
    {
      std::unique_lock<std::mutex> lock(mutex);
      entered.notify_all();
      release.wait_for(lock, std::chrono::milliseconds(2000),
                        [this] { return canceled.load(); });
    }
    // The cancelled path also returns a success path: the execution detects cancel_requested and yields CANCELED.
    (void)canceled.exchange(false);
    ayanami::nav_msgs::Path plan;
    plan.poses.resize(2);
    plan_out = std::move(plan);
    return ResultT<void>::success();
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
  void configure(const ayanami::ModuleContext&, CostmapMapContext::ConstPtr) override {}
  ResultT<void> computeVelocityCommand(const PoseStamped&,
                                           const TwistStamped&, ayanami::geometry_msgs::TwistStamped& cmd) override {
    calls++;
    cmd = TwistStamped{};
    cmd.twist.linear.x() = 0.5;
    return ResultT<void>::success();
  }
  bool setPlan(const Path&) override { return true; }
  bool cancel() override { return true; }
  std::atomic<int> calls{0};
};

/// Checker test stubs: never reached + always progressing (for controller logic tests only).
class NeverGoalChecker : public ayanami::abstract::AbstractGoalChecker {
 public:
  void reset() override {}
  bool isGoalReached(const PoseStamped&, const PoseStamped&,
                     const TwistStamped&) override {
    return false;
  }
  ayanami::GoalCheckerTolerances getTolerances() const override {
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
TEST(CostmapComponentsTest, plannerExecutionActivatesBeforeAndDeactivatesAfter) {
  auto wrapper = makeWrapper();
  auto planner = std::make_shared<BlockingPlanner>();
  ayanami::costmap::CostmapPlannerExecution exec(
      "test_planner", planner, makeContext(wrapper));

  EXPECT_FALSE(wrapper->isActive());
  exec.prepareGoal(PoseStamped{}, PoseStamped{}, 0.1);
  std::thread exec_th([&] { exec.runBlocking(); });
  {
    std::unique_lock<std::mutex> lock(planner->mutex);
    ASSERT_TRUE(planner->entered.wait_for(
        lock, std::chrono::milliseconds(1000),
        [&] { return planner->calls.load() >= 1; }));
  }
  // During execution: preRun has called checkActivate
  EXPECT_TRUE(wrapper->isActive());

  exec.cancel();
  exec_th.join();
  // After execution: postRun has called checkDeactivate
  EXPECT_FALSE(wrapper->isActive());
}

TEST(CostmapComponentsTest, plannerExecutionDoesNotHoldCostmapMutex) {
  auto wrapper = makeWrapper();
  auto planner = std::make_shared<BlockingPlanner>();
  ayanami::costmap::CostmapPlannerExecution exec(
      "test_planner", planner, makeContext(wrapper));

  exec.prepareGoal(PoseStamped{}, PoseStamped{}, 0.1);
  std::thread exec_th([&] { exec.runBlocking(); });
  {
    std::unique_lock<std::mutex> lock(planner->mutex);
    ASSERT_TRUE(planner->entered.wait_for(
        lock, std::chrono::milliseconds(1000),
        [&] { return planner->calls.load() >= 1; }));
  }
  // Snapshot discipline: the planner never holds the wrapper write lock while executing -- the obstacle writer can take it freely.
  const bool locked = wrapper->mutex().try_lock();
  ASSERT_TRUE(locked);
  wrapper->mutex().unlock();

  exec.cancel();
  exec_th.join();
}

// ---- CostmapControllerExecution: safetyCheck uses isCurrent; SILENT when not current ----

TEST(CostmapComponentsTest, controllerSafetyCheckStopsWhenCostmapNotCurrent) {
  auto wrapper = makeWrapper();
  wrapper->setCurrent(false);
  auto controller = std::make_shared<TestController>();
  ayanami::costmap::CostmapControllerExecution exec(
      "test_controller", controller, makeContext(nullptr, wrapper));
  // Checker injected (the execution requires a checker to exist; this scenario only verifies safetyCheck publishes nothing).
  exec.setGoalChecker(std::make_shared<NeverGoalChecker>());
  exec.setProgressChecker(std::make_shared<TrueProgressChecker>());

  std::atomic<int> published{0};
  int cb_id = ayanami::interfaces::NavigationCallbacks::on_velocity_command.add(
      [&](const TwistStamped&) { published++; });

  Path plan;
  plan.poses.resize(4);
  exec.setNewPlan(plan);
  exec.setControllerFrequency(500.0);
  std::thread exec_th([&] { exec.runBlocking(); });

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  // With safetyCheck false the loop stays SILENT (nav2 semantics: wait for
  // the costmap without publishing) and computeVelocityCommands is never
  // called. The only allowed publication is the single zeroed stop pulse
  // from exec.cancel() below.
  EXPECT_EQ(controller->calls, 0);

  exec.cancel();
  exec_th.join();
  // cancel() contributes exactly ONE zeroed stop pulse (the safety-cancel
  // contract); the stale-map cycles themselves published nothing.
  EXPECT_LE(published.load(), 1);
  ayanami::interfaces::NavigationCallbacks::on_velocity_command.remove(cb_id);
}

// ---- CostmapControllerExecution: no progress -> NO_PROGRESS (mapped to ABORTED) ----

TEST(CostmapComponentsTest, controllerNoProgressMapsToNoProgressState) {
  auto& robot_state = ayanami::components::RobotStateController::instance();
  robot_state.initialize("map", "odom", "base_link", 10.0);
  testTransforms().set("map", "base_link", ayanami::Timestamp::latest(),
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
  std::thread exec_th([&] { exec.runBlocking(); });

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

  exec_th.join();
}
}  // namespace
