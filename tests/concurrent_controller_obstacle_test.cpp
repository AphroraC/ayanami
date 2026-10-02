// Concurrency regression: the controller takes the local read scope via readLocal each cycle (shared
// lock, zero-copy direct read of the master grid), interleaved with a concurrent obstacle writer (drops obstacles +
// updateMapUnlocked recompute); neither side starves.
//
// Old architecture (BLOCK #1): CostmapControllerExecution::run held the local wrapper recursive_mutex for the
// whole control loop, starving the obstacle feed. New architecture: single writer + multiple readers:
// - the controller takes the local read scope via readLocal each cycle and releases it on return;
// - the obstacle writer (the only writer) drops obstacles and recomputes the master under a unique lock;
// - readers hold the lock briefly, the writer waits for readers to release, and both sides make progress (no starvation / no deadlock).
//
// Asserts: both sides complete enough rounds (no starvation / no deadlock) and the controller obtains a valid lock view
// on every compute (no MapUnavailable).

#include <gtest/gtest.h>

#include "test_transform_source.hpp"

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <costmap/costmap_2d.hpp>
#include <costmap/layered_costmap.hpp>
#include <costmap/costmap_wrapper.hpp>

#include <ayanami/abstract/core/abstract_goal_checker.hpp>
#include <ayanami/abstract/core/abstract_progress_checker.hpp>
#include <ayanami/costmap/components/costmap_controller_execution.hpp>
#include <ayanami/costmap/components/costmap_map_context.hpp>
#include <ayanami/abstract/components/robot_state_controller.hpp>
#include <ayanami/costmap/core/costmap_controller.hpp>
#include <ayanami/messages/failure.hpp>
#include <ayanami/types.hpp>

#include <shared_mutex>

using ayanami::testing::testTransforms;
namespace {

using ayanami::ErrorCode;
using ayanami::ResultT;
using ayanami::geometry_msgs::PoseStamped;
using ayanami::geometry_msgs::TwistStamped;
using ayanami::nav_msgs::Path;

/// Lock-read-view fake controller: records the cycle count (concurrency check that the writer is not starved by readers).
class SnapshotController : public ayanami::costmap::CostmapController {
 public:
  void configure(const ayanami::ModuleContext&,
                 ayanami::costmap::CostmapMapContext::ConstPtr map) override {
    map_context = std::move(map);
  }
  ResultT<void> computeVelocityCommand(const PoseStamped&,
                                           const TwistStamped&, ayanami::geometry_msgs::TwistStamped& cmd) override {
    ++cycles;
    // This cycle's map read: the callback's scope is the cycle, so the writer is
    // excluded for exactly that long.
    return map_context->readLocal(
        [&](const ayanami::costmap::CostmapReader& view) -> ResultT<void> {
          if (!view.valid()) {
            ++snapshot_missing;
            return ResultT<void>::failure(ErrorCode::MAP_UNAVAILABLE,
                                          "no local view");
          }
          cmd = TwistStamped{};
          cmd.twist.linear.x() = 0.5;
          return ResultT<void>::success();
        });
  }

  std::atomic<int> cycles{0};
  std::atomic<int> snapshot_missing{0};

 private:
  ayanami::costmap::CostmapMapContext::ConstPtr map_context;
  bool setPlan(const Path&) override { return true; }
  bool cancel() override { return true; }
};

/// Checker test stubs: never reached + always progressing.
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

std::shared_ptr<costmap_2d::CostmapWrapper> makeLocalWrapper() {
  auto lm = std::make_shared<costmap_2d::LayeredCostmap>(
      "map", /*rolling_window=*/true, /*track_unknown=*/false);
  lm->resizeMap(10, 10, 0.5, 0.0, 0.0);
  return std::make_shared<costmap_2d::CostmapWrapper>(lm, "map");
}

}  // namespace

TEST(ConcurrentControllerObstacle, noDeadlockNoStarvationSnapshotRefreshes) {
  auto& robot_state = ayanami::components::RobotStateController::instance();
  robot_state.initialize("map", "odom", "base_link", 10.0);
  testTransforms().set("map", "base_link", ayanami::Timestamp::latest(),
                           Eigen::Isometry3d::Identity());
  auto local = makeLocalWrapper();
  auto ctx = std::make_shared<ayanami::costmap::CostmapMapContext>();
  ctx->setLocalCostmap(local);

  auto controller = std::make_shared<SnapshotController>();
  controller->configure(ayanami::ModuleContext{"test_controller", ayanami::Config("")}, ctx);
  ayanami::costmap::CostmapControllerExecution exec("test_controller", controller,
                                                    ctx);
  exec.setGoalChecker(std::make_shared<NeverGoalChecker>());
  exec.setProgressChecker(std::make_shared<TrueProgressChecker>());

  Path plan;
  plan.poses.resize(4);
  exec.setNewPlan(plan);
  exec.setControllerFrequency(20.0);  // ~50ms/cycle

  std::atomic<bool> feeding{true};
  std::atomic<int> obstacle_feeds{0};
  std::atomic<int> wrapped{0};

  std::thread exec_th([&] { exec.runBlocking(); });

  // Obstacle writer thread: the only writer -- drops obstacles via markObservable under the write lock + updateMap
  // publishes a new epoch. If the controller held the write lock for a whole cycle, this would starve (feeds would stop
  // growing) / conflict with the old implementation that held the lock for the whole loop.
  std::thread feeder([&] {
    while (feeding.load()) {
      {
        std::unique_lock<std::shared_mutex> lock(local->mutex());
        local->get().getCostmap()->setCost(3, 3, costmap_2d::LETHAL_OBSTACLE);
        local->updateMapUnlocked(0.0, 0.0, 0.0);
        ++obstacle_feeds;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    ++wrapped;
  });

  // Run for ~1.2s: at 20Hz -> ~22 controller cycles; every 5ms -> ~240 feeds.
  std::this_thread::sleep_for(std::chrono::milliseconds(1200));

  feeding.store(false);
  feeder.join();

  exec.cancel();
  exec_th.join();

  // Anti-starvation: the controller completes >= 15 cycles and got a valid view every time (no MapUnavailable).
  EXPECT_GE(controller->cycles.load(), 15);
  EXPECT_EQ(controller->snapshot_missing.load(), 0)
      << "controller must get a valid view on every cycle";
  // The obstacle writer also completes >= 15 times (the reader shared lock does not block readers other than the writer;
  // the writer waits for readers to release, both sides progress).
  EXPECT_GE(obstacle_feeds.load(), 15);
}
