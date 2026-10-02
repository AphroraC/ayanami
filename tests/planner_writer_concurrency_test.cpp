// Concurrency regression: while a reader is inside the map read callback the writer is blocked;
// after release the writer completes its write with no data race (locked direct-read model).
//
// Old architecture (BLOCK #1): CostmapPlannerExecution::run held the global wrapper recursive_mutex for the entire
// plan, starving the obstacle feed during multi-hundred-millisecond plans (large A*).
// New architecture: single writer + multiple readers; a reader reads via CostmapMapContext::readGlobal
// (zero-copy direct read of the master grid, held briefly for a <10ms plan), and the writer (the only one) waits for readers
// to release before writing under the unique lock.
//
// Assertions:
// - during the read window the writer unique-lock attempt is blocked (try_lock fails);
// - during the read window another reader can read concurrently with consistent values across reads (shared lock shared, no torn reads);
// - after the read window is released the writer completes its write and later readers see the new value (no loss / no data race).

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <thread>

#include <costmap/costmap_2d.hpp>
#include <costmap/layered_costmap.hpp>
#include <costmap/costmap_wrapper.hpp>

#include <ayanami/costmap/components/costmap_map_context.hpp>
#include <ayanami/costmap/components/costmap_planner_execution.hpp>
#include <ayanami/costmap/core/costmap_planner.hpp>
#include <ayanami/messages/failure.hpp>
#include <ayanami/types.hpp>
#include <ayanami/utility/config.hpp>

namespace {

using ayanami::ErrorCode;
using ayanami::ResultT;
using ayanami::geometry_msgs::PoseStamped;
using ayanami::nav_msgs::Path;

/// Slow planning module: holds the map ConstPtr; inside makePlan it reads via readGlobal (shared
/// lock) and blocks until release (simulating a long search). The view RAII lifetime = the read window holding the lock;
/// the writer is blocked during the read window; the recorded (2,2) value proves the map stayed stable within the read window.
class SlowPlanner : public ayanami::costmap::CostmapPlanner {
 public:
  void configure(const ayanami::ModuleContext&,
                 ayanami::costmap::CostmapMapContext::ConstPtr map) override {
    map_context = std::move(map);
  }
  ResultT<void> makePlan(
      const ayanami::geometry_msgs::PoseStamped&,
      const ayanami::geometry_msgs::PoseStamped&, double, ayanami::nav_msgs::Path& plan_out) override {
    ++calls;
    // The read callback is the only way to hold the map lock, so a slow reader
    // is expressed by staying inside it -- exactly the window the writer is
    // expected to be excluded from.
    return map_context->readGlobal(
        [&](const ayanami::costmap::CostmapReader& view) -> ResultT<void> {
          if (!view.valid()) {
            return ResultT<void>::failure(ErrorCode::MAP_UNAVAILABLE,
                                          "no global view");
          }
          saw_cell.store(view.grid()->getCost(2, 2));
          ayanami::nav_msgs::Path plan;
          plan.header.frame_id = "map";
          plan.poses.resize(1);
          {
            std::unique_lock<std::mutex> lock(mutex);
            entered.notify_all();
            release.wait_for(lock, std::chrono::seconds(30),
                             [this] { return released.load(); });
          }
          plan_out = std::move(plan);
          return ResultT<void>::success();
        });
  }
  bool cancel() override { return true; }

  void releasePlanner() {
    released.store(true);
    release.notify_all();
  }

  std::atomic<int> calls{0};
  std::atomic<costmap_2d::CostValue> saw_cell{0};
  std::mutex mutex;
  std::condition_variable entered;
  std::condition_variable release;
  std::atomic<bool> released{false};

 private:
  ayanami::costmap::CostmapMapContext::ConstPtr map_context;
};

std::shared_ptr<costmap_2d::CostmapWrapper> makeGlobalWrapper() {
  auto lm = std::make_shared<costmap_2d::LayeredCostmap>(
      "map", /*rolling_window=*/false, /*track_unknown=*/false);
  lm->resizeMap(10, 10, 0.5, 0.0, 0.0);
  auto wrapper = std::make_shared<costmap_2d::CostmapWrapper>(lm, "map");
  wrapper->updateMap(0.0, 0.0, 0.0);
  return wrapper;
}

}  // namespace

TEST(PlannerWriterConcurrency, readerWindowBlocksWriterThenSeesNewValue) {
  auto global = makeGlobalWrapper();
  auto ctx = std::make_shared<ayanami::costmap::CostmapMapContext>();
  ctx->setGlobalCostmap(global);

  // The writer first drops an obstacle at (2,2): the stable reads within the read window must see it and it must not be rewritten.
  {
    std::unique_lock<std::shared_mutex> lock(global->mutex());
    global->get().getCostmap()->setCost(2, 2, costmap_2d::LETHAL_OBSTACLE);
    global->updateMapUnlocked(0.0, 0.0, 0.0);
  }

  auto planner = std::make_shared<SlowPlanner>();
  planner->configure(ayanami::ModuleContext{"test_planner", ayanami::Config("")}, ctx);
  ayanami::costmap::CostmapPlannerExecution exec("test_planner", planner, ctx);

  exec.prepareGoal(PoseStamped{}, PoseStamped{}, 0.1);
  std::thread exec_th([&] { exec.runBlocking(); });

  // Wait for the planner to take its read view and block (the read window is open and holding the shared lock).
  {
    std::unique_lock<std::mutex> lock(planner->mutex);
    ASSERT_TRUE(planner->entered.wait_for(
        lock, std::chrono::seconds(5),
        [&] { return planner->calls.load() >= 1; }));
  }

  // Another reader can read concurrently within the read window, matching the planner value (no torn reads).
  {
    ctx->readGlobal([&](const ayanami::costmap::CostmapReader& view) {
      ASSERT_TRUE(view.valid());
      EXPECT_EQ(view.grid()->getCost(2, 2), planner->saw_cell.load());
    });
  }

  // Within the read window the writer unique-lock attempt fails -> blocked by the reader.
  {
    std::unique_lock<std::shared_mutex> lock_writer(global->mutex(),
                                                    std::try_to_lock);
    EXPECT_FALSE(lock_writer.owns_lock());
  }

  // Release the planner: the read view destructor frees the shared lock -> the writer takes the unique lock and completes its write.
  planner->releasePlanner();
  exec_th.join();
  EXPECT_EQ(planner->calls.load(), 1);
  {
    std::unique_lock<std::shared_mutex> lock(global->mutex());
    global->get().getCostmap()->setCost(
        2, 2, costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
    global->updateMapUnlocked(0.0, 0.0, 0.0);
  }

  // After the writer finishes, readers read the new value (no loss / no data race).
  ctx->readGlobal([&](const ayanami::costmap::CostmapReader& reader) {
    EXPECT_EQ(reader.grid()->getCost(2, 2),
              costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  });
}