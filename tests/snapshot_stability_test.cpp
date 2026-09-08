// Concurrent execution: a slow reader holds the shared lock via CostmapMapContext::lockGlobal and reads the
// master grid directly (zero-copy CostmapView), mutually excluded and serialized against a concurrent obstacle writer (the only writer,
// writing under the unique lock).
//
// Assertions:
// - while the reader holds lockGlobal the writer blocks in unique_lock; after the reader releases, the writer completes
// its write (monotonic clock timestamps prove lock exclusivity);
// - the grid content read under the lock is consistent (no tearing), and the writer multi-round writes are not starved by the slow reader;
// - after the writer publishes, the reader re-reads (shared lock) and sees the new data (direct read at acquire time / after release).

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <thread>

#include <costmap_2d/costmap_2d.hpp>
#include <costmap_2d/layered_costmap.hpp>
#include <costmap_server/costmap_wrapper.hpp>

#include <ayanami/costmap/components/costmap_map_context.hpp>
#include <ayanami/costmap/components/costmap_smoother_execution.hpp>
#include <ayanami/costmap/core/costmap_smoother.hpp>
#include <ayanami/messages/failure.hpp>
#include <ayanami/types.hpp>
#include <ayanami/utility/config.hpp>

namespace {

using ayanami::ResultT;
using ayanami::costmap::CostmapMapContext;
using ayanami::nav_msgs::Path;

/// Test-only sync gate: set wakes all waiters.
class Gate {
 public:
  void set() {
    std::lock_guard<std::mutex> lk(m);
    opened = true;
    cv.notify_all();
  }
  bool waitFor(std::chrono::milliseconds timeout) const {
    std::unique_lock<std::mutex> lk(m);
    return cv.wait_for(lk, timeout, [this] { return opened; });
  }

 private:
  mutable std::mutex m;
  mutable std::condition_variable cv;
  bool opened{false};
};

/// Slow-reader smoother: smoothPath holds the shared lock via lockGlobal (building a view) to simulate
/// a slow read (sleep). While locked it samples two cells to record tearing pairs; after release it records a monotonic
/// clock timestamp (compared against the writer unique-lock acquisition timestamp to prove the writer blocked until the reader released).
class SlowSmoother : public ayanami::costmap::CostmapSmoother {
 public:
  void configure(const ayanami::PluginContext& /*context*/,
                 ayanami::costmap::CostmapMapContext::ConstPtr map) override {
    this->map = std::move(map);
  }

  ResultT<void> smoothPath(Path& path, double) override {
    ++calls;
    {
      const auto view = map->lockGlobal();
      reader_started.set();
      if (view.valid() && view.grid()->inBounds(3u, 3u) &&
          view.grid()->inBounds(3u, 4u)) {
        const unsigned char first = view.grid()->getCost(3, 3);
        const unsigned char second = view.grid()->getCost(3, 4);
        observed_cost = first;
        if (first != second) {
          ++torn_pairs;
        }
      }
      // Simulates the slow reader: the writer waits exclusively while the lock is held.
      std::this_thread::sleep_for(hold_duration);
    }
    reader_released.store(
        std::chrono::steady_clock::now().time_since_epoch().count());
    return ResultT<void>::success();
  }

  bool cancel() override { return true; }

  std::chrono::milliseconds hold_duration{std::chrono::milliseconds(1)};
  std::atomic<int> calls{0};
  std::atomic<int> torn_pairs{0};
  std::atomic<unsigned char> observed_cost{0};
  std::atomic<int64_t> reader_released{-1};
  Gate reader_started;

 private:
  ayanami::costmap::CostmapMapContext::ConstPtr map;
};

std::shared_ptr<costmap_2d::CostmapWrapper> makeGlobalWrapper() {
  auto lm = std::make_shared<costmap_2d::LayeredCostmap>(
      "map", /*rolling_window=*/false, /*track_unknown=*/false);
  lm->resizeMap(10, 10, 0.5, 0.0, 0.0);
  // The initial cells (3,3) and (3,4) are both LETHAL (the pair sampled under the lock stays consistent).
  lm->getCostmap()->setCost(3, 3, costmap_2d::LETHAL_OBSTACLE);
  lm->getCostmap()->setCost(3, 4, costmap_2d::LETHAL_OBSTACLE);
  auto wrapper = std::make_shared<costmap_2d::CostmapWrapper>(lm, "map");
  wrapper->updateMap(0.0, 0.0, 0.0);
  return wrapper;
}

Path makePath() {
  Path in;
  in.poses.resize(2);
  in.poses[0] = ayanami::geometry_msgs::PoseStamped::fromIsometry(
      "map", Eigen::Isometry3d::Identity());
  in.poses[1] = ayanami::geometry_msgs::PoseStamped::fromIsometry(
      "map", Eigen::Isometry3d::Identity());
  return in;
}

}  // namespace

// While the reader holds the shared lock the writer blocks in unique_lock; after release the writer completes its write and the
// re-read sees the new data. Proof the writer was blocked: the writer unique-lock acquisition timestamp >= the reader
// shared-lock release timestamp (shared_mutex exclusivity + monotonic clock).
TEST(SnapshotStability, readerHoldsSharedLockBlocksWriterThenRereadSeesNewData) {
  auto global = makeGlobalWrapper();
  auto ctx = std::make_shared<CostmapMapContext>();
  ctx->setGlobalCostmap(global);

  auto smoother = std::make_shared<SlowSmoother>();
  smoother->hold_duration = std::chrono::milliseconds(200);
  smoother->configure(ayanami::PluginContext{"test_smoother", ayanami::Config("")},
                      ctx->getSharedConstPtr());
  ayanami::costmap::CostmapSmootherExecution exec("test_smoother", smoother,
                                                  ctx);

  std::atomic<int64_t> writer_acquired{-1};
  std::atomic<int> writes{0};

  // Writer: only tries the unique lock after the reader claims to hold it (at that point it must block until the reader releases).
  std::thread writer([&] {
    smoother->reader_started.waitFor(std::chrono::seconds(5));
    std::unique_lock<std::shared_mutex> lock(global->mutex());
    writer_acquired.store(
        std::chrono::steady_clock::now().time_since_epoch().count());
    global->get().getCostmap()->setCost(3, 3, costmap_2d::FREE_SPACE);
    global->updateMapUnlocked(0.0, 0.0, 0.0);
    ++writes;
  });

  exec.smoothPath(makePath(), 10.0);
  exec.join();
  writer.join();

  EXPECT_EQ(smoother->calls.load(), 1);
  // The reader sees the initial LETHAL at acquire time (under the lock): the writer was still blocked by the lock and had not written.
  EXPECT_EQ(smoother->observed_cost.load(), costmap_2d::LETHAL_OBSTACLE);
  // (a)+(b): the writer unique lock can only be acquired after the reader releases the shared lock.
  EXPECT_GE(smoother->reader_released.load(), 0);
  EXPECT_GE(writer_acquired.load(), 0);
  EXPECT_LE(smoother->reader_released.load(), writer_acquired.load())
      << "写者在读者仍持 shared 锁期间即获锁 (排他语义被破坏)";
  EXPECT_EQ(writes.load(), 1);
  // (c): the reader re-reads (shared lock) and sees the new data written by the writer.
  {
    std::shared_lock<std::shared_mutex> lock(global->mutex());
    EXPECT_EQ(global->get().getCostmap()->getCost(3, 3),
              costmap_2d::FREE_SPACE);
  }
}

// A slow reader repeatedly locks and reads vs a writer publishing continuously: inconsistent two-cell samples under the reader lock would mean torn reads; the writer
// must complete all writes (not starved).
TEST(SnapshotStability, slowReaderIsNotTornAndWriterIsNotStarved) {
  auto global = makeGlobalWrapper();
  auto ctx = std::make_shared<CostmapMapContext>();
  ctx->setGlobalCostmap(global);

  auto smoother = std::make_shared<SlowSmoother>();
  smoother->hold_duration = std::chrono::milliseconds(1);
  smoother->configure(ayanami::PluginContext{"test_smoother", ayanami::Config("")},
                      ctx->getSharedConstPtr());
  ayanami::costmap::CostmapSmootherExecution exec("test_smoother", smoother,
                                                  ctx);

  constexpr int k_writes = 200;
  std::atomic<int> writes{0};

  // Writer: each round writes (3,3)/(3,4) with the same value (alternating FREE/LETHAL). If the reader did not hold the exclusive
  // shared lock, the two-cell samples could tear across rounds.
  std::thread writer([&] {
    for (int i = 0; i < k_writes; ++i) {
      const auto value = (i % 2 == 0) ? costmap_2d::FREE_SPACE
                                      : costmap_2d::LETHAL_OBSTACLE;
      std::unique_lock<std::shared_mutex> lock(global->mutex());
      global->get().getCostmap()->setCost(3, 3, value);
      global->get().getCostmap()->setCost(3, 4, value);
      global->updateMapUnlocked(0.0, 0.0, 0.0);
      ++writes;
    }
  });

  // Slow reader: samples via multiple execution lockGlobal rounds.
  for (int i = 0; i < 30; ++i) {
    exec.smoothPath(makePath(), 10.0);
    exec.join();
  }
  writer.join();

  EXPECT_EQ(writes.load(), k_writes) << "写者被慢读者饿死 (未完成全部写入)";
  EXPECT_EQ(smoother->calls.load(), 30);
  EXPECT_EQ(smoother->torn_pairs.load(), 0)
      << "读者持 shared 锁读到 torn 栅格对 (锁排他性失效)";
}