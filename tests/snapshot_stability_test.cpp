// Concurrent execution: a slow reader holds the shared lock inside CostmapMapContext::readGlobal and reads the
// master grid directly (zero copy), mutually excluded and serialized against a concurrent obstacle writer (the only writer,
// writing under the unique lock).
//
// Assertions:
// - while the reader is inside the read callback the writer blocks in unique_lock; after it returns, the writer completes
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

#include <costmap/costmap_2d.hpp>
#include <costmap/layered_costmap.hpp>
#include <costmap/costmap_wrapper.hpp>

#include <ayanami/costmap/components/costmap_map_context.hpp>
#include <ayanami/costmap/components/costmap_smoother_execution.hpp>
#include <ayanami/costmap/core/costmap_smoother.hpp>
#include <ayanami/costmap/core/costmap_snapshot.hpp>
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

/// Slow-reader smoother: smoothPath holds the shared lock by staying inside the map read callback to simulate
/// a slow read (sleep). While locked it samples two cells to record tearing pairs; after release it records a monotonic
/// clock timestamp (compared against the writer unique-lock acquisition timestamp to prove the writer blocked until the reader released).
class SlowSmoother : public ayanami::costmap::CostmapSmoother {
 public:
  void configure(const ayanami::ModuleContext& /*context*/,
                 ayanami::costmap::CostmapMapContext::ConstPtr map) override {
    this->map = std::move(map);
  }

  ResultT<void> smoothPath(Path& path, double) override {
    ++calls;
    // The read callback is the only way to hold the map lock: staying inside it
    // (including the sleep) is what simulates the slow reader the writer must
    // wait for.
    map->readGlobal([&](const ayanami::costmap::CostmapReader& view) {
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
      // Simulates the slow reader: the writer waits exclusively while the lock
      // is held.
      std::this_thread::sleep_for(hold_duration);
    });
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
  smoother->configure(ayanami::ModuleContext{"test_smoother", ayanami::Config("")},
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

  std::thread exec_th([&] { exec.smoothPath(makePath(), 10.0); });
  exec_th.join();
  writer.join();

  EXPECT_EQ(smoother->calls.load(), 1);
  // The reader sees the initial LETHAL at acquire time (under the lock): the writer was still blocked by the lock and had not written.
  EXPECT_EQ(smoother->observed_cost.load(), costmap_2d::LETHAL_OBSTACLE);
  // (a)+(b): the writer unique lock can only be acquired after the reader releases the shared lock.
  EXPECT_GE(smoother->reader_released.load(), 0);
  EXPECT_GE(writer_acquired.load(), 0);
  EXPECT_LE(smoother->reader_released.load(), writer_acquired.load())
      << "writer acquired the lock while a reader still held it (exclusivity broken)";
  EXPECT_EQ(writes.load(), 1);
  // (c): the reader re-reads (shared lock) and sees the new data written by the writer.
  {
    std::shared_lock<std::shared_mutex> lock(global->mutex());
    EXPECT_EQ(global->get().getCostmap()->getCost(3, 3),
              costmap_2d::FREE_SPACE);
  }
}

// The snapshot read contract: snapshotGlobal holds the shared lock only for
// the memcpy, and the returned copy is immutable -- a later writer publish
// must not change it, while a fresh snapshot observes the new data.
TEST(SnapshotStability, snapshotIsImmutableAndLockIsShort) {
  auto global = makeGlobalWrapper();
  auto ctx = std::make_shared<CostmapMapContext>();
  ctx->setGlobalCostmap(global);

  ayanami::costmap::CostmapSnapshot first;
  ASSERT_TRUE(ctx->snapshotGlobal(first));
  ASSERT_TRUE(first.valid());
  EXPECT_EQ(first.frame, "map");
  EXPECT_EQ(first.size_x, 10u);
  EXPECT_EQ(first.size_y, 10u);
  EXPECT_DOUBLE_EQ(first.resolution, 0.5);
  // Cell-center conversion mirrors Costmap2D::mapToWorld.
  EXPECT_DOUBLE_EQ(first.origin_x, 0.0);
  EXPECT_EQ(first.getCost(3, 3), costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(first.getCost(3, 4), costmap_2d::LETHAL_OBSTACLE);
  // worldToMap mirrors Costmap2D: (2.6, 2.4) -> cell (5, 4); out of bounds rejected.
  unsigned int mx = 0;
  unsigned int my = 0;
  ASSERT_TRUE(first.worldToMap(2.6, 2.4, mx, my));
  EXPECT_EQ(mx, 5u);
  EXPECT_EQ(my, 4u);
  EXPECT_FALSE(first.worldToMap(-0.1, 0.0, mx, my));

  // A writer publishes after the snapshot: the copy must NOT change.
  {
    std::unique_lock<std::shared_mutex> lock(global->mutex());
    global->get().getCostmap()->setCost(3, 3, costmap_2d::FREE_SPACE);
    global->get().getCostmap()->setCost(8, 8, costmap_2d::LETHAL_OBSTACLE);
    global->updateMapUnlocked(0.0, 0.0, 0.0);
  }
  EXPECT_EQ(first.getCost(3, 3), costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(first.getCost(8, 8), costmap_2d::FREE_SPACE);

  // A fresh snapshot observes the published state.
  ayanami::costmap::CostmapSnapshot second;
  ASSERT_TRUE(ctx->snapshotGlobal(second));
  EXPECT_EQ(second.getCost(3, 3), costmap_2d::FREE_SPACE);
  EXPECT_EQ(second.getCost(8, 8), costmap_2d::LETHAL_OBSTACLE);

  // No wrapper: snapshot fails and leaves an empty (invalid) out-param.
  auto empty_ctx = std::make_shared<CostmapMapContext>();
  ayanami::costmap::CostmapSnapshot none;
  EXPECT_FALSE(empty_ctx->snapshotGlobal(none));
  EXPECT_FALSE(none.valid());
}

// A slow reader repeatedly locks and reads vs a writer publishing continuously: inconsistent two-cell samples under the reader lock would mean torn reads; the writer
// must complete all writes (not starved).
TEST(SnapshotStability, slowReaderIsNotTornAndWriterIsNotStarved) {
  auto global = makeGlobalWrapper();
  auto ctx = std::make_shared<CostmapMapContext>();
  ctx->setGlobalCostmap(global);

  auto smoother = std::make_shared<SlowSmoother>();
  smoother->hold_duration = std::chrono::milliseconds(1);
  smoother->configure(ayanami::ModuleContext{"test_smoother", ayanami::Config("")},
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

  // Slow reader: samples via multiple read-callback rounds owned by the execution.
  for (int i = 0; i < 30; ++i) {
    std::thread exec_th([&] { exec.smoothPath(makePath(), 10.0); });
    exec_th.join();
  }
  writer.join();

  EXPECT_EQ(writes.load(), k_writes) << "writer starved by slow readers (not all writes completed)";
  EXPECT_EQ(smoother->calls.load(), 30);
  EXPECT_EQ(smoother->torn_pairs.load(), 0)
      << "reader observed a torn grid pair under the shared lock (lock exclusivity broken)";
}
// ---- generation contract (CostmapWrapper::generation) ----

namespace {

/// Minimal layer that always reports a fixed non-empty dirty window and
/// paints one cell with its current value (the rebuild half of the mutation
/// contract).
class PaintingLayer : public costmap_2d::Layer {
 public:
  void setValue(unsigned char v) { value = v; }

  void updateBounds(double, double, double, double* min_x, double* min_y,
                    double* max_x, double* max_y) override {
    useExtraBounds(min_x, min_y, max_x, max_y);
    touch(0.0, 0.0, min_x, min_y, max_x, max_y);
    touch(0.5, 0.5, min_x, min_y, max_x, max_y);
  }

  void updateCosts(costmap_2d::Costmap2D& master_grid, int, int, int,
                   int) override {
    master_grid.setCost(3, 3, value);
    data_current.store(true);
  }

 private:
  unsigned char value{costmap_2d::FREE_SPACE};
};

}  // namespace

TEST(SnapshotStability, generationBumpsOnlyOnRealMutation) {
  // Rebuild half: a layer-reported dirty window bumps the generation; with
  // no layer-reported window (and a non-rolling map) the cycle is a no-op
  // and the generation stays.
  auto lm = std::make_shared<costmap_2d::LayeredCostmap>(
      "map", /*rolling_window=*/false, /*track_unknown=*/false);
  lm->resizeMap(10, 10, 0.5, 0.0, 0.0);
  lm->addLayer(std::make_shared<PaintingLayer>());
  auto wrapper = std::make_shared<costmap_2d::CostmapWrapper>(lm, "map");
  EXPECT_EQ(wrapper->generation(), 0u);

  // Real rebuild through the layer window.
  wrapper->updateMap(0.0, 0.0, 0.0);
  const std::uint64_t after_rebuild = wrapper->generation();
  EXPECT_GT(after_rebuild, 0u);

  // A snapshot records exactly the generation it was taken at, and the
  // context's live counter matches it.
  auto ctx = std::make_shared<CostmapMapContext>();
  ctx->setGlobalCostmap(wrapper);
  ayanami::costmap::CostmapSnapshot snap;
  ASSERT_TRUE(ctx->snapshotGlobal(snap));
  EXPECT_EQ(snap.generation, after_rebuild);
  EXPECT_EQ(ctx->globalGeneration(), after_rebuild);

  // Another real rebuild advances the context counter while the OLD
  // snapshot keeps its own (immutability of the returned copy).
  wrapper->updateMap(0.0, 0.0, 0.0);
  EXPECT_GT(wrapper->generation(), after_rebuild);
  EXPECT_EQ(snap.generation, after_rebuild);
  EXPECT_NE(ctx->globalGeneration(), snap.generation);

  // No-op half: an empty LayeredCostmap reports no dirty window and does
  // not roll -> the cycle mutates nothing and bumps nothing.
  auto plain_lm = std::make_shared<costmap_2d::LayeredCostmap>(
      "map", /*rolling_window=*/false, /*track_unknown=*/false);
  plain_lm->resizeMap(10, 10, 0.5, 0.0, 0.0);
  auto plain = std::make_shared<costmap_2d::CostmapWrapper>(plain_lm, "map");
  plain->updateMap(0.0, 0.0, 0.0);
  EXPECT_EQ(plain->generation(), 0u);

  // Rolling half: once the window is centered, a sub-cell pose change is a
  // no-op (no origin shift) while a >=1-cell move shifts the origin and
  // bumps.
  auto rolling_lm = std::make_shared<costmap_2d::LayeredCostmap>(
      "map", /*rolling_window=*/true, /*track_unknown=*/false);
  rolling_lm->resizeMap(10, 10, 0.5, 0.0, 0.0);
  auto rolling = std::make_shared<costmap_2d::CostmapWrapper>(rolling_lm, "map");
  rolling->updateMap(0.0, 0.0, 0.0);  // initial centering (whole-window jump)
  const std::uint64_t rolling_base = rolling->generation();
  rolling->updateMap(0.2, 0.2, 0.0);  // sub-cell: no shift
  EXPECT_EQ(rolling->generation(), rolling_base);
  rolling->updateMap(0.8, 0.8, 0.0);  // >= 1 cell: shift -> bump
  EXPECT_GT(rolling->generation(), rolling_base);
}

// Buffer reuse: snapshotting twice into the SAME CostmapSnapshot object
// (the held-copy pattern of planner/smoother/controller) refreshes the
// content and the generation, and the second copy is as self-consistent as
// the first -- reuse must never leak stale bytes.
TEST(SnapshotStability, snapshotBufferReuseRefreshesContentAndGeneration) {
  auto lm = std::make_shared<costmap_2d::LayeredCostmap>(
      "map", /*rolling_window=*/false, /*track_unknown=*/false);
  lm->resizeMap(10, 10, 0.5, 0.0, 0.0);
  auto layer = std::make_shared<PaintingLayer>();
  lm->addLayer(layer);
  auto wrapper = std::make_shared<costmap_2d::CostmapWrapper>(lm, "map");
  auto ctx = std::make_shared<CostmapMapContext>();
  ctx->setGlobalCostmap(wrapper);

  ayanami::costmap::CostmapSnapshot snap;  // deliberately reused below

  layer->setValue(costmap_2d::LETHAL_OBSTACLE);
  wrapper->updateMap(0.0, 0.0, 0.0);
  ASSERT_TRUE(ctx->snapshotGlobal(snap));
  ASSERT_TRUE(snap.valid());
  EXPECT_EQ(snap.getCost(3, 3), costmap_2d::LETHAL_OBSTACLE);
  const std::uint64_t first_generation = snap.generation;

  layer->setValue(costmap_2d::FREE_SPACE);
  wrapper->updateMap(0.0, 0.0, 0.0);
  ASSERT_TRUE(ctx->snapshotGlobal(snap));
  EXPECT_TRUE(snap.valid());
  EXPECT_EQ(snap.getCost(3, 3), costmap_2d::FREE_SPACE);
  EXPECT_GT(snap.generation, first_generation);
}
