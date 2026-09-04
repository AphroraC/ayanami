// 并发达执行: 慢读者经 CostmapContext::lockGlobal() 持 shared 锁直接读
// master grid (CostmapView 零拷贝), 与并发 obstacle 写者 (唯一写者, unique
// 锁写入) 互斥串行化。
//
// 断言:
//  - 读者持 lockGlobal() 期间, 写者被阻塞在 unique_lock; 读者释放后写者完成
//    写入 (单调时钟时间戳证明锁排他);
//  - 锁内读到的栅格内容一致 (无撕裂), 写者多轮写入不被慢读者饿死;
//  - 写者发布后, 读者重读 (shared 锁) 看到新数据 (获取时刻/释放后的直读)。

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

#include <ayanami/costmap/components/costmap_context.hpp>
#include <ayanami/costmap/components/costmap_smoother_execution.hpp>
#include <ayanami/costmap/core/costmap_smoother.hpp>
#include <ayanami/messages/failure.hpp>
#include <ayanami/types.hpp>
#include <ayanami/utility/config.hpp>

namespace {

using ayanami::SmootherResultT;
using ayanami::costmap::CostmapContext;
using ayanami::nav_msgs::Path;

// 同步门 (仅测试): set() 唤醒全部等待者。
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

// 慢读者 smoother: smoothPath 内 lockGlobal() 持 shared 锁 (构造 view) 模拟
// 慢读 (sleep)。锁内采样两个 cell 记录撕裂对数; 释放后记录单调时钟时间戳
// (与写者获取 unique 锁的时间戳比较, 证明写者阻塞到读者释放)。
class SlowSmoother : public ayanami::costmap::CostmapSmoother {
 public:
  void configure(const ayanami::PluginContext& /*context*/,
                 ayanami::costmap::CostmapContext::ConstPtr map) override {
    this->map = std::move(map);
  }

  SmootherResultT smoothPath(Path& path, double, bool) override {
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
      // 模拟慢读者: 持锁期间写者排他等待。
      std::this_thread::sleep_for(hold_duration);
    }
    reader_released.store(
        std::chrono::steady_clock::now().time_since_epoch().count());
    return SmootherResultT::success(std::move(path));
  }

  bool cancel() override { return true; }

  std::chrono::milliseconds hold_duration{std::chrono::milliseconds(1)};
  std::atomic<int> calls{0};
  std::atomic<int> torn_pairs{0};
  std::atomic<unsigned char> observed_cost{0};
  std::atomic<int64_t> reader_released{-1};
  Gate reader_started;

 private:
  ayanami::costmap::CostmapContext::ConstPtr map;
};

std::shared_ptr<costmap_2d::CostmapWrapper> makeGlobalWrapper() {
  auto lm = std::make_shared<costmap_2d::LayeredCostmap>(
      "map", /*rolling_window=*/false, /*track_unknown=*/false);
  lm->resizeMap(10, 10, 0.5, 0.0, 0.0);
  // 初始 cell (3,3) 与 (3,4) 同为 LETHAL (锁内采样 pair 保持一致)。
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

// 读者持 shared 锁期间写者阻塞在 unique_lock; 释放后写者完成写入; 重读看到
// 新数据。写者到达被阻塞的证明: 写者获取 unique 锁的时钟时间戳 >= 读者释放
// shared 锁的时间戳 (shared_mutex 排他语义 + 单调时钟)。
TEST(SnapshotStability, readerHoldsSharedLockBlocksWriterThenRereadSeesNewData) {
  auto global = makeGlobalWrapper();
  auto ctx = std::make_shared<CostmapContext>();
  ctx->setGlobalCostmap(global);

  auto smoother = std::make_shared<SlowSmoother>();
  smoother->hold_duration = std::chrono::milliseconds(200);
  smoother->configure(ayanami::PluginContext{"test_smoother", ayanami::Config("")},
                      ctx->getSharedConstPtr());
  ayanami::costmap::CostmapSmootherExecution exec("test_smoother", smoother,
                                                  ctx);

  std::atomic<int64_t> writer_acquired{-1};
  std::atomic<int> writes{0};

  // 写者: 等读者声称已持锁后才尝试 unique 锁 (此时必然被阻塞到读者释放)。
  std::thread writer([&] {
    smoother->reader_started.waitFor(std::chrono::seconds(5));
    std::unique_lock<std::shared_mutex> lock(global->mutex());
    writer_acquired.store(
        std::chrono::steady_clock::now().time_since_epoch().count());
    global->get().getCostmap()->setCost(3, 3, costmap_2d::FREE_SPACE);
    global->updateMapUnlocked(0.0, 0.0, 0.0);
    ++writes;
  });

  exec.smoothPath(makePath(), 10.0, false);
  exec.join();
  writer.join();

  EXPECT_EQ(smoother->calls.load(), 1);
  // 读者在获取时刻 (锁内) 看到初始 LETHAL: 写者此时还被锁阻塞, 尚未写入。
  EXPECT_EQ(smoother->observed_cost.load(), costmap_2d::LETHAL_OBSTACLE);
  // (a)+(b): 写者 unique 锁只能在读者释放 shared 锁之后获得。
  EXPECT_GE(smoother->reader_released.load(), 0);
  EXPECT_GE(writer_acquired.load(), 0);
  EXPECT_LE(smoother->reader_released.load(), writer_acquired.load())
      << "写者在读者仍持 shared 锁期间即获锁 (排他语义被破坏)";
  EXPECT_EQ(writes.load(), 1);
  // (c): 读者重读 (shared 锁) 看到写者写入后的新数据。
  {
    std::shared_lock<std::shared_mutex> lock(global->mutex());
    EXPECT_EQ(global->get().getCostmap()->getCost(3, 3),
              costmap_2d::FREE_SPACE);
  }
}

// 慢读者反复持锁读 vs 写者连续发布: 读者锁内两 cell 采样永不一致则 torn; 写者
// 全部完成 (不被饿死)。
TEST(SnapshotStability, slowReaderIsNotTornAndWriterIsNotStarved) {
  auto global = makeGlobalWrapper();
  auto ctx = std::make_shared<CostmapContext>();
  ctx->setGlobalCostmap(global);

  auto smoother = std::make_shared<SlowSmoother>();
  smoother->hold_duration = std::chrono::milliseconds(1);
  smoother->configure(ayanami::PluginContext{"test_smoother", ayanami::Config("")},
                      ctx->getSharedConstPtr());
  ayanami::costmap::CostmapSmootherExecution exec("test_smoother", smoother,
                                                  ctx);

  constexpr int k_writes = 200;
  std::atomic<int> writes{0};

  // 写者: 每轮把 (3,3)/(3,4) 写成相同值 (FREE/LETHAL 交替)。若读者未持排他
  // shared 锁, 两 cell 采样可能跨轮 torn。
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

  // 慢读者: 经 execution 多轮 lockGlobal 采样。
  for (int i = 0; i < 30; ++i) {
    exec.smoothPath(makePath(), 10.0, false);
    exec.join();
  }
  writer.join();

  EXPECT_EQ(writes.load(), k_writes) << "写者被慢读者饿死 (未完成全部写入)";
  EXPECT_EQ(smoother->calls.load(), 30);
  EXPECT_EQ(smoother->torn_pairs.load(), 0)
      << "读者持 shared 锁读到 torn 栅格对 (锁排他性失效)";
}