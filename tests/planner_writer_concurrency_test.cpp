// 并发回归: 读者持 shared 锁 (lockGlobal 读视图) 期间写者被阻塞, 释放后写者
// 完成写入且无数据竞争 (加锁直读模型)。
//
// 旧架构 (BLOCK #1): CostmapPlannerExecution::run() 整次规划持 global wrapper
// recursive_mutex, 规划长达数百毫秒 (大宗 A*) 期间 obstacle feed 被饿死。
// 新架构: 单写者 + 多读者, 读者经 CostmapMapContext::lockGlobal() 取 RAII shared
// 锁视图 (零拷贝直读 master grid, 单次规划 <10ms 短持), 写者 (唯一) 等读者
// 释放后 unique 锁写入。
//
// 断言:
//  - 读窗口内写者的 unique 锁尝试被阻塞 (try_lock 失败);
//  - 读窗口内另一读者可并发读且多次读值一致 (shared 锁共享, 无 torn 读);
//  - 读窗口释放后写者完成写入, 后续读者看到新值 (无丢失 / 无数据竞争)。

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
#include <ayanami/costmap/components/costmap_planner_execution.hpp>
#include <ayanami/costmap/core/costmap_planner.hpp>
#include <ayanami/messages/failure.hpp>
#include <ayanami/types.hpp>
#include <ayanami/utility/config.hpp>

namespace {

using ayanami::ErrorCode;
using ayanami::PlannerResultT;
using ayanami::geometry_msgs::PoseStamped;
using ayanami::nav_msgs::Path;

// 慢速规划插件: 持有 map ConstPtr, makePlan 内经 lockGlobal() 取读视图 (shared
// 锁) 并阻塞到 release 才返回 (模拟长时搜索)。视图 RAII 存活 = 读窗口持锁,
// 读窗口内写者被阻塞; 记录读到的 (2,2) 值证明读窗口内地图稳定。
class SlowPlanner : public ayanami::costmap::CostmapPlanner {
 public:
  void configure(const ayanami::PluginContext&,
                 ayanami::costmap::CostmapMapContext::ConstPtr map) override {
    map_ = std::move(map);
  }
  PlannerResultT makePlan(
      const ayanami::geometry_msgs::PoseStamped&,
      const ayanami::geometry_msgs::PoseStamped&, double) override {
    ++calls;
    auto view = map_->lockGlobal();
    if (!view.valid()) {
      return PlannerResultT::failure(ErrorCode::MAP_UNAVAILABLE,
                                     "no global view");
    }
    saw_cell.store(view.grid()->getCost(2, 2));
    ayanami::nav_msgs::Path plan;
    plan.header.frame_id = "map";
    plan.poses.resize(1);
    {
      std::unique_lock<std::mutex> lock(mutex);
      entered.notify_all();
      release.wait_for(lock, std::chrono::seconds(30), [this] {
        return released.load();
      });
    }
    return PlannerResultT::success(std::move(plan));
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
  ayanami::costmap::CostmapMapContext::ConstPtr map_;
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

  // 写者先行落一个障碍 (2,2): 读窗口内的稳定读必须看到它且不被改写。
  {
    std::unique_lock<std::shared_mutex> lock(global->mutex());
    global->get().getCostmap()->setCost(2, 2, costmap_2d::LETHAL_OBSTACLE);
    global->updateMapUnlocked(0.0, 0.0, 0.0);
  }

  auto planner = std::make_shared<SlowPlanner>();
  planner->configure(ayanami::PluginContext{"test_planner", ayanami::Config("")}, ctx);
  ayanami::costmap::CostmapPlannerExecution exec("test_planner", planner, ctx);

  exec.start(PoseStamped{}, PoseStamped{}, 0.1);

  // 等 planner 取到读视图并阻塞 (此刻读窗口打开, 持有 shared 锁)。
  {
    std::unique_lock<std::mutex> lock(planner->mutex);
    ASSERT_TRUE(planner->entered.wait_for(
        lock, std::chrono::seconds(5),
        [&] { return planner->calls.load() >= 1; }));
  }

  // 读窗口内另一读者可并发读, 且与 planner 读到的值一致 (无 torn 读)。
  {
    auto view = ctx->lockGlobal();
    ASSERT_TRUE(view.valid());
    EXPECT_EQ(view.grid()->getCost(2, 2), planner->saw_cell.load());
  }

  // 读窗口内写者尝试 unique 锁失败 → 被读者阻塞。
  {
    std::unique_lock<std::shared_mutex> lock_writer(global->mutex(),
                                                    std::try_to_lock);
    EXPECT_FALSE(lock_writer.owns_lock());
  }

  // 放行 planner: 读视图析构释放 shared 锁 → 写者取到 unique 锁完成写入。
  planner->releasePlanner();
  exec.join();
  EXPECT_EQ(planner->calls.load(), 1);
  {
    std::unique_lock<std::shared_mutex> lock(global->mutex());
    global->get().getCostmap()->setCost(
        2, 2, costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
    global->updateMapUnlocked(0.0, 0.0, 0.0);
  }

  // 写者完成后读者读到新值 (无丢失 / 无数据竞争)。
  ctx->readGlobal([&](const costmap_2d::Costmap2D& grid) {
    EXPECT_EQ(grid.getCost(2, 2), costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  });
}