// Task 2 (M2 阶段 A BLOCK #3): recovery 锁 global+local (scoped_lock) 防与
// obstacle feed 数据竞态。
//
// 回归对象: CostmapRecoveryExecution::run() 只锁 global_costmap_->mutex();
// CostmapClearCostmapRecovery::runRecovery 直接写 local master grid (无锁),
// 与 handleObstaclePointCloud (走 local wrapper mutex) 并发竞态 local master grid。
//
// 本测试: recovery 线程反复 start()+join() 清 robot 附近 (global+local);
// obstacle feed 线程每帧在 local wrapper 锁内 markObstacles + updateMap 写回
// local master grid (模拟 handleObstaclePointCloud)。修复前 (只锁 global) 两者
// 并发写同一 cell → feed 在锁内读回其刚写的 cell 会被 recovery 覆写为 FREE
// (data race / 撕裂); 修复后 scoped_lock(global, local) 串行化 → 锁内读回必为
// LETHAL, 0 违例, 且整体不死锁。

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <thread>

#include <Eigen/Geometry>

#include <costmap_2d/layered_costmap.hpp>
#include <costmap_2d_layers/simple_obstacle_layer.hpp>
#include <costmap_server/costmap_wrapper.hpp>

#include <ayanami/costmap/components/costmap_context.hpp>
#include <ayanami/costmap/components/costmap_recovery_execution.hpp>
#include <ayanami/interfaces/transform_store.hpp>
#include <ayanami/modules/costmap_clear_costmap_recovery/costmap_clear_costmap_recovery.hpp>

namespace {

using ayanami::costmap::CostmapContext;
using ayanami::modules::CostmapClearCostmapRecovery;

// 20x20 @0.5m, origin (0,0): cell (4,4) 中心 = 世界 (2.25, 2.25)。
constexpr double kRobotX = 2.25;
constexpr double kRobotY = 2.25;

std::shared_ptr<costmap_2d::CostmapWrapper> makeWrapper(
    const costmap_2d_layers::SimpleObstacleLayer::Ptr& layer = nullptr) {
  auto lm = std::make_shared<costmap_2d::LayeredCostmap>(
      "map", /*rolling_window=*/false, /*track_unknown=*/false);
  lm->resizeMap(20, 20, 0.5, 0.0, 0.0);
  if (layer) {
    lm->addLayer(layer);
  }
  return std::make_shared<costmap_2d::CostmapWrapper>(lm, "map");
}

void setRobot(double wx, double wy) {
  auto& store = ayanami::interfaces::TransformStore::instance();
  store.setTransform(
      "map", "base_link", ayanami::interfaces::TransformStore::fromSeconds(0.0),
      Eigen::Translation3d(wx, wy, 0.0) * Eigen::Quaterniond::Identity());
}

// std::recursive_mutex 无 try_lock_for (非 TimedLockable): 有限时长 try_lock
// 轮询, 保证 feed 线程在锁被整段占用时也能在时限内让出而非挂死。
bool lockWithTimeout(std::recursive_mutex& m,
                     std::chrono::milliseconds timeout) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (m.try_lock()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return false;
}

}  // namespace

TEST(RecoveryVsObstacle, NoDataRaceOnLocalMasterGrid) {
  // local costmap 挂 SimpleObstacleLayer (M1 obstacle feed 落图目标)。
  auto layer = std::make_shared<costmap_2d_layers::SimpleObstacleLayer>();
  auto global = makeWrapper();
  auto local = makeWrapper(layer);

  auto ctx = std::make_shared<CostmapContext>();
  ctx->global_costmap_ = global;
  ctx->local_costmap_ = local;
  ctx->transform_store_ = &ayanami::interfaces::TransformStore::instance();
  setRobot(kRobotX, kRobotY);

  auto recovery = std::make_shared<CostmapClearCostmapRecovery>();
  recovery->initialize("costmap_clear_costmap_recovery", ctx);

  ayanami::costmap::CostmapRecoveryExecution exec("recovery_vs_obstacle",
                                                   recovery, global, local);

  costmap_2d::Costmap2D* lg = local->get().getCostmap();

  std::atomic<bool> feeding{true};
  std::atomic<long> violations{0};
  std::atomic<long> feeds{0};

  // obstacle feed 线程: 模拟 handleObstaclePointCloud 走 local wrapper mutex,
  // 每帧 markObstacles + updateMap 把 (4,4) 写回 LETHAL。
  std::thread feeder([&] {
    while (feeding.load()) {
      if (lockWithTimeout(local->mutex(), std::chrono::milliseconds(100))) {
        layer->markObstacles(kRobotX, kRobotY);
        local->updateMap(kRobotX, kRobotY, 0.0);
        // 锁内宽观察窗: 修复前 recovery 无 local 锁, 可并发把 (4,4) 覆写为
        // FREE; 修复后 scoped_lock(global, local) 保证此处 read-back 必为
        // LETHAL (写入被串行化, 无撕裂)。
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        if (lg->getCost(4, 4) != costmap_2d::LETHAL_OBSTACLE) {
          ++violations;
        }
        ++feeds;
        local->mutex().unlock();
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  });

  // recovery 反复清 robot 附近 (global+local): 每轮 start()+join() = 一次
  // runRecovery (直接写 local master grid)。
  for (int i = 0; i < 300; ++i) {
    exec.start();
    exec.join();
  }

  feeding.store(false);
  feeder.join();

  EXPECT_GT(feeds.load(), 0);
  // 锁内 read-back 违例必须为 0: recovery 与 obstacle feed 对 local master grid
  // 的写入被 scoped_lock 串行化, 无竞态撕裂。
  EXPECT_EQ(violations.load(), 0);
}
