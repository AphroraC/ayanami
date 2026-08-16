// Task 1 (M2 阶段 A BLOCK #1): controller 整循环持锁 → per-cycle short lock。
//
// 回归对象: M1 CostmapControllerExecution::run() 整 control loop 持
// local_costmap_->mutex() (recursive_mutex)。M2 接 ROS 障碍回调
// (handleObstaclePointCloud 走同一 local 锁) 会饿死 controller / 阻断 spinOnce。
//
// 本测试: controller 线程 @20Hz 每 cycle 短持锁 compute; 另一线程每 5ms 投
// obstacle feed (模拟 handleObstaclePointCloud 走 local wrapper mutex)。
// 断言双方都未被饿死 (≥15/run), 且测试整体快速完成 (不死锁)。

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <costmap_2d/costmap_2d.hpp>
#include <costmap_2d/layered_costmap.hpp>
#include <costmap_server/costmap_wrapper.hpp>

#include <ayanami/costmap/components/costmap_controller_execution.hpp>
#include <ayanami/costmap/components/costmap_context.hpp>

namespace {

using ayanami::geometry_msgs::PoseStamped;
using ayanami::geometry_msgs::TwistStamped;
using ayanami::nav_msgs::Path;

// FakeController: 模拟真实 costmap 控制器 —— computeVelocityCommands 在临界段
// (controller 持 local wrapper 锁内) sleep 5ms 模拟工作 (M2-P5 裁决)。
class FakeController : public ayanami::abstract::AbstractController {
 public:
  uint32_t computeVelocityCommands(const PoseStamped&, const TwistStamped&,
                                   TwistStamped& cmd_vel, std::string&) override {
    ++cycles_;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    cmd_vel.twist.linear.x() = 0.5;
    return 0;
  }
  bool isGoalReached(double, double) override { return false; }
  bool setPlan(const Path&) override { return true; }
  bool cancel() override { return true; }
  std::atomic<int> cycles_{0};
};

std::shared_ptr<costmap_2d::CostmapWrapper> makeLocalWrapper() {
  auto lm = std::make_shared<costmap_2d::LayeredCostmap>(
      "map", /*rolling_window=*/true, /*track_unknown=*/false);
  lm->resizeMap(10, 10, 0.5, 0.0, 0.0);
  return std::make_shared<costmap_2d::CostmapWrapper>(lm, "map");
}

// std::recursive_mutex 无 try_lock_for (非 TimedLockable): 用有限时长的
// try_lock 轮询模拟, 保证测试在锁被整循环占用时也能在时限内完成而非挂死。
bool lockWithTimeout(std::recursive_mutex& m, std::chrono::milliseconds timeout) {
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

TEST(ConcurrentControllerObstacle, NoDeadlockNoStarvation) {
  auto local = makeLocalWrapper();
  auto ctx = std::make_shared<ayanami::costmap::CostmapContext>();
  ctx->local_costmap_ = local;

  auto controller = std::make_shared<FakeController>();
  ayanami::costmap::CostmapControllerExecution exec("test_controller", controller,
                                                    ctx->local_costmap_);

  Path plan;
  plan.poses.resize(4);
  exec.setNewPlan(plan);
  exec.setControllerFrequency(20.0);  // ~50ms/cycle

  std::atomic<bool> feeding{true};
  std::atomic<int> obstacle_feeds{0};

  exec.start();

  // obstacle feed 线程: 每 5ms 投一次 (模拟 handleObstaclePointCloud 走 local
  // wrapper mutex)。用有限时长 try_lock 轮询 —— 旧实现整循环持锁时只会记录
  // 超时而非挂死, 保证测试本身不死锁、饿死可观测。
  std::thread feeder([&] {
    while (feeding.load()) {
      if (lockWithTimeout(local->mutex(), std::chrono::milliseconds(100))) {
        ++obstacle_feeds;  // 模拟锁内 markObstacles 短工作
        local->mutex().unlock();
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  });

  // 跑 ~1.2s: @20Hz → ~22 controller cycle; 每 5ms → ~240 次 feed 尝试。
  std::this_thread::sleep_for(std::chrono::milliseconds(1200));

  feeding.store(false);
  feeder.join();
  exec.cancel();
  exec.join();

  // 防饿死: controller 完成 ≥75% 预期 cycle; obstacle feed ≥15 次成功落锁。
  EXPECT_GE(controller->cycles_.load(), 15);
  EXPECT_GE(obstacle_feeds.load(), 15);
}
