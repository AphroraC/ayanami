// 并发回归: controller 每 cycle 经 lockLocal() 取 local 读视图 (RAII shared
// 锁, 零拷贝直读 master grid), 与并发 obstacle 写者 (落障碍 + updateMapUnlocked
// 重算) 交错, 互不饿死。
//
// 旧架构 (BLOCK #1): CostmapControllerExecution::run() 整 control loop 持
// local wrapper recursive_mutex, 会饿死 obstacle feed。新架构单写者 + 多读者:
// - controller 每 cycle 经 lockLocal() 取 local shared 锁视图, 周期结束即释放;
// - obstacle 写者 (唯一写者) unique 锁内落障碍 + updateMapUnlocked 重算 master;
// - 读者短持锁, 写者等读者释放后写入, 双方均能推进 (无饿死 / 无死锁)。
//
// 断言: 双方都完成足够轮次 (无饿死 / 无死锁), 且 controller 每次 compute 都能
// 拿到有效锁视图 (无 MapUnavailable)。

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

#include <ayanami/abstract/core/abstract_goal_checker.hpp>
#include <ayanami/abstract/core/abstract_progress_checker.hpp>
#include <ayanami/costmap/components/costmap_controller_execution.hpp>
#include <ayanami/costmap/components/costmap_map_context.hpp>
#include <ayanami/abstract/components/robot_state_controller.hpp>
#include <ayanami/costmap/core/costmap_controller.hpp>
#include <ayanami/messages/failure.hpp>
#include <ayanami/types.hpp>

#include <shared_mutex>

namespace {

using ayanami::ErrorCode;
using ayanami::ControllerCommand;
using ayanami::ControllerResultT;
using ayanami::geometry_msgs::PoseStamped;
using ayanami::geometry_msgs::TwistStamped;
using ayanami::nav_msgs::Path;

// 加锁读视图 FakeController: 记录周期数 (写者不被读者饿死的并发验证)。
class SnapshotController : public ayanami::costmap::CostmapController {
 public:
  void configure(const ayanami::PluginContext&,
                 ayanami::costmap::CostmapMapContext::ConstPtr map) override {
    map_ = std::move(map);
  }
  ControllerResultT computeVelocityCommand(const PoseStamped&,
                                           const TwistStamped&) override {
    ++cycles;
    auto view = map_->lockLocal();
    if (!view.valid()) {
      ++snapshot_missing;
      return ControllerResultT::failure(ErrorCode::MAP_UNAVAILABLE,
                                        "no local view");
    }
    ControllerCommand cmd;
    cmd.velocity = TwistStamped{};
    cmd.velocity.twist.linear.x() = 0.5;
    return ControllerResultT::success(std::move(cmd));
  }

  std::atomic<int> cycles{0};
  std::atomic<int> snapshot_missing{0};

 private:
  ayanami::costmap::CostmapMapContext::ConstPtr map_;
  bool setPlan(const Path&) override { return true; }
  bool cancel() override { return true; }
};

// checker 测试桩: 永不到达 + 恒有进展。
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
  robot_state.setTransform("map", "base_link", ayanami::Timestamp::latest(),
                           Eigen::Isometry3d::Identity());
  auto local = makeLocalWrapper();
  auto ctx = std::make_shared<ayanami::costmap::CostmapMapContext>();
  ctx->setLocalCostmap(local);

  auto controller = std::make_shared<SnapshotController>();
  controller->configure(ayanami::PluginContext{"test_controller", ayanami::Config("")}, ctx);
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

  exec.start();

  // obstacle 写者线程: 唯一写者 —— 写锁内 markObservable 落障碍 + updateMap
  // 发布新 epoch。若 controller 曾持写锁整循环, 这里会被饿死 (feeds 长期不
  // 增长) / 与旧实现整循环持锁冲突。
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

  // 跑 ~1.2s: @20Hz → ~22 controller cycle; 每 5ms → ~240 次 feed。
  std::this_thread::sleep_for(std::chrono::milliseconds(1200));

  feeding.store(false);
  feeder.join();

  exec.cancel();
  exec.join();

  // 防饿死: controller 完成 ≥15 cycle 且每次都拿到有效视图 (无 MapUnavailable)。
  EXPECT_GE(controller->cycles.load(), 15);
  EXPECT_EQ(controller->snapshot_missing.load(), 0)
      << "controller 每 cycle 都应拿到有效视图";
  // obstacle 写者同样完成 ≥15 次 (读者 shared 锁不阻塞写者以外的读者;
  // 写者等读者释放, 双方都能推进)。
  EXPECT_GE(obstacle_feeds.load(), 15);
}
