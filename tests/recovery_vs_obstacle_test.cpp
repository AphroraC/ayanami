// 恢复 vs 障碍并发 (地图变更经唯一写者命令端口)。
//
// 新架构 (加锁直读 CostmapView, 无快照/epoch): 恢复插件不接触可变
// wrapper/layer; clear recovery 只构造类型化 ClearCostmapCommand 并经
// CostmapCommandPort 提交, 唯一写者 (这里模拟等价 writer) 持 wrapper 的
// unique 锁清层 + updateMapUnlocked 发布。读者经 shared 锁直读 master grid。
//
// 断言:
// - clear-command-port 路径真实生效: 提交后 master grid 机器人附近 → FREE,
// 远处保持 LETHAL;
// - 与 obstacle 写者交替多轮, 读写方各持锁 (unique/shared), 每步锁内读回
// 确定性值 (无撕裂 / 无死锁); 命令提交计数随轮次严格递增 (计数代数)。

#include <gtest/gtest.h>

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Geometry>

#include <costmap_2d/layered_costmap.hpp>
#include <costmap_2d_layers/simple_obstacle_layer.hpp>
#include <costmap_server/costmap_wrapper.hpp>

#include <ayanami/abstract/components/robot_state_controller.hpp>
#include <ayanami/costmap/components/costmap_commands.hpp>
#include <ayanami/costmap/components/costmap_map_context.hpp>
#include <ayanami/costmap/core/costmap_command.hpp>
#include <ayanami/costmap/core/costmap_recovery.hpp>
#include <ayanami/messages/failure.hpp>
#include <ayanami/types.hpp>
#include <costmap_clear_costmap_recovery/costmap_clear_costmap_recovery.hpp>

namespace {

using ayanami::ErrorCode;
using ayanami::RecoveryResultT;
using ayanami::costmap::ClearCostmapCommand;
using ayanami::costmap::CostmapCommandPort;
using ayanami::costmap::CostmapMapContext;
using ayanami::modules::CostmapClearCostmapRecovery;

// 20x20 @0.5m, origin (0,0): cell (4,4) 中心 = 世界 (2.25, 2.25)。
constexpr double k_robot_x = 2.25;
constexpr double k_robot_y = 2.25;

// 等价写者命令端口: submit 即执行 writer 级清图 (clearCostmapArea 在写锁内
// 清层 + updateMapUnlocked 发布)。恢复插件只持有该接口, 不接触 wrapper。
class TestWriterPort : public CostmapCommandPort {
 public:
  TestWriterPort(const std::shared_ptr<costmap_2d::CostmapWrapper>& global,
                 const std::shared_ptr<costmap_2d::CostmapWrapper>& local,
                 ayanami::components::RobotStateController* rsc,
                 const std::string& robot_frame)
      : global(global),
        local(local),
        rsc(rsc),
        robot_frame(robot_frame) {}

  bool submit(const ClearCostmapCommand& command) override {
    ++submissions;
    if (!command.enabled) {
      return true;
    }
    const bool clear_global =
        command.affected_maps == "global" || command.affected_maps == "both";
    const bool clear_local =
        command.affected_maps == "local" || command.affected_maps == "both";
    bool ok = true;
    if (clear_global && global) {
      Eigen::Isometry3d p = Eigen::Isometry3d::Identity();
      ok = rsc->lookupTransform(global->getGlobalFrame(), robot_frame,
                                ayanami::Timestamp::latest(), p) &&
           ok;
      ok = ayanami::costmap::clearCostmapArea(global, p, command.clear_radius) &&
           ok;
    }
    if (clear_local && local) {
      Eigen::Isometry3d p = Eigen::Isometry3d::Identity();
      ok = rsc->lookupTransform(local->getGlobalFrame(), robot_frame,
                                ayanami::Timestamp::latest(), p) &&
           ok;
      ok = ayanami::costmap::clearCostmapArea(local, p, command.clear_radius) &&
           ok;
    }
    return ok;
  }

  // 写锁 (writer submit) 完成时递增的计数: 表达"版本/epoch"单调"语义。
  std::atomic<int> submissions{0};

 private:
  std::shared_ptr<costmap_2d::CostmapWrapper> global;
  std::shared_ptr<costmap_2d::CostmapWrapper> local;
  ayanami::components::RobotStateController* rsc;
  std::string robot_frame;
};

// 同步门 (仅测试): set 唤醒全部等待者。
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
  auto& rsc = ayanami::components::RobotStateController::instance();
  rsc.setTransform(
      "map", "base_link", ayanami::Timestamp::latest(),
      Eigen::Translation3d(wx, wy, 0.0) * Eigen::Quaterniond::Identity());
}

// 唯一写者路径: unique 锁内 mark 动态障碍层 + updateMapUnlocked 重算发布。
void feedObstacles(
    const std::shared_ptr<costmap_2d::CostmapWrapper>& wrapper,
    const std::shared_ptr<costmap_2d_layers::SimpleObstacleLayer>& layer) {
  std::unique_lock<std::shared_mutex> lock(wrapper->mutex());
  layer->markObstacles(k_robot_x, k_robot_y);
  layer->markObstacles(7.75, 7.75);
  wrapper->updateMapUnlocked(k_robot_x, k_robot_y, 0.0);
}

// 读者路径: shared 锁内直读 master grid 单个 cell。
unsigned char cellCost(
    const std::shared_ptr<costmap_2d::CostmapWrapper>& wrapper, unsigned int x,
    unsigned int y) {
  std::shared_lock<std::shared_mutex> lock(wrapper->mutex());
  return wrapper->get().getCostmap()->getCost(x, y);
}

}  // namespace

// command port 真实生效: 提交 clear 命令后, 机器人附近清回 FREE, 远处保持
// LETHAL; 且只允许 writer 触发变更 (恢复插件无任何可变访问权)。
TEST(RecoveryVsObstacle, clearCommandViaPortOnlyWriterMutates) {
  auto global_layer = std::make_shared<costmap_2d_layers::SimpleObstacleLayer>();
  auto local_layer = std::make_shared<costmap_2d_layers::SimpleObstacleLayer>();
  auto global = makeWrapper(global_layer);
  auto local = makeWrapper(local_layer);

  auto ctx = std::make_shared<CostmapMapContext>();
  ctx->setGlobalCostmap(global);
  ctx->setLocalCostmap(local);
  setRobot(k_robot_x, k_robot_y);

  auto port = std::make_shared<TestWriterPort>(
      global, local, &ayanami::components::RobotStateController::instance(),
      ctx->getRobotFrame());

  // 命令端口经地图上下文暴露 (configure 内 map->commandPort 获取)。
  ctx->setCommandPort(port);
  auto rec = std::make_shared<CostmapClearCostmapRecovery>();
  rec->configure(ayanami::PluginContext{"costmap_clear_costmap_recovery",
                                        ayanami::Config("")},
                 ctx->getSharedConstPtr());

  // 先经写者落障碍 (feed): 近处与远处都 LETHAL。
  feedObstacles(local, local_layer);
  feedObstacles(global, global_layer);
  EXPECT_EQ(cellCost(local, 4, 4), costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(cellCost(local, 15, 15), costmap_2d::LETHAL_OBSTACLE);

  // 恢复: 经命令端口提交 typed clear 命令 (仅经命令端口), 不清除远处障碍。
  rec->start();
  const RecoveryResultT outcome = rec->tick();
  ASSERT_TRUE(outcome.ok) << outcome.message;
  EXPECT_EQ(port->submissions.load(), 1)
      << "clear 必须经命令端口提交给唯一写者执行";

  // 本地 + 全局: 机器人附近 → FREE, 远处保持 LETHAL。
  EXPECT_EQ(cellCost(local, 4, 4), costmap_2d::FREE_SPACE);
  EXPECT_EQ(cellCost(local, 5, 5), costmap_2d::FREE_SPACE);
  EXPECT_EQ(cellCost(local, 15, 15), costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(cellCost(global, 4, 4), costmap_2d::FREE_SPACE);
  EXPECT_EQ(cellCost(global, 15, 15), costmap_2d::LETHAL_OBSTACLE);
}

// 干净的确定性交替 (clear 命令 vs obstacle feed) 多轮: 每轮 feed 完成后读回
// LETHAL, clear 后读回 FREE —— 写全经唯一写者 (unique 锁), 读写方各持锁,
// 无撕裂/无死锁; 命令提交计数随轮次递增。
TEST(RecoveryVsObstacle, clearCommandAlternatesWithObstacleWithoutDeadlock) {
  auto global_layer = std::make_shared<costmap_2d_layers::SimpleObstacleLayer>();
  auto local_layer = std::make_shared<costmap_2d_layers::SimpleObstacleLayer>();
  auto global = makeWrapper(global_layer);
  auto local = makeWrapper(local_layer);

  auto ctx = std::make_shared<CostmapMapContext>();
  ctx->setGlobalCostmap(global);
  ctx->setLocalCostmap(local);
  setRobot(k_robot_x, k_robot_y);

  auto port = std::make_shared<TestWriterPort>(
      global, local, &ayanami::components::RobotStateController::instance(),
      ctx->getRobotFrame());

  // 命令端口经地图上下文暴露 (configure 内 map->commandPort 获取)。
  ctx->setCommandPort(port);
  auto rec = std::make_shared<CostmapClearCostmapRecovery>();
  rec->configure(ayanami::PluginContext{"costmap_clear_costmap_recovery",
                                        ayanami::Config("")},
                 ctx->getSharedConstPtr());

  constexpr int k_rounds = 16;
  std::atomic<int> bad_reads{0};
  std::atomic<int> bad_after_clear{0};
  std::atomic<int> rounds_done{0};

  std::vector<Gate> go_feed(k_rounds);
  std::vector<Gate> feed_done(k_rounds);

  // obstacle 写者线程: 每轮经写者 (unique 锁) 落障碍 + 发布, 完成后回报。
  std::thread feeder([&] {
    for (int i = 0; i < k_rounds; ++i) {
      go_feed[i].waitFor(std::chrono::seconds(5));
      {
        std::unique_lock<std::shared_mutex> lock(local->mutex());
        local_layer->markObstacles(k_robot_x, k_robot_y);
        local->updateMapUnlocked(k_robot_x, k_robot_y, 0.0);
        // 锁内读回必须看到自己的写入 (writer 串行, 无撕裂)。
        if (local->get().getCostmap()->getCost(4, 4) !=
            costmap_2d::LETHAL_OBSTACLE) {
          ++bad_reads;
        }
      }
      feed_done[i].set();
    }
  });

  for (int i = 0; i < k_rounds; ++i) {
    go_feed[i].set();
    ASSERT_TRUE(feed_done[i].waitFor(std::chrono::seconds(5)));
    if (cellCost(local, 4, 4) != costmap_2d::LETHAL_OBSTACLE) {
      ++bad_reads;
    }
    // 恢复: 经命令端口提交 typed clear → 唯一写者清回 FREE (start 复位
    // submitted, 每轮重新提交一次命令)。
    rec->start();
    const RecoveryResultT outcome = rec->tick();
    ASSERT_TRUE(outcome.ok) << outcome.message;
    if (cellCost(local, 4, 4) != costmap_2d::FREE_SPACE) {
      ++bad_after_clear;
    }
    ++rounds_done;
  }

  feeder.join();
  EXPECT_EQ(rounds_done.load(), k_rounds);
  EXPECT_EQ(port->submissions.load(), k_rounds)
      << "clear 提交计数必须随轮次严格递增 (命令全经唯一写者)";
  EXPECT_EQ(bad_reads.load(), 0)
      << "feed 写者锁内读回被并发 clear 撕裂 (writer 串行化被破坏)";
  EXPECT_EQ(bad_after_clear.load(), 0) << "clear 后 cell 未被写回 LETHAL";
  // 无死锁: 双线程均完成 (写者线程 join 返回 + 主线程 k_rounds 轮全部完成)。
}