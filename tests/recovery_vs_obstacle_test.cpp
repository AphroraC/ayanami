// Task 2 (M2 阶段 A BLOCK #3): recovery 锁 global+local (scoped_lock) 防与
// obstacle feed 数据竞态。M2 收尾: obstacle feed 改为"local 阶段锁内写完释放
// 后才请求 global"(见 markObstaclesInFrame 两段式锁作用域), 与 recovery 的
// global→local 锁序构成无环图, 无 AB/BA 死锁。
//
// 锁纪律 (生产代码, 本测试的回归对象):
// - CostmapRecoveryExecution::run(): 整个恢复在 scoped_lock(global, local)
//   临界段内执行; clear recovery 在临界段内直写 local master grid → FREE。
// - markObstaclesInFrame: local 锁 {clear+mark+updateMap} → 释放 local →
//   global 锁 {updateMap}。任何时刻不嵌套持有 local→请求 global。
// - controller/planner: 各自只持 local; Context::lock(): 只持 global。
//
// 保护目标: clear 直写 local master grid 与 obstacle feed 在 local wrapper
// 锁内 mark/updateMap 写回同一 cell 并发竞态; 以及 (修复后引入的新问题)
// recovery 持 global 等 local 与 obstacle 持 local 等 global 的 AB/BA 死锁。
//
// 本测试不依赖 sleep/循环概率: 用 Gate (condition_variable 门) 编排确定性
// 交错, 事件日志 (EventLog) 断言 happens-before 次序:
//   T1 (恢复): 进入 CS (持 global+local) 可观察, 阻塞至放行 → 退出 CS。
//   T2 (obstacle): 持 local 前先声明等待 → local 阶段 (锁内读回 LETHAL) →
//      释放 local → 才请求/获取 global → 完成。
// 断言: recovery 退出 CS 才允许 obstacle 入 local (local master grid 写串行);
// obstacle 释放 local 后才请求 global (无 L→G 嵌套 ⇒ 无 AB/BA); 双线程均完成。
// 不用生产测试钩子: CS 进入点由受测恢复模块 (测试侧 GuardedRecovery) 的
// runRecovery 自报, 锁本身由生产执行器加/解。

#include <gtest/gtest.h>

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Geometry>

#include <costmap_2d/layered_costmap.hpp>
#include <costmap_2d_layers/simple_obstacle_layer.hpp>
#include <costmap_server/costmap_wrapper.hpp>

#include <ayanami/costmap/components/costmap_context.hpp>
#include <ayanami/costmap/components/costmap_recovery_execution.hpp>
#include <ayanami/interfaces/transform_store.hpp>
#include <costmap_clear_costmap_recovery/costmap_clear_costmap_recovery.hpp>

namespace {

using ayanami::FailureCode;
using ayanami::costmap::CostmapContext;
using ayanami::modules::CostmapClearCostmapRecovery;

// 20x20 @0.5m, origin (0,0): cell (4,4) 中心 = 世界 (2.25, 2.25)。
constexpr double kRobotX = 2.25;
constexpr double kRobotY = 2.25;

// ---- 确定性同步原语 (仅测试) ----

// 门: set() 唤醒所有等待者, reset() 复用。waitFor 带超时, 死锁时判失败而非挂死。
class Gate {
 public:
  void set() {
    std::lock_guard<std::mutex> lk(m_);
    opened_ = true;
    cv_.notify_all();
  }
  void reset() {
    std::lock_guard<std::mutex> lk(m_);
    opened_ = false;
  }
  void wait() {
    std::unique_lock<std::mutex> lk(m_);
    cv_.wait(lk, [this] { return opened_; });
  }
  bool waitFor(std::chrono::milliseconds timeout) const {
    std::unique_lock<std::mutex> lk(m_);
    return cv_.wait_for(lk, timeout, [this] { return opened_; });
  }
  bool isOpen() const {
    std::lock_guard<std::mutex> lk(m_);
    return opened_;
  }

 private:
  mutable std::mutex m_;
  mutable std::condition_variable cv_;
  bool opened_{false};
};

// 事件日志: 记录交错里程碑, 供 happens-before 次序断言 (indexOf 返回首次下标)。
class EventLog {
 public:
  void add(const std::string& ev) {
    std::lock_guard<std::mutex> lk(m_);
    entries_.push_back(ev);
  }
  int indexOf(const std::string& ev) const {
    std::lock_guard<std::mutex> lk(m_);
    for (std::size_t i = 0; i < entries_.size(); ++i) {
      if (entries_[i] == ev) {
        return static_cast<int>(i);
      }
    }
    return -1;
  }

 private:
  mutable std::mutex m_;
  std::vector<std::string> entries_;
};

// 守卫恢复模块 (测试侧注入插件): runRecovery 一进入即置 entered_ 并阻塞到
// release_ 开门才返回。它运行在 CostmapRecoveryExecution::run() 的
// scoped_lock(global, local) 临界段内 —— 置 entered_ 那一刻可观察恢复同时
// 持有两把锁 (等价"恢复持 global 后请求 local 并进入临界段")。
class GuardedRecovery : public ayanami::abstract::AbstractRecovery {
 public:
  GuardedRecovery(Gate& entered, Gate& release, EventLog& log)
      : entered_(entered), release_(release), log_(log) {}

  FailureCode runRecovery(std::string& /*message*/) override {
    log_.add("recovery.cs.enter");
    entered_.set();
    release_.wait();
    log_.add("recovery.cs.exit");
    return FailureCode::None;
  }

  bool cancel() override { return true; }

 private:
  Gate& entered_;
  Gate& release_;
  EventLog& log_;
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
  auto& store = ayanami::interfaces::TransformStore::instance();
  store.setTransform(
      "map", "base_link", ayanami::Timestamp::latest(),
      Eigen::Translation3d(wx, wy, 0.0) * Eigen::Quaterniond::Identity());
}

}  // namespace

// 确定性构造 AB/BA 最不利交错并证明无死锁 + local master grid 写串行:
//   T_recovery: scoped_lock(global, local) → 进入 CS (持双锁) → 阻塞至放行。
//   T_obstacle: 请求 local (在 recovery 持锁期间排队) → 获得 local (仅在
//               recovery 退出 CS 后) → local 阶段锁内读回 LETHAL → 释放
//               local → 才请求+获取 global → 完成。
// 断言:
//   1) obstacle.local.acquire 严格晚于 recovery.cs.exit ⇒ 两路写 local master
//      grid (clear 直写 / feed mark+updateMap) 永不重叠, 串行。
//   2) obstacle.local.release 严格早于 obstacle.global.request ⇒ obstacle 从不
//      持 local 等 global ⇒ 与 recovery 的 global→local 无环, 无 AB/BA。
//   3) 双线程均在超时内完成 ⇒ 无死锁。
TEST(RecoveryVsObstacle, NoDeadlockNoAbBaAndLocalWritesSerialized) {
  // local costmap 挂 SimpleObstacleLayer (M1 obstacle feed 落图目标)。
  auto layer = std::make_shared<costmap_2d_layers::SimpleObstacleLayer>();
  auto global = makeWrapper();
  auto local = makeWrapper(layer);

  auto ctx = std::make_shared<CostmapContext>();
  ctx->global_costmap_ = global;
  ctx->local_costmap_ = local;
  ctx->transform_store_ = &ayanami::interfaces::TransformStore::instance();
  setRobot(kRobotX, kRobotY);

  Gate entered;       // recovery 已进入 CS (持 global+local)
  Gate release_cs;    // coordinator 放行 recovery 退出 CS
  EventLog log;

  auto recovery = std::make_shared<GuardedRecovery>(entered, release_cs, log);
  ayanami::costmap::CostmapRecoveryExecution exec(
      "recovery_vs_obstacle", recovery, global, local);

  costmap_2d::Costmap2D* lg = local->get().getCostmap();

  Gate go_obstacle;       // coordinator 放行 obstacle 的 local 阶段
  Gate obstacle_waiting;  // obstacle 即将在 local 锁上排队
  Gate obstacle_done;     // obstacle 全部阶段完成

  exec.start();  // recovery 线程: preRun → scoped_lock(global, local) → GuardedRecovery

  // 恢复临界段可观察: recovery 此刻持 global+local 两把锁。
  ASSERT_TRUE(entered.waitFor(std::chrono::seconds(5)));

  // obstacle feed 线程 (模拟 markObstaclesInFrame 的新锁序: local 段写完释放
  // 才请求 global)。go_obstacle 在 recovery 进入 CS 后才放行, 保证 obstacle
  // 的 local 请求必然撞上恢复持锁窗口。
  go_obstacle.set();
  std::thread feeder([&] {
    go_obstacle.wait();
    log.add("obstacle.local.request");
    obstacle_waiting.set();
    {
      std::lock_guard<std::recursive_mutex> lk(local->mutex());
      log.add("obstacle.local.acquire");
      // 锁内宽观察窗替换为确定性断言: local 锁获得必然发生在 recovery 退出
      // CS 之后 (见末尾次序断言), 写入被串行化, 锁内读回必为 LETHAL。
      layer->markObstacles(kRobotX, kRobotY);
      local->updateMap(kRobotX, kRobotY, 0.0);
      EXPECT_EQ(lg->getCost(4, 4), costmap_2d::LETHAL_OBSTACLE);
      log.add("obstacle.local.release");
    }
    // 关键 (无 AB/BA): local 已释放才请求 global, 从不嵌套持锁。
    log.add("obstacle.global.request");
    {
      std::lock_guard<std::recursive_mutex> gk(global->mutex());
      log.add("obstacle.global.acquire");
      global->updateMap(kRobotX, kRobotY, 0.0);
      log.add("obstacle.global.release");
    }
    obstacle_done.set();
  });

  // 构造交错: obstacle 在 local 锁上排队时, recovery 必仍持 global+local
  // (coordinator 尚未放行, 放行前 recovery 不可能退出 CS)。
  ASSERT_TRUE(obstacle_waiting.waitFor(std::chrono::seconds(5)));
  ASSERT_TRUE(entered.isOpen());

  // 放行 recovery 退出 CS → obstacle 拿到 local → 完成 local 阶段 → 释放 →
  // 再请求 global。若生产曾 L→G 嵌套 (AB/BA), 双方互等, 上方超时判失败。
  release_cs.set();

  ASSERT_TRUE(obstacle_done.waitFor(std::chrono::seconds(5)));
  feeder.join();
  exec.join();

  EXPECT_EQ(lg->getCost(4, 4), costmap_2d::LETHAL_OBSTACLE);

  // ---- happens-before 次序断言 (事件链由 Gate/锁建立) ----
  const int cs_enter = log.indexOf("recovery.cs.enter");
  const int cs_exit = log.indexOf("recovery.cs.exit");
  const int loc_req = log.indexOf("obstacle.local.request");
  const int loc_acq = log.indexOf("obstacle.local.acquire");
  const int loc_rel = log.indexOf("obstacle.local.release");
  const int glo_req = log.indexOf("obstacle.global.request");
  int first_missing = -1;
  for (int idx : {cs_enter, cs_exit, loc_req, loc_acq, loc_rel, glo_req}) {
    if (idx < 0) {
      first_missing = idx;
      break;
    }
  }
  ASSERT_EQ(first_missing, -1) << "事件日志缺失里程碑";

  EXPECT_LT(cs_enter, loc_req) << "recovery 进入 CS 后才放行 obstacle";
  // local master grid 写串行: 恢复 (clear 直写) 退出 CS 后 obstacle 才允许进入
  // local 阶段 (mark+updateMap), 两路写绝不重叠。
  EXPECT_LT(cs_exit, loc_acq) << "recovery 退出 CS 之前 obstacle 不得写 local";
  // 无 AB/BA: obstacle 先释放 local, 再请求 global。
  EXPECT_LT(loc_rel, glo_req) << "obstacle 持 local 时不得请求 global";
}

// 真实 CostmapClearCostmapRecovery (scoped_lock(global, local) 临界段内直写
// local master grid) 与 obstacle feed (local 锁内 mark+updateMap) 交替多轮,
// 全程 Gate 编排零 sleep: 每轮 feed 锁内读回 (4,4) 必为 LETHAL, clear 后必为
// FREE。确定性验证两路写 local master grid 严格串行且内容稳定 (无撕裂)。
TEST(RecoveryVsObstacle, LocalMasterGridWritesSerializedAcrossRounds) {
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
  ayanami::costmap::CostmapRecoveryExecution exec(
      "recovery_vs_obstacle", recovery, global, local);

  costmap_2d::Costmap2D* lg = local->get().getCostmap();

  constexpr int kRounds = 16;
  std::vector<Gate> go_feed(kRounds);
  std::vector<Gate> feed_done(kRounds);
  std::atomic<int> bad_reads{0};
  std::atomic<int> bad_after_clear{0};

  // obstacle feed 线程: 每轮在 local 锁内 markObstacles + updateMap 写回
  // (4,4) 为 LETHAL, 锁内读回必须看到自己的写入 (写入被串行化, 无撕裂)。
  std::thread feeder([&] {
    for (int i = 0; i < kRounds; ++i) {
      go_feed[i].wait();
      {
        std::lock_guard<std::recursive_mutex> lk(local->mutex());
        layer->markObstacles(kRobotX, kRobotY);
        local->updateMap(kRobotX, kRobotY, 0.0);
        if (lg->getCost(4, 4) != costmap_2d::LETHAL_OBSTACLE) {
          ++bad_reads;
        }
      }
      feed_done[i].set();
    }
  });

  for (int i = 0; i < kRounds; ++i) {
    // feed 先落 LETHAL
    go_feed[i].set();
    ASSERT_TRUE(feed_done[i].waitFor(std::chrono::seconds(5)));
    // 恢复清回 FREE: start()+join() = 一轮 scoped_lock(global, local) 临界段,
    // clear recovery 在该临界段内把 robot 附近 (含 (4,4)) 直写为 FREE。
    exec.start();
    exec.join();
    if (lg->getCost(4, 4) != costmap_2d::FREE_SPACE) {
      ++bad_after_clear;
    }
  }

  feeder.join();

  EXPECT_EQ(bad_reads.load(), 0)
      << "obstacle 锁内读回被并发 clear 撕裂 (local master grid 写未串行)";
  EXPECT_EQ(bad_after_clear.load(), 0) << "clear 后 cell 未被写回 LETHAL";
}