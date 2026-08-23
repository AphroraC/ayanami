// Task 24: modules/costmap_*_recovery (CostmapRecovery 族 4 个恢复插件)。
//
// 覆盖:
// - CostmapSpinRecovery::runRecovery 完成所设 spin_dist → None,
//   期间经 on_velocity_command 发纯角速度, 结束发零速度
// - spin 中途 cancel() → FailureCode::Canceled 且末帧零速度
// - CostmapBackUpRecovery 倒车检测碰撞 (global costmap lethal) → RecoveryFailed,
//   message 含 "collision detected"
// - CostmapWaitRecovery wait_duration 后 → None; 中途 cancel 立即 Canceled
// - CostmapClearCostmapRecovery 清除 global+local 内 robot 附近 radius → None,
//   master grid 对应区域 = FREE_SPACE(0), 远处保持 lethal
// - 4 个 module.cpp 统一导出 create_costmap_recovery_module (dlopen 验证正确类)

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Geometry>

#include <costmap_2d/costmap_2d.hpp>
#include <costmap_2d/layered_costmap.hpp>
#include <costmap_server/costmap_wrapper.hpp>

#include <ayanami/costmap/components/costmap_context.hpp>
#include <ayanami/costmap/core/costmap_recovery.hpp>
#include <ayanami/interfaces/callbacks.hpp>
#include <ayanami/interfaces/transform_store.hpp>
#include <ayanami/messages/failure.hpp>
#include <ayanami/messages/geometry_msgs.hpp>
#include <costmap_back_up_recovery/costmap_back_up_recovery.hpp>
#include <costmap_clear_costmap_recovery/costmap_clear_costmap_recovery.hpp>
#include <costmap_spin_recovery/costmap_spin_recovery.hpp>
#include <costmap_wait_recovery/costmap_wait_recovery.hpp>
#include <ayanami/utility/config.hpp>
#include <ayanami/utility/load_module.hpp>
#include <costmap_2d_layers/simple_obstacle_layer.hpp>

namespace {

using ayanami::FailureCode;
using ayanami::costmap::CostmapContext;
using ayanami::geometry_msgs::TwistStamped;
using ayanami::modules::CostmapBackUpRecovery;
using ayanami::modules::CostmapClearCostmapRecovery;
using ayanami::modules::CostmapSpinRecovery;
using ayanami::modules::CostmapWaitRecovery;

// 4 个模块 .so (argv 注入, 顺序: spin/back_up/wait/clear_costmap)。
std::string g_spin_so;
std::string g_backup_so;
std::string g_wait_so;
std::string g_clear_so;

// 捕捉 spin/back_up emit 的 on_velocity_command。
std::vector<TwistStamped> g_emitted;
std::mutex g_emit_mutex;
int g_emit_cb_id = -1;

// ---------------- 测试工具 ----------------

// 建一个 NxN @ res, origin=(0,0) 的 master grid (无 layer, 直接 setCost)。
std::shared_ptr<costmap_2d::CostmapWrapper> makeWrapper(unsigned int size,
                                                        double res) {
  auto lm = std::make_shared<costmap_2d::LayeredCostmap>(
      "map", /*rolling_window=*/false, /*track_unknown=*/false);
  lm->resizeMap(size, size, res, 0.0, 0.0);
  return std::make_shared<costmap_2d::CostmapWrapper>(lm, "map");
}

std::shared_ptr<CostmapContext> makeContext(
    const std::shared_ptr<costmap_2d::CostmapWrapper>& global,
    const std::shared_ptr<costmap_2d::CostmapWrapper>& local = nullptr) {
  auto ctx = std::make_shared<CostmapContext>();
  ctx->global_costmap_ = global;
  ctx->local_costmap_ = local;
  return ctx;
}

// 设置 TransformStore: map←base_link, 机器人位于世界 (wx, wy)。
void setRobot(double wx, double wy) {
  auto& store = ayanami::interfaces::TransformStore::instance();
  store.setTransform(
      "map", "base_link", ayanami::Timestamp::latest(),
      Eigen::Translation3d(wx, wy, 0.0) * Eigen::Quaterniond::Identity());
}

void attachVelocityCapture() {
  g_emitted.clear();
  g_emit_cb_id =
      ayanami::interfaces::NavigationCallbacks::on_velocity_command.add(
          [](const TwistStamped& t) {
            std::lock_guard<std::mutex> lock(g_emit_mutex);
            g_emitted.push_back(t);
          });
}

void detachVelocityCapture() {
  if (g_emit_cb_id >= 0) {
    ayanami::interfaces::NavigationCallbacks::on_velocity_command.remove(
        g_emit_cb_id);
    g_emit_cb_id = -1;
  }
}

TwistStamped lastEmitted() {
  std::lock_guard<std::mutex> lock(g_emit_mutex);
  return g_emitted.empty() ? TwistStamped{} : g_emitted.back();
}

bool anyNonZeroAngular() {
  std::lock_guard<std::mutex> lock(g_emit_mutex);
  for (const auto& t : g_emitted) {
    if (std::abs(t.twist.angular.z()) > 1e-9) {
      return true;
    }
  }
  return false;
}

}  // namespace

// ---------- CostmapSpinRecovery ----------

TEST(CostmapRecoveryModulesTest, SpinCompletesWithSuccess) {
  attachVelocityCapture();
  auto rec = std::make_shared<CostmapSpinRecovery>();
  rec->initialize("costmap_spin_recovery", makeContext(makeWrapper(20, 0.5)));

  std::string message;
  const FailureCode failure = rec->runRecovery(message);
  EXPECT_EQ(failure, FailureCode::None) << message;

  EXPECT_TRUE(anyNonZeroAngular());           // 发过纯角速度
  EXPECT_NEAR(lastEmitted().twist.angular.z(), 0.0, 1e-9);  // 结束零速度
  detachVelocityCapture();
}

TEST(CostmapRecoveryModulesTest, SpinCancelEmitsZeroAndCanceled) {
  attachVelocityCapture();
  auto rec = std::make_shared<CostmapSpinRecovery>();
  rec->initialize("costmap_spin_recovery", makeContext(makeWrapper(20, 0.5)));

  std::atomic<bool> done{false};
  std::atomic<FailureCode> failure{FailureCode::None};
  std::string message;
  std::thread worker([&] {
    failure.store(rec->runRecovery(message));
    done.store(true);
  });
  std::this_thread::sleep_for(std::chrono::milliseconds(80));
  EXPECT_TRUE(rec->cancel());
  worker.join();
  EXPECT_TRUE(done.load());
  EXPECT_EQ(failure.load(), FailureCode::Canceled);
  EXPECT_NEAR(lastEmitted().twist.angular.z(), 0.0, 1e-9);
  detachVelocityCapture();
}

// ---------- CostmapBackUpRecovery ----------

TEST(CostmapRecoveryModulesTest, BackUpCollisionFailsWithMessage) {
  auto global = makeWrapper(20, 0.5);
  costmap_2d::Costmap2D* grid = global->get().getCostmap();
  // 机器人位于 cell (4,4) (世界 2.25,2.25), 该 cell + 邻域设为 lethal。
  for (unsigned int y = 3; y <= 5; ++y) {
    for (unsigned int x = 3; x <= 5; ++x) {
      grid->setCost(x, y, costmap_2d::LETHAL_OBSTACLE);
    }
  }
  auto ctx = makeContext(global);
  ctx->transform_store_ = &ayanami::interfaces::TransformStore::instance();
  setRobot(2.25, 2.25);

  auto rec = std::make_shared<CostmapBackUpRecovery>();
  rec->initialize("costmap_back_up_recovery", ctx);

  std::string message;
  const FailureCode failure = rec->runRecovery(message);
  EXPECT_EQ(failure, FailureCode::RecoveryFailed);
  EXPECT_NE(message.find("collision detected"), std::string::npos) << message;
}

TEST(CostmapRecoveryModulesTest, BackUpCompletesOnFreeMap) {
  attachVelocityCapture();
  auto rec = std::make_shared<CostmapBackUpRecovery>();
  rec->initialize("costmap_back_up_recovery",
                  makeContext(makeWrapper(20, 0.5)));

  std::string message;
  const FailureCode failure = rec->runRecovery(message);
  EXPECT_EQ(failure, FailureCode::None) << message;
  EXPECT_NEAR(lastEmitted().twist.angular.z(), 0.0, 1e-9);
  detachVelocityCapture();
}

// ---------- CostmapWaitRecovery ----------

TEST(CostmapRecoveryModulesTest, WaitCompletesAfterDuration) {
  auto rec = std::make_shared<CostmapWaitRecovery>();
  rec->initialize("costmap_wait_recovery", makeContext(makeWrapper(20, 0.5)));

  const auto start = std::chrono::steady_clock::now();
  std::string message;
  const FailureCode failure = rec->runRecovery(message);
  const auto elapsed = std::chrono::duration<double>(
                           std::chrono::steady_clock::now() - start)
                           .count();

  EXPECT_EQ(failure, FailureCode::None) << message;
  EXPECT_GE(elapsed, 0.09);  // wait_duration=0.1 基本等满
}

TEST(CostmapRecoveryModulesTest, WaitCancelReturnsImmediately) {
  auto rec = std::make_shared<CostmapWaitRecovery>();
  rec->initialize("costmap_wait_recovery", makeContext(makeWrapper(20, 0.5)));

  std::atomic<bool> done{false};
  std::atomic<FailureCode> failure{FailureCode::None};
  std::string message;
  std::thread worker([&] {
    failure.store(rec->runRecovery(message));
    done.store(true);
  });
  std::this_thread::sleep_for(std::chrono::milliseconds(40));
  EXPECT_TRUE(rec->cancel());
  worker.join();
  EXPECT_TRUE(done.load());
  EXPECT_EQ(failure.load(), FailureCode::Canceled);
}

// ---------- CostmapClearCostmapRecovery ----------

TEST(CostmapRecoveryModulesTest, ClearCostmapClearsRadiusOnBothMaps) {
  auto global = makeWrapper(20, 0.5);
  auto local = makeWrapper(20, 0.5);
  auto global_obstacles =
      std::make_shared<costmap_2d_layers::SimpleObstacleLayer>();
  auto local_obstacles =
      std::make_shared<costmap_2d_layers::SimpleObstacleLayer>();
  global->get().addLayer(global_obstacles);
  local->get().addLayer(local_obstacles);
  costmap_2d::Costmap2D* gg = global->get().getCostmap();
  costmap_2d::Costmap2D* lg = local->get().getCostmap();
  // Mark dynamic obstacles near and far from the robot through the real layer.
  for (auto* layer : {global_obstacles.get(), local_obstacles.get()}) {
    layer->markObstacles(2.25, 2.25);
    layer->markObstacles(2.75, 2.75);
    layer->markObstacles(7.75, 7.75);
  }
  global->updateMap(2.25, 2.25, 0.0);
  local->updateMap(2.25, 2.25, 0.0);
  auto ctx = makeContext(global, local);
  ctx->transform_store_ = &ayanami::interfaces::TransformStore::instance();
  setRobot(2.25, 2.25);  // cell (4,4) 中心

  auto rec = std::make_shared<CostmapClearCostmapRecovery>();
  rec->initialize("costmap_clear_costmap_recovery", ctx);

  std::string message;
  const FailureCode failure = rec->runRecovery(message);
  EXPECT_EQ(failure, FailureCode::None) << message;

  // robot 附近 (半径内) → FREE_SPACE(0); 远处保持 lethal。
  for (auto* g : {gg, lg}) {
    EXPECT_EQ(g->getCost(4, 4), costmap_2d::FREE_SPACE);
    EXPECT_EQ(g->getCost(5, 5), costmap_2d::FREE_SPACE);
    EXPECT_EQ(g->getCost(15, 15), costmap_2d::LETHAL_OBSTACLE);
  }
}

// ---------- module.cpp 工厂符号 (dlopen 验证正确派生类) ----------

TEST(CostmapRecoveryModulesTest, ModuleFactoriesCreateCorrectTypes) {
  ASSERT_FALSE(g_spin_so.empty());
  ASSERT_FALSE(g_backup_so.empty());
  ASSERT_FALSE(g_wait_so.empty());
  ASSERT_FALSE(g_clear_so.empty());

  using Factory = ayanami::costmap::CostmapRecovery* (*)();

  {
    void* h = ayanami::open_so(g_spin_so);
    ASSERT_NE(h, nullptr);
    auto factory = reinterpret_cast<Factory>(
        ayanami::load_symbol(h, "create_costmap_recovery_module"));
    ASSERT_NE(factory, nullptr);
    ayanami::costmap::CostmapRecovery* r = factory();
    EXPECT_NE(dynamic_cast<CostmapSpinRecovery*>(r), nullptr);
    delete r;
  }
  {
    void* h = ayanami::open_so(g_backup_so);
    ASSERT_NE(h, nullptr);
    auto factory = reinterpret_cast<Factory>(
        ayanami::load_symbol(h, "create_costmap_recovery_module"));
    ASSERT_NE(factory, nullptr);
    ayanami::costmap::CostmapRecovery* r = factory();
    EXPECT_NE(dynamic_cast<CostmapBackUpRecovery*>(r), nullptr);
    delete r;
  }
  {
    void* h = ayanami::open_so(g_wait_so);
    ASSERT_NE(h, nullptr);
    auto factory = reinterpret_cast<Factory>(
        ayanami::load_symbol(h, "create_costmap_recovery_module"));
    ASSERT_NE(factory, nullptr);
    ayanami::costmap::CostmapRecovery* r = factory();
    EXPECT_NE(dynamic_cast<CostmapWaitRecovery*>(r), nullptr);
    delete r;
  }
  {
    void* h = ayanami::open_so(g_clear_so);
    ASSERT_NE(h, nullptr);
    auto factory = reinterpret_cast<Factory>(
        ayanami::load_symbol(h, "create_costmap_recovery_module"));
    ASSERT_NE(factory, nullptr);
    ayanami::costmap::CostmapRecovery* r = factory();
    EXPECT_NE(dynamic_cast<CostmapClearCostmapRecovery*>(r), nullptr);
    delete r;
  }
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  if (argc > 1) {
    g_spin_so = argv[1];
  }
  if (argc > 2) {
    g_backup_so = argv[2];
  }
  if (argc > 3) {
    g_wait_so = argv[3];
  }
  if (argc > 4) {
    g_clear_so = argv[4];
  }

  // 临时 config 目录: spin_dist/max_w 等参数 (与测试默认一致; wait_duration=0.1)。
  const std::filesystem::path dir =
      std::filesystem::temp_directory_path() / "ayanami_costmap_recovery_test";
  std::filesystem::remove_all(dir);
  std::filesystem::create_directories(dir);
  {
    std::ofstream ofs(dir / "config.json");
    ofs << R"({"global": {
      "costmap_spin_recovery": "costmap_spin_recovery.json",
      "costmap_back_up_recovery": "costmap_back_up_recovery.json",
      "costmap_wait_recovery": "costmap_wait_recovery.json",
      "costmap_clear_costmap_recovery": "costmap_clear_costmap_recovery.json"
    }})";
  }
  {
    std::ofstream ofs(dir / "costmap_spin_recovery.json");
    ofs << R"({"recovery": {"spin_dist": 1.57, "max_w": 0.5, "sim_cycle_dt": 0.01, "collision_radius": 0.3}})";
  }
  {
    std::ofstream ofs(dir / "costmap_back_up_recovery.json");
    ofs << R"({"recovery": {"backup_dist": 0.2, "max_v": 0.2, "sim_cycle_dt": 0.01, "collision_radius": 0.3}})";
  }
  {
    std::ofstream ofs(dir / "costmap_wait_recovery.json");
    ofs << R"({"recovery": {"wait_duration": 0.1, "sim_cycle_dt": 0.01}})";
  }
  {
    std::ofstream ofs(dir / "costmap_clear_costmap_recovery.json");
    ofs << R"({"recovery": {"clear_radius": 1.0, "affected_maps": "both"}})";
  }
  ayanami::GlobalConfig::instance(dir.string(), /*override_path=*/true);

  return RUN_ALL_TESTS();
}
