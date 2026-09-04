// Task 24+redesign: modules/costmap*_recovery (CostmapRecovery 族 4 个恢复插件)。
//
// 覆盖 (周期驱动 + 唯一发布者架构):
// - 插件只返回每周期期望速度, 由 CostmapRecoveryExecution 作为唯一发布者
//   按周期发布; spin/back_up 结束/取消/collision 终态必须补零 (行为测试)。
// - 插件不直接调 on_velocity_command / 不 sleep (行为 + grep 源码测试)。
// - spin 完成所设 spin_dist → 执行 RECOVERY_DONE, 期间发纯角速度, 终态零速度
// - back_up 倒车检测碰撞 (不可变 global 快照 lethal) → RECOVERY_FAILED,
//   message 含 "collision detected" (插件层 tick 契约)
// - wait 经执行周期驱动等满 wait_duration → RECOVERY_DONE; 中途 cancel → CANCELED
// - clear 经 CostmapCommandPort 提交 typed clear 命令, 唯一写者清机器人附近
//   radius → None, master grid 对应区域 = FREE_SPACE(0), 远处保持 lethal
// - 4 个 module.cpp 统一导出 create_extension_module (dlopen 验证正确类)

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Geometry>

#include <costmap_2d/costmap_2d.hpp>
#include <costmap_2d/layered_costmap.hpp>
#include <costmap_2d_layers/simple_obstacle_layer.hpp>
#include <costmap_server/costmap_wrapper.hpp>

#include <ayanami/abstract/components/robot_state_controller.hpp>
#include <ayanami/costmap/components/costmap_commands.hpp>
#include <ayanami/costmap/components/costmap_context.hpp>
#include <ayanami/costmap/components/costmap_recovery_execution.hpp>
#include <ayanami/costmap/core/costmap_command.hpp>
#include <ayanami/costmap/core/costmap_recovery.hpp>
#include <ayanami/interfaces/callbacks.hpp>
#include <ayanami/types.hpp>
#include <ayanami/messages/failure.hpp>
#include <ayanami/messages/geometry_msgs.hpp>
#include <costmap_back_up_recovery/costmap_back_up_recovery.hpp>
#include <costmap_clear_costmap_recovery/costmap_clear_costmap_recovery.hpp>
#include <costmap_spin_recovery/costmap_spin_recovery.hpp>
#include <costmap_wait_recovery/costmap_wait_recovery.hpp>
#include <ayanami/utility/config.hpp>
#include <ayanami/utility/load_module.hpp>

namespace {

using ayanami::ErrorCode;
using ayanami::RecoveryResultT;
using ayanami::costmap::ClearCostmapCommand;
using ayanami::costmap::CostmapCommandPort;
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

// 模块源码目录 (CMake 注入), 供 grep 源码测试用。
std::string g_modules_src;

// 临时 config 目录 (main() 内创建), 供构造各模块 PluginContext.config。
std::string g_config_dir;

// 各模块自身 json (PluginContext.config 参数单一来源)。
std::string moduleConfig(const std::string& json_name) {
  return (std::filesystem::path(g_config_dir) / json_name).string();
}

// 捕捉执行层发布的 on_velocity_command。
std::vector<TwistStamped> g_emitted;
std::mutex g_emit_mutex;
int g_emit_cb_id = -1;

void writeJsonFile(const std::filesystem::path& path,
                   const std::string& content) {
  std::ofstream ofs(path);
  ofs << content;
}

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
  ctx->setGlobalCostmap(global);
  ctx->setLocalCostmap(local);
  return ctx;
}

// 测试侧等价写者命令端口: submit → writer 级清图 (清层 + updateMapUnlocked 发布)。
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

 private:
  std::shared_ptr<costmap_2d::CostmapWrapper> global;
  std::shared_ptr<costmap_2d::CostmapWrapper> local;
  ayanami::components::RobotStateController* rsc;
  std::string robot_frame;
};

// 经 RobotStateController 设置 TF: map←base_link, 机器人位于世界 (wx, wy)。
void setRobot(double wx, double wy) {
  auto& rsc = ayanami::components::RobotStateController::instance();
  rsc.setTransform(
      "map", "base_link", ayanami::Timestamp::latest(),
      Eigen::Translation3d(wx, wy, 0.0) * Eigen::Quaterniond::Identity());
}

unsigned char cellCost(
    const std::shared_ptr<costmap_2d::CostmapWrapper>& wrapper, unsigned int x,
    unsigned int y) {
  std::shared_lock<std::shared_mutex> lock(wrapper->mutex());
  return wrapper->get().getCostmap()->getCost(x, y);
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

bool emittedIsEmpty() {
  std::lock_guard<std::mutex> lock(g_emit_mutex);
  return g_emitted.empty();
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

bool anyNonZeroLinear() {
  std::lock_guard<std::mutex> lock(g_emit_mutex);
  for (const auto& t : g_emitted) {
    if (std::abs(t.twist.linear.x()) > 1e-9) {
      return true;
    }
  }
  return false;
}

// 轮询直至 terminal state 或超时。
void waitExecutionState(ayanami::costmap::CostmapRecoveryExecution& exec,
                        int target, int max_iters = 200) {
  for (int i = 0; i < max_iters; ++i) {
    if (exec.getState() == target) {
      return;
    }
    exec.waitForStateUpdate(std::chrono::milliseconds(10));
  }
  ASSERT_EQ(exec.getState(), target);
}

}  // namespace

// ---------- 唯一发布者行为: 插件不直接调 on_velocity_command ----------

TEST(CostmapRecoveryModulesTest, pluginsDoNotPublishVelocity) {
  attachVelocityCapture();
  auto ctx = makeContext(makeWrapper(20, 0.5));
  auto spin = std::make_shared<CostmapSpinRecovery>();
  spin->configure(ayanami::PluginContext{"costmap_spin_recovery",
                                         ayanami::Config(moduleConfig("spin.json"))},
                  ctx->getSharedConstPtr());
  auto backup = std::make_shared<CostmapBackUpRecovery>();
  backup->configure(ayanami::PluginContext{"costmap_back_up_recovery",
                                           ayanami::Config(moduleConfig("backup.json"))},
                    ctx->getSharedConstPtr());
  auto wait = std::make_shared<CostmapWaitRecovery>();
  wait->configure(ayanami::PluginContext{"costmap_wait_recovery",
                                         ayanami::Config(moduleConfig("wait.json"))},
                  ctx->getSharedConstPtr());

  spin->start();
  backup->start();
  wait->start();
  (void)spin->tick();
  (void)backup->tick();
  (void)wait->tick();

  // 插件的 tick 只是驱动周期逻辑, 绝不自行发布速度命令。
  EXPECT_TRUE(emittedIsEmpty())
      << "recovery plugins must not publish on_velocity_command directly";
  detachVelocityCapture();
}

TEST(CostmapRecoveryModulesTest, pluginsSourceHasNoDirectPublish) {
  ASSERT_FALSE(g_modules_src.empty());
  const std::vector<std::string> files = {
      "costmap_spin_recovery/costmap_spin_recovery.cpp",
      "costmap_back_up_recovery/costmap_back_up_recovery.cpp",
      "costmap_wait_recovery/costmap_wait_recovery.cpp",
      "costmap_clear_costmap_recovery/costmap_clear_costmap_recovery.cpp"};
  const std::vector<std::string> forbidden = {
      "on_velocity_command", "NavigationCallbacks", "sleep_for", "<thread>"};
  for (const auto& rel : files) {
    std::ifstream ifs(std::filesystem::path(g_modules_src) / rel);
    ASSERT_TRUE(ifs.good()) << "cannot open " << rel;
    const std::string src((std::istreambuf_iterator<char>(ifs)),
                          std::istreambuf_iterator<char>());
    for (const auto& token : forbidden) {
      EXPECT_EQ(src.find(token), std::string::npos)
          << rel << " must not contain '" << token
          << "' (发布与 sleep 职责在 RecoveryExecution)";
    }
  }
}

// ---------- CostmapSpinRecovery (经执行驱动, 唯一发布者) ----------

TEST(CostmapRecoveryModulesTest, spinCompletesWithSuccessViaExecution) {
  attachVelocityCapture();
  auto ctx = makeContext(makeWrapper(20, 0.5));
  auto rec = std::make_shared<CostmapSpinRecovery>();
  rec->configure(ayanami::PluginContext{"costmap_spin_recovery",
                                        ayanami::Config(moduleConfig("spin.json"))},
                 ctx->getSharedConstPtr());
  ayanami::costmap::CostmapRecoveryExecution exec("spin", rec, ctx);
  exec.start();
  waitExecutionState(exec, ayanami::components::RecoveryExecution::RECOVERY_DONE);
  EXPECT_EQ(exec.getError(), ErrorCode::NONE);
  exec.join();

  EXPECT_TRUE(anyNonZeroAngular());            // 发过纯角速度
  EXPECT_NEAR(lastEmitted().twist.angular.z(), 0.0, 1e-9);  // 终态零速度
  detachVelocityCapture();
}

TEST(CostmapRecoveryModulesTest, spinCancelEmitsZeroAndCanceledViaExecution) {
  attachVelocityCapture();
  auto ctx = makeContext(makeWrapper(20, 0.5));
  auto rec = std::make_shared<CostmapSpinRecovery>();
  rec->configure(ayanami::PluginContext{"costmap_spin_recovery",
                                        ayanami::Config(moduleConfig("spin.json"))},
                 ctx->getSharedConstPtr());
  ayanami::costmap::CostmapRecoveryExecution exec("spin", rec, ctx);
  exec.setCyclePeriod(std::chrono::milliseconds(10));
  exec.start();
  // 等至少一帧角速度发出, 再 cancel。
  for (int i = 0; i < 100 && !anyNonZeroAngular(); ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  EXPECT_TRUE(anyNonZeroAngular());
  exec.cancel();
  waitExecutionState(exec, ayanami::components::RecoveryExecution::CANCELED);
  exec.join();
  EXPECT_NEAR(lastEmitted().twist.angular.z(), 0.0, 1e-9);
  detachVelocityCapture();
}

// ---------- 唯一发布者安全校验: finite + max_vel 上限 (超限清零) ----------

TEST(CostmapRecoveryModulesTest, spinVelocityExceedingMaxThetaIsZeroed) {
  attachVelocityCapture();
  auto ctx = makeContext(makeWrapper(20, 0.5));
  auto rec = std::make_shared<CostmapSpinRecovery>();
  rec->configure(ayanami::PluginContext{"costmap_spin_recovery",
                                        ayanami::Config(moduleConfig("spin.json"))},
                 ctx->getSharedConstPtr());
  ayanami::costmap::CostmapRecoveryExecution exec("spin", rec, ctx);
  // max_w=0.3 > 注入上限 max_vel_theta=0.1 → 非零命令必须被清零。
  exec.setMaximumVelocity(1.0, 1.0, 0.1);
  exec.start();
  waitExecutionState(exec, ayanami::components::RecoveryExecution::RECOVERY_DONE);
  exec.join();
  EXPECT_FALSE(anyNonZeroAngular())
      << "recovery commands exceeding max_vel_theta must be zeroed by the "
         "execution";
  EXPECT_NEAR(lastEmitted().twist.angular.z(), 0.0, 1e-9);
  detachVelocityCapture();
}

// ---------- CostmapBackUpRecovery ----------

TEST(CostmapRecoveryModulesTest, backUpCollisionFailsWithMessage) {
  auto global = makeWrapper(20, 0.5);
  {
    std::unique_lock<std::shared_mutex> lock(global->mutex());
    costmap_2d::Costmap2D* grid = global->get().getCostmap();
    // 机器人位于 cell (4,4) (世界 2.25,2.25), 该 cell + 邻域设为 lethal。
    for (unsigned int y = 3; y <= 5; ++y) {
      for (unsigned int x = 3; x <= 5; ++x) {
        grid->setCost(x, y, costmap_2d::LETHAL_OBSTACLE);
      }
    }
    global->updateMapUnlocked(2.25, 2.25, 0.0);
  }
  auto ctx = makeContext(global);
  setRobot(2.25, 2.25);

  auto rec = std::make_shared<CostmapBackUpRecovery>();
  rec->configure(ayanami::PluginContext{"costmap_back_up_recovery",
                                        ayanami::Config(moduleConfig("backup.json"))},
                 ctx->getSharedConstPtr());

  rec->start();
  const RecoveryResultT outcome = rec->tick();
  ASSERT_FALSE(outcome.ok);
  EXPECT_EQ(outcome.error, ErrorCode::RECOVERY_FAILED);
  EXPECT_NE(outcome.message.find("collision detected"), std::string::npos)
      << outcome.message;
}

TEST(CostmapRecoveryModulesTest, backUpCompletesOnFreeMapViaExecution) {
  attachVelocityCapture();
  auto ctx = makeContext(makeWrapper(20, 0.5));
  auto rec = std::make_shared<CostmapBackUpRecovery>();
  rec->configure(ayanami::PluginContext{"costmap_back_up_recovery",
                                        ayanami::Config(moduleConfig("backup.json"))},
                 ctx->getSharedConstPtr());
  ayanami::costmap::CostmapRecoveryExecution exec("back_up", rec, ctx);
  exec.start();
  waitExecutionState(exec, ayanami::components::RecoveryExecution::RECOVERY_DONE);
  EXPECT_EQ(exec.getError(), ErrorCode::NONE);
  exec.join();

  EXPECT_TRUE(anyNonZeroLinear());              // 发过负向线速度
  EXPECT_NEAR(lastEmitted().twist.linear.x(), 0.0, 1e-9);  // 终态零速度
  detachVelocityCapture();
}

// ---------- CostmapWaitRecovery (经执行周期驱动, 无运动) ----------

TEST(CostmapRecoveryModulesTest, waitCompletesAfterDurationViaExecution) {
  auto ctx = makeContext(makeWrapper(20, 0.5));
  auto rec = std::make_shared<CostmapWaitRecovery>();
  rec->configure(ayanami::PluginContext{"costmap_wait_recovery",
                                        ayanami::Config(moduleConfig("wait.json"))},
                 ctx->getSharedConstPtr());
  ayanami::costmap::CostmapRecoveryExecution exec("wait", rec, ctx);
  const auto start = std::chrono::steady_clock::now();
  exec.start();
  waitExecutionState(exec, ayanami::components::RecoveryExecution::RECOVERY_DONE);
  const auto elapsed = std::chrono::duration<double>(
                           std::chrono::steady_clock::now() - start)
                           .count();
  EXPECT_EQ(exec.getError(), ErrorCode::NONE);
  exec.join();

  // wait_duration=0.1, 周期 10ms ⇒ 约 10 帧 (总时长明显 > 0)。
  EXPECT_GE(elapsed, 0.05);
}

TEST(CostmapRecoveryModulesTest, waitCancelReturnsImmediatelyViaExecution) {
  auto ctx = makeContext(makeWrapper(20, 0.5));
  auto rec = std::make_shared<CostmapWaitRecovery>();
  rec->configure(ayanami::PluginContext{"costmap_wait_recovery",
                                        ayanami::Config(moduleConfig("wait.json"))},
                 ctx->getSharedConstPtr());
  ayanami::costmap::CostmapRecoveryExecution exec("wait", rec, ctx);
  exec.start();
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  exec.cancel();
  waitExecutionState(exec, ayanami::components::RecoveryExecution::CANCELED);
  exec.join();
  EXPECT_EQ(exec.getError(), ErrorCode::CANCELED);
}

// ---------- CostmapClearCostmapRecovery (经命令端口, 无运动) ----------

TEST(CostmapRecoveryModulesTest, clearCostmapClearsRadiusOnBothMaps) {
  auto global = makeWrapper(20, 0.5);
  auto local = makeWrapper(20, 0.5);
  auto global_obstacles =
      std::make_shared<costmap_2d_layers::SimpleObstacleLayer>();
  auto local_obstacles =
      std::make_shared<costmap_2d_layers::SimpleObstacleLayer>();
  global->get().addLayer(global_obstacles);
  local->get().addLayer(local_obstacles);
  // Mark dynamic obstacles near and far from the robot through the real layer.
  {
    std::unique_lock<std::shared_mutex> lock(global->mutex());
    global_obstacles->markObstacles(2.25, 2.25);
    global_obstacles->markObstacles(2.75, 2.75);
    global_obstacles->markObstacles(7.75, 7.75);
    global->updateMapUnlocked(2.25, 2.25, 0.0);
  }
  {
    std::unique_lock<std::shared_mutex> lock(local->mutex());
    local_obstacles->markObstacles(2.25, 2.25);
    local_obstacles->markObstacles(2.75, 2.75);
    local_obstacles->markObstacles(7.75, 7.75);
    local->updateMapUnlocked(2.25, 2.25, 0.0);
  }
  auto ctx = makeContext(global, local);
  setRobot(2.25, 2.25);  // cell (4,4) 中心

  auto port = std::make_shared<TestWriterPort>(
      global, local, &ayanami::components::RobotStateController::instance(),
      ctx->getRobotFrame());
  auto rec = std::make_shared<CostmapClearCostmapRecovery>();
  rec->configure(ayanami::PluginContext{"costmap_clear_costmap_recovery",
                                        ayanami::Config(moduleConfig("clear.json"))},
                 ctx->getSharedConstPtr());
  rec->setCommandPort(port);

  rec->start();
  const RecoveryResultT outcome = rec->tick();
  ASSERT_TRUE(outcome.ok) << outcome.message;

  // robot 附近 (半径内) → FREE_SPACE(0); 远处保持 lethal。
  for (const auto& wrapper : {global, local}) {
    EXPECT_EQ(cellCost(wrapper, 4, 4), costmap_2d::FREE_SPACE);
    EXPECT_EQ(cellCost(wrapper, 5, 5), costmap_2d::FREE_SPACE);
    EXPECT_EQ(cellCost(wrapper, 15, 15), costmap_2d::LETHAL_OBSTACLE);
  }
}

// ---------- module.cpp 工厂符号 (公共工厂 + 类型化 cast 验证正确派生类) ----------

TEST(CostmapRecoveryModulesTest, moduleFactoriesCreateCorrectTypes) {
  ASSERT_FALSE(g_spin_so.empty());
  ASSERT_FALSE(g_backup_so.empty());
  ASSERT_FALSE(g_wait_so.empty());
  ASSERT_FALSE(g_clear_so.empty());

  EXPECT_NE(std::dynamic_pointer_cast<CostmapSpinRecovery>(
                ayanami::ExtensionModule::load_module(g_spin_so)),
            nullptr);
  EXPECT_NE(std::dynamic_pointer_cast<CostmapBackUpRecovery>(
                ayanami::ExtensionModule::load_module(g_backup_so)),
            nullptr);
  EXPECT_NE(std::dynamic_pointer_cast<CostmapWaitRecovery>(
                ayanami::ExtensionModule::load_module(g_wait_so)),
            nullptr);
  EXPECT_NE(std::dynamic_pointer_cast<CostmapClearCostmapRecovery>(
                ayanami::ExtensionModule::load_module(g_clear_so)),
            nullptr);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  if (argc > 1) g_spin_so = argv[1];
  if (argc > 2) g_backup_so = argv[2];
  if (argc > 3) g_wait_so = argv[3];
  if (argc > 4) g_clear_so = argv[4];
  if (argc > 5) g_modules_src = argv[5];

  // 临时 config 目录 (严格单层索引): 4 个恢复插件各在自己的 json 文件中声明
  // "recovery" 段参数; 每个模块构造时经 GlobalConfig::getConfigPath 显式读取
  // (无 thread_local 注入)。键互不冲突 (短时长便于执行层周期驱动测试;
  // wait_duration=0.1)。
  const std::filesystem::path dir =
      std::filesystem::temp_directory_path() / "ayanami_costmap_recovery_test";
  std::filesystem::remove_all(dir);
  std::filesystem::create_directories(dir);
  g_config_dir = dir.string();
  {
    std::ofstream ofs(dir / "config.json");
    ofs << R"({"global": {
      "config_spin_recovery": "spin.json",
      "config_back_up_recovery": "backup.json",
      "config_wait_recovery": "wait.json",
      "config_clear_costmap_recovery": "clear.json"
    }})";
  }
  writeJsonFile(dir / "spin.json", R"({"recovery": {
      "spin_dist": 0.03, "max_w": 0.3,
      "sim_cycle_dt": 0.01, "collision_radius": 0.3
    }})");
  writeJsonFile(dir / "backup.json", R"({"recovery": {
      "backup_dist": 0.02, "max_v": 0.2,
      "sim_cycle_dt": 0.01, "collision_radius": 0.3
    }})");
  writeJsonFile(dir / "wait.json", R"({"recovery": {
      "wait_duration": 0.1,
      "sim_cycle_dt": 0.01
    }})");
  writeJsonFile(dir / "clear.json", R"({"recovery": {
      "clear_radius": 1.0, "affected_maps": "both"
    }})");
  ayanami::GlobalConfig::instance(dir.string(), /*override_path=*/true);

  return RUN_ALL_TESTS();
}
