// +redesign: modules/costmap*_recovery (the 4 CostmapRecovery-family recovery modules).
//
// Coverage (cycle-driven + single-publisher architecture):
// - modules only return the desired per-cycle velocity; CostmapRecoveryExecution is the single publisher and
// publishes it per cycle; spin/back_up end/cancel/collision terminal states must emit zeros (behavior tests).
// - modules never call on_velocity_command directly / never sleep (behavior + grep-source tests).
// - spin completes once the configured spin_dist is covered -> execution RECOVERY_DONE; pure angular velocity while spinning, zero velocity in the terminal state
// - back_up detects a collision while reversing (lethal on the immutable global snapshot) -> RECOVERY_FAILED,
// message contains "collision detected" (module-layer tick contract)
// - wait is driven by execution cycles until wait_duration elapses -> RECOVERY_DONE; cancel mid-way -> CANCELED
// - clear submits a typed clear command via CostmapCommandPort; the single writer clears the area around the robot
// to None and the master grid region becomes FREE_SPACE(0) while distant cells stay lethal
// - the 4 module.cpp files uniformly export create_extension_module (dlopen verifies the correct classes)

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
#include <ayanami/costmap/components/costmap_map_context.hpp>
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
using ayanami::ResultT;
using ayanami::costmap::ClearCostmapCommand;
using ayanami::costmap::CostmapCommandPort;
using ayanami::costmap::CostmapMapContext;
using ayanami::geometry_msgs::TwistStamped;
using ayanami::modules::CostmapBackUpRecovery;
using ayanami::modules::CostmapClearCostmapRecovery;
using ayanami::modules::CostmapSpinRecovery;
using ayanami::modules::CostmapWaitRecovery;

// The 4 module .so files (injected via argv, order: spin/back_up/wait/clear_costmap).
std::string g_spin_so;
std::string g_backup_so;
std::string g_wait_so;
std::string g_clear_so;

// Module source directory (injected by CMake), used by the grep-source tests.
std::string g_modules_src;

// Temp config dir (created in main), used to build each module ModuleContext.config.
std::string g_config_dir;

// Each module's own json (single source of ModuleContext.config parameters).
std::string moduleConfig(const std::string& json_name) {
  return (std::filesystem::path(g_config_dir) / json_name).string();
}

// Captures on_velocity_command published by the execution layer.
std::vector<TwistStamped> g_emitted;
std::mutex g_emit_mutex;
int g_emit_cb_id = -1;

void writeJsonFile(const std::filesystem::path& path,
                   const std::string& content) {
  std::ofstream ofs(path);
  ofs << content;
}

// ---------------- Test utilities ----------------

// Builds an NxN @ res master grid with origin=(0,0) (no layers, direct setCost).
std::shared_ptr<costmap_2d::CostmapWrapper> makeWrapper(unsigned int size,
                                                        double res) {
  auto lm = std::make_shared<costmap_2d::LayeredCostmap>(
      "map", /*rolling_window=*/false, /*track_unknown=*/false);
  lm->resizeMap(size, size, res, 0.0, 0.0);
  return std::make_shared<costmap_2d::CostmapWrapper>(lm, "map");
}

std::shared_ptr<CostmapMapContext> makeContext(
    const std::shared_ptr<costmap_2d::CostmapWrapper>& global,
    const std::shared_ptr<costmap_2d::CostmapWrapper>& local = nullptr) {
  auto ctx = std::make_shared<CostmapMapContext>();
  ctx->setGlobalCostmap(global);
  ctx->setLocalCostmap(local);
  return ctx;
}

/// Test-side equivalent writer command port: submit -> writer-level map clear (clear layers + updateMapUnlocked publish).
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
      ok = rsc->lookupTransform(global->globalFrame(), robot_frame,
                                ayanami::Timestamp::latest(), p) &&
           ok;
      ok = ayanami::costmap::clearCostmapArea(global, p, command.clear_radius) &&
           ok;
    }
    if (clear_local && local) {
      Eigen::Isometry3d p = Eigen::Isometry3d::Identity();
      ok = rsc->lookupTransform(local->globalFrame(), robot_frame,
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

/// Sets the TF via RobotStateController: map<-base_link with the robot at world (wx, wy).
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

/// Polls until a terminal state or timeout.
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

// ---------- Single-publisher behavior: modules never call on_velocity_command ----------

TEST(CostmapRecoveryModulesTest, modulesDoNotPublishVelocity) {
  attachVelocityCapture();
  auto ctx = makeContext(makeWrapper(20, 0.5));
  auto spin = std::make_shared<CostmapSpinRecovery>();
  spin->configure(ayanami::ModuleContext{"costmap_spin_recovery",
                                         ayanami::Config(moduleConfig("spin.json"))},
                  ctx->getSharedConstPtr());
  auto backup = std::make_shared<CostmapBackUpRecovery>();
  backup->configure(ayanami::ModuleContext{"costmap_back_up_recovery",
                                           ayanami::Config(moduleConfig("backup.json"))},
                    ctx->getSharedConstPtr());
  auto wait = std::make_shared<CostmapWaitRecovery>();
  wait->configure(ayanami::ModuleContext{"costmap_wait_recovery",
                                         ayanami::Config(moduleConfig("wait.json"))},
                  ctx->getSharedConstPtr());

  spin->start();
  backup->start();
  wait->start();
  ayanami::RecoveryCycleData cycle;
  (void)spin->tick(cycle);
  (void)backup->tick(cycle);
  (void)wait->tick(cycle);

  // The module tick only drives the cycle logic; it never publishes velocity commands itself.
  EXPECT_TRUE(emittedIsEmpty())
      << "recovery modules must not publish on_velocity_command directly";
  detachVelocityCapture();
}

TEST(CostmapRecoveryModulesTest, modulesSourceHasNoDirectPublish) {
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
          << "' (publish and sleep duties belong to RecoveryExecution)";
    }
  }
}

// ---------- CostmapSpinRecovery (execution-driven, single publisher) ----------

TEST(CostmapRecoveryModulesTest, spinCompletesWithSuccessViaExecution) {
  attachVelocityCapture();
  auto ctx = makeContext(makeWrapper(20, 0.5));
  auto rec = std::make_shared<CostmapSpinRecovery>();
  rec->configure(ayanami::ModuleContext{"costmap_spin_recovery",
                                        ayanami::Config(moduleConfig("spin.json"))},
                 ctx->getSharedConstPtr());
  ayanami::costmap::CostmapRecoveryExecution exec("spin", rec, ctx);
  // The execution owns the cadence (production: CostmapNavigationServer reads
  // it from the module json's recovery.sim_cycle_dt); the module takes it from here.
  exec.setCyclePeriod(std::chrono::milliseconds(10));
  exec.start();
  waitExecutionState(exec, ayanami::components::RecoveryExecution::RECOVERY_DONE);
  EXPECT_EQ(exec.getError(), ErrorCode::NONE);
  exec.join();

  EXPECT_TRUE(anyNonZeroAngular());            // Pure angular velocity was emitted
  EXPECT_NEAR(lastEmitted().twist.angular.z(), 0.0, 1e-9);  // zero velocity in the terminal state
  detachVelocityCapture();
}

TEST(CostmapRecoveryModulesTest, spinCancelEmitsZeroAndCanceledViaExecution) {
  attachVelocityCapture();
  auto ctx = makeContext(makeWrapper(20, 0.5));
  auto rec = std::make_shared<CostmapSpinRecovery>();
  rec->configure(ayanami::ModuleContext{"costmap_spin_recovery",
                                        ayanami::Config(moduleConfig("spin.json"))},
                 ctx->getSharedConstPtr());
  ayanami::costmap::CostmapRecoveryExecution exec("spin", rec, ctx);
  exec.setCyclePeriod(std::chrono::milliseconds(10));
  exec.start();
  // Wait until at least one angular velocity frame is emitted, then cancel.
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

// ---------- Single-publisher safety checks: finite + max_vel caps (over-limit zeroed) ----------

TEST(CostmapRecoveryModulesTest, spinVelocityExceedingMaxThetaIsZeroed) {
  attachVelocityCapture();
  auto ctx = makeContext(makeWrapper(20, 0.5));
  auto rec = std::make_shared<CostmapSpinRecovery>();
  rec->configure(ayanami::ModuleContext{"costmap_spin_recovery",
                                        ayanami::Config(moduleConfig("spin.json"))},
                 ctx->getSharedConstPtr());
  ayanami::costmap::CostmapRecoveryExecution exec("spin", rec, ctx);
  // The execution owns the cadence (production: CostmapNavigationServer reads
  // it from the module json's recovery.sim_cycle_dt); the module takes it from here.
  exec.setCyclePeriod(std::chrono::milliseconds(10));
  // max_w=0.3 > the injected cap max_vel_theta=0.1 -> non-zero commands must be zeroed.
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
    // The robot is at cell (4,4) (world 2.25,2.25); that cell + neighborhood are set lethal.
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
  rec->configure(ayanami::ModuleContext{"costmap_back_up_recovery",
                                        ayanami::Config(moduleConfig("backup.json"))},
                 ctx->getSharedConstPtr());

  rec->start();
  ayanami::RecoveryCycleData cycle;
  const ResultT<void> outcome = rec->tick(cycle);
  ASSERT_FALSE(outcome.ok);
  EXPECT_EQ(outcome.error, ErrorCode::RECOVERY_FAILED);
  EXPECT_NE(outcome.message.find("collision detected"), std::string::npos)
      << outcome.message;
}

TEST(CostmapRecoveryModulesTest, backUpCompletesOnFreeMapViaExecution) {
  attachVelocityCapture();
  auto ctx = makeContext(makeWrapper(20, 0.5));
  auto rec = std::make_shared<CostmapBackUpRecovery>();
  rec->configure(ayanami::ModuleContext{"costmap_back_up_recovery",
                                        ayanami::Config(moduleConfig("backup.json"))},
                 ctx->getSharedConstPtr());
  ayanami::costmap::CostmapRecoveryExecution exec("back_up", rec, ctx);
  // The execution owns the cadence (production: CostmapNavigationServer reads
  // it from the module json's recovery.sim_cycle_dt); the module takes it from here.
  exec.setCyclePeriod(std::chrono::milliseconds(10));
  exec.start();
  waitExecutionState(exec, ayanami::components::RecoveryExecution::RECOVERY_DONE);
  EXPECT_EQ(exec.getError(), ErrorCode::NONE);
  exec.join();

  EXPECT_TRUE(anyNonZeroLinear());              // Negative linear velocity was emitted
  EXPECT_NEAR(lastEmitted().twist.linear.x(), 0.0, 1e-9);  // zero velocity in the terminal state
  detachVelocityCapture();
}

// ---------- CostmapWaitRecovery (execution-cycle driven, no motion) ----------

TEST(CostmapRecoveryModulesTest, waitCompletesAfterDurationViaExecution) {
  auto ctx = makeContext(makeWrapper(20, 0.5));
  auto rec = std::make_shared<CostmapWaitRecovery>();
  rec->configure(ayanami::ModuleContext{"costmap_wait_recovery",
                                        ayanami::Config(moduleConfig("wait.json"))},
                 ctx->getSharedConstPtr());
  ayanami::costmap::CostmapRecoveryExecution exec("wait", rec, ctx);
  // The execution owns the cadence (production: CostmapNavigationServer reads
  // it from the module json's recovery.sim_cycle_dt); the module takes it from here.
  exec.setCyclePeriod(std::chrono::milliseconds(10));
  const auto start = std::chrono::steady_clock::now();
  exec.start();
  waitExecutionState(exec, ayanami::components::RecoveryExecution::RECOVERY_DONE);
  const auto elapsed = std::chrono::duration<double>(
                           std::chrono::steady_clock::now() - start)
                           .count();
  EXPECT_EQ(exec.getError(), ErrorCode::NONE);
  exec.join();

  // wait_duration=0.1 with a 10ms cycle => about 10 frames (total duration clearly > 0).
  EXPECT_GE(elapsed, 0.05);
}

TEST(CostmapRecoveryModulesTest, waitCancelReturnsImmediatelyViaExecution) {
  auto ctx = makeContext(makeWrapper(20, 0.5));
  auto rec = std::make_shared<CostmapWaitRecovery>();
  rec->configure(ayanami::ModuleContext{"costmap_wait_recovery",
                                        ayanami::Config(moduleConfig("wait.json"))},
                 ctx->getSharedConstPtr());
  ayanami::costmap::CostmapRecoveryExecution exec("wait", rec, ctx);
  // The execution owns the cadence (production: CostmapNavigationServer reads
  // it from the module json's recovery.sim_cycle_dt); the module takes it from here.
  exec.setCyclePeriod(std::chrono::milliseconds(10));
  exec.start();
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  exec.cancel();
  waitExecutionState(exec, ayanami::components::RecoveryExecution::CANCELED);
  exec.join();
  EXPECT_EQ(exec.getError(), ErrorCode::CANCELED);
}

// ---------- CostmapClearCostmapRecovery (via the command port, no motion) ----------

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
  setRobot(2.25, 2.25);  // Center of cell (4,4)

  auto port = std::make_shared<TestWriterPort>(
      global, local, &ayanami::components::RobotStateController::instance(),
      ctx->robotFrame());
  // The command port is exposed via the map context (fetched as map->commandPort inside configure).
  ctx->setCommandPort(port);
  auto rec = std::make_shared<CostmapClearCostmapRecovery>();
  rec->configure(ayanami::ModuleContext{"costmap_clear_costmap_recovery",
                                        ayanami::Config(moduleConfig("clear.json"))},
                 ctx->getSharedConstPtr());

  rec->start();
  ayanami::RecoveryCycleData cycle;
  const ResultT<void> outcome = rec->tick(cycle);
  ASSERT_TRUE(outcome.ok) << outcome.message;

  // Near the robot (within the radius) -> FREE_SPACE(0); distant cells stay lethal.
  for (const auto& wrapper : {global, local}) {
    EXPECT_EQ(cellCost(wrapper, 4, 4), costmap_2d::FREE_SPACE);
    EXPECT_EQ(cellCost(wrapper, 5, 5), costmap_2d::FREE_SPACE);
    EXPECT_EQ(cellCost(wrapper, 15, 15), costmap_2d::LETHAL_OBSTACLE);
  }
}

// ---------- module.cpp factory symbols (common factory + typed cast verifying the correct derived classes) ----------

TEST(CostmapRecoveryModulesTest, moduleFactoriesCreateCorrectTypes) {
  ASSERT_FALSE(g_spin_so.empty());
  ASSERT_FALSE(g_backup_so.empty());
  ASSERT_FALSE(g_wait_so.empty());
  ASSERT_FALSE(g_clear_so.empty());

  EXPECT_NE(std::dynamic_pointer_cast<CostmapSpinRecovery>(
                ayanami::ExtensionModule::loadModule(g_spin_so)),
            nullptr);
  EXPECT_NE(std::dynamic_pointer_cast<CostmapBackUpRecovery>(
                ayanami::ExtensionModule::loadModule(g_backup_so)),
            nullptr);
  EXPECT_NE(std::dynamic_pointer_cast<CostmapWaitRecovery>(
                ayanami::ExtensionModule::loadModule(g_wait_so)),
            nullptr);
  EXPECT_NE(std::dynamic_pointer_cast<CostmapClearCostmapRecovery>(
                ayanami::ExtensionModule::loadModule(g_clear_so)),
            nullptr);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  if (argc > 1) g_spin_so = argv[1];
  if (argc > 2) g_backup_so = argv[2];
  if (argc > 3) g_wait_so = argv[3];
  if (argc > 4) g_clear_so = argv[4];
  if (argc > 5) g_modules_src = argv[5];

  // Temp config dir (strictly single-level index): the 4 recovery modules each declare their "recovery"
  // section parameters in their own json file; each module reads them explicitly at construction via GlobalConfig::getConfigPath
  // (no thread_local injection). The keys do not collide (short durations ease execution-layer cycle-driven testing;
  // wait_duration=0.1).
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
  // Section convention: module params live under the json "name" field
  // (= ModuleContext::name); sim_cycle_dt is read by the *server* out of the
  // section next to name/so_name/role.
  writeJsonFile(dir / "spin.json", R"({"recovery": {"sim_cycle_dt": 0.01},
      "costmap_spin_recovery": {
        "spin_dist": 0.03, "max_w": 0.3, "collision_radius": 0.3
      }})");
  writeJsonFile(dir / "backup.json", R"({"recovery": {"sim_cycle_dt": 0.01},
      "costmap_back_up_recovery": {
        "backup_dist": 0.02, "max_v": 0.2, "collision_radius": 0.3
      }})");
  writeJsonFile(dir / "wait.json", R"({"recovery": {"sim_cycle_dt": 0.01},
      "costmap_wait_recovery": {
        "wait_duration": 0.1
      }})");
  writeJsonFile(dir / "clear.json", R"({
      "costmap_clear_costmap_recovery": {
        "clear_radius": 1.0, "affected_maps": "both"
      }})");
  ayanami::GlobalConfig::instance(dir.string(), /*override_path=*/true);

  return RUN_ALL_TESTS();
}
