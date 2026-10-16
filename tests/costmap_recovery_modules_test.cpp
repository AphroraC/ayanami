// The four CostmapRecovery-family recovery modules (revived for the FSM
// era): plain RecoveryExecution drives them, the cleanup module clears
// through the writer-port seam (clearCostmapArea), spin/back_up read the
// global grid under the reader lock.
//
// Coverage:
// - modules only return the desired per-cycle velocity; RecoveryExecution
//   is the single publisher (per-cycle publish, zero on terminal states);
// - modules never call on_velocity_command directly / never sleep;
// - spin completes over its configured duration; cancel mid-way zeros and
//   settles CANCELED; over-limit suggestions are clamped, not zeroed;
// - back_up: direction-aware refusal near obstacles; contact-side escape;
//   boxed stall fails on physical displacement;
// - wait runs its duration then SUCCESS;
// - cleanup clears the radius around the robot on both maps through the
//   injected clear port (distant marks survive);
// - the 4 module.cpp factories export the correct derived classes.

#include <gtest/gtest.h>

#include "test_transform_source.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Geometry>

#include <costmap/costmap_wrapper.hpp>
#include <costmap/layered_costmap.hpp>
#include <costmap/simple_obstacle_layer.hpp>

#include <ayanami/abstract/components/recovery_execution.hpp>
#include <ayanami/abstract/components/robot_state_controller.hpp>
#include <ayanami/costmap/components/costmap_commands.hpp>
#include <ayanami/costmap/components/costmap_map_context.hpp>
#include <ayanami/costmap/core/costmap_recovery.hpp>
#include <ayanami/interfaces/callbacks.hpp>
#include <ayanami/types.hpp>
#include <ayanami/messages/failure.hpp>
#include <ayanami/messages/geometry_msgs.hpp>
#include <ayanami/utility/config.hpp>
#include <ayanami/utility/load_module.hpp>

#include "costmap_back_up_recovery.hpp"
#include "costmap_cleanup_recovery.hpp"
#include "costmap_spin_recovery.hpp"
#include "costmap_wait_recovery.hpp"

using ayanami::testing::testTransforms;
namespace {

using ayanami::ErrorCode;
using ayanami::ResultT;
using ayanami::costmap::ClearCostmapCommand;
using ayanami::costmap::CostmapMapContext;
using ayanami::geometry_msgs::TwistStamped;
using ayanami::modules::CostmapBackUpRecovery;
using ayanami::modules::CostmapCleanupRecovery;
using ayanami::modules::CostmapSpinRecovery;
using ayanami::modules::CostmapWaitRecovery;
using RecoveryExecution = ayanami::components::RecoveryExecution;

// The 4 module .so files (injected via argv: cleanup/spin/wait/back_up).
std::string g_cleanup_so;
std::string g_spin_so;
std::string g_wait_so;
std::string g_backup_so;

// Module source directory (injected by CMake), for the grep-source test.
std::string g_modules_src;

// Temp config dir (created in main), the single source of each module's
// ModuleContext.config.
std::string g_config_dir;

// Captures on_velocity_command published by the execution layer.
std::vector<TwistStamped> g_emitted;
std::mutex g_emit_mutex;
ayanami::CallbackSubscription g_emit_cb;

// ---------------- Test utilities ----------------

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

/// Test-side writer port: the same body the host binds to
/// CostmapNavigationServer::executeClearCommand (lookup pose + the
/// sole-writer clearCostmapArea on the requested maps).
ayanami::costmap::CostmapRecovery::ClearCommandFn makeClearPort(
    const std::shared_ptr<costmap_2d::CostmapWrapper>& global,
    const std::shared_ptr<costmap_2d::CostmapWrapper>& local,
    const std::string& robot_frame) {
  return [global, local, robot_frame](const ClearCostmapCommand& command) {
    if (!command.enabled) {
      return true;
    }
    auto& rsc = ayanami::components::RobotStateController::instance();
    const bool clear_global =
        command.affected_maps == "global" || command.affected_maps == "both";
    const bool clear_local =
        command.affected_maps == "local" || command.affected_maps == "both";
    bool ok = true;
    if (clear_global && global) {
      Eigen::Isometry3d p = Eigen::Isometry3d::Identity();
      ok = rsc.lookupTransform(global->globalFrame(), robot_frame,
                               ayanami::Timestamp::latest(), p) &&
           ok;
      ok = ayanami::costmap::clearCostmapArea(global, p,
                                              command.clear_radius) &&
           ok;
    }
    if (clear_local && local) {
      Eigen::Isometry3d p = Eigen::Isometry3d::Identity();
      ok = rsc.lookupTransform(local->globalFrame(), robot_frame,
                               ayanami::Timestamp::latest(), p) &&
           ok;
      ok = ayanami::costmap::clearCostmapArea(local, p,
                                              command.clear_radius) &&
           ok;
    }
    return ok;
  };
}

/// Sets the TF via RobotStateController: map<-base_link with the robot at (wx, wy).
void setRobot(double wx, double wy) {
  auto& rsc = ayanami::components::RobotStateController::instance();
  testTransforms().set(
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
  g_emit_cb = ayanami::interfaces::NavigationCallbacks::on_velocity_command
      .subscribe([](const TwistStamped& t) {
        std::lock_guard<std::mutex> lock(g_emit_mutex);
        g_emitted.push_back(t);
      });
}

void detachVelocityCapture() { g_emit_cb.reset(); }

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

TwistStamped lastEmitted() {
  std::lock_guard<std::mutex> lock(g_emit_mutex);
  return g_emitted.empty() ? TwistStamped{} : g_emitted.back();
}

/// Polls until the execution reaches the target state or times out.
bool waitExecutionState(RecoveryExecution& exec, int target,
                        int timeout_ms = 3000) {
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    if (exec.getState() == target) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  return exec.getState() == target;
}

ayanami::ModuleContext ctxOf(const std::string& json_name) {
  // Fixture json stem -> the module name (the param section convention).
  static const std::map<std::string, std::string> module_of = {
      {"spin.json", "costmap_spin_recovery"},
      {"backup.json", "costmap_back_up_recovery"},
      {"wait.json", "costmap_wait_recovery"},
      {"clear.json", "costmap_cleanup_recovery"}};
  return ayanami::ModuleContext{
      module_of.at(json_name),
      ayanami::Config(
          (std::filesystem::path(g_config_dir) / json_name).string())};
}

}  // namespace

// ---------- Single-publisher behavior ----------

TEST(CostmapRecoveryModulesTest, modulesDoNotPublishVelocity) {
  attachVelocityCapture();
  auto ctx = makeContext(makeWrapper(20, 0.5));
  auto clear = makeClearPort(ctx->globalCostmap(), nullptr, "base_link");
  auto spin = std::make_shared<CostmapSpinRecovery>();
  spin->configure(ctxOf("spin.json"), ctx->getSharedConstPtr(), clear);
  auto backup = std::make_shared<CostmapBackUpRecovery>();
  backup->configure(ctxOf("backup.json"), ctx->getSharedConstPtr(), clear);
  auto wait = std::make_shared<CostmapWaitRecovery>();
  wait->configure(ctxOf("wait.json"), ctx->getSharedConstPtr(), clear);

  spin->start();
  backup->start();
  wait->start();
  ayanami::RecoveryCycleData cycle;
  (void)spin->tick(cycle);
  (void)backup->tick(cycle);
  (void)wait->tick(cycle);

  std::lock_guard<std::mutex> lock(g_emit_mutex);
  EXPECT_TRUE(g_emitted.empty())
      << "recovery modules must not publish on_velocity_command directly";
  detachVelocityCapture();
}

TEST(CostmapRecoveryModulesTest, modulesSourceHasNoDirectPublish) {
  ASSERT_FALSE(g_modules_src.empty());
  const std::vector<std::string> files = {
      "costmap_spin_recovery/costmap_spin_recovery.cpp",
      "costmap_back_up_recovery/costmap_back_up_recovery.cpp",
      "costmap_wait_recovery/costmap_wait_recovery.cpp",
      "costmap_cleanup_recovery/costmap_cleanup_recovery.cpp"};
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

// ---------- CostmapSpinRecovery ----------

TEST(CostmapRecoveryModulesTest, spinCompletesWithSuccessViaExecution) {
  attachVelocityCapture();
  auto ctx = makeContext(makeWrapper(20, 0.5));
  auto rec = std::make_shared<CostmapSpinRecovery>();
  rec->configure(ctxOf("spin.json"), ctx->getSharedConstPtr(),
                 makeClearPort(ctx->globalCostmap(), nullptr, "base_link"));
  RecoveryExecution exec(rec);
  exec.setCyclePeriod(std::chrono::milliseconds(10));
  std::thread exec_th([&] { exec.runBlocking(); });
  ASSERT_TRUE(waitExecutionState(exec, RecoveryExecution::RECOVERY_DONE));
  EXPECT_EQ(exec.getError(), ErrorCode::NONE);
  exec_th.join();

  EXPECT_TRUE(anyNonZeroAngular());
  EXPECT_NEAR(lastEmitted().twist.angular.z(), 0.0, 1e-9);
  detachVelocityCapture();
}

TEST(CostmapRecoveryModulesTest, spinCancelEmitsZeroAndCanceledViaExecution) {
  attachVelocityCapture();
  auto ctx = makeContext(makeWrapper(20, 0.5));
  auto rec = std::make_shared<CostmapSpinRecovery>();
  rec->configure(ctxOf("spin.json"), ctx->getSharedConstPtr(),
                 makeClearPort(ctx->globalCostmap(), nullptr, "base_link"));
  RecoveryExecution exec(rec);
  exec.setCyclePeriod(std::chrono::milliseconds(10));
  std::thread exec_th([&] { exec.runBlocking(); });
  for (int i = 0; i < 100 && !anyNonZeroAngular(); ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  EXPECT_TRUE(anyNonZeroAngular());
  exec.cancel();
  ASSERT_TRUE(waitExecutionState(exec, RecoveryExecution::CANCELED));
  exec_th.join();
  EXPECT_NEAR(lastEmitted().twist.angular.z(), 0.0, 1e-9);
  detachVelocityCapture();
}

TEST(CostmapRecoveryModulesTest, spinVelocityExceedingMaxThetaIsClamped) {
  attachVelocityCapture();
  auto ctx = makeContext(makeWrapper(20, 0.5));
  auto rec = std::make_shared<CostmapSpinRecovery>();
  rec->configure(ctxOf("spin.json"), ctx->getSharedConstPtr(),
                 makeClearPort(ctx->globalCostmap(), nullptr, "base_link"));
  RecoveryExecution exec(rec);
  exec.setCyclePeriod(std::chrono::milliseconds(10));
  // max_w=0.3 > the injected cap max_vel_theta=0.1 -> clamped (over-limit
  // suggestions are clamped + warned, never zeroed).
  exec.setMaximumVelocity(1.0, 1.0, 0.1);
  std::thread exec_th([&] { exec.runBlocking(); });
  ASSERT_TRUE(waitExecutionState(exec, RecoveryExecution::RECOVERY_DONE));
  exec_th.join();
  EXPECT_TRUE(anyNonZeroAngular());
  EXPECT_NEAR(lastEmitted().twist.angular.z(), 0.0, 1e-9);
  {
    std::lock_guard<std::mutex> lock(g_emit_mutex);
    for (const auto& t : g_emitted) {
      EXPECT_LE(std::abs(t.twist.angular.z()), 0.1 + 1e-9)
          << "every published recovery command must respect max_vel_theta";
    }
  }
  detachVelocityCapture();
}

// ---------- CostmapBackUpRecovery ----------

TEST(CostmapRecoveryModulesTest, backUpCollisionFailsWithMessage) {
  auto global = makeWrapper(20, 0.5);
  {
    std::unique_lock<std::shared_mutex> lock(global->mutex());
    costmap_2d::Costmap2D* grid = global->get().getCostmap();
    // Robot at cell (4,4) (world 2.25,2.25); lethal on the two REAR
    // diagonal neighbors: inside the near belt, outside the contact belt
    // -- backing up leads INTO the near obstacles: refusal fires.
    grid->setCost(3, 3, costmap_2d::LETHAL_OBSTACLE);
    grid->setCost(3, 5, costmap_2d::LETHAL_OBSTACLE);
    global->updateMapUnlocked(2.25, 2.25, 0.0);
  }
  auto ctx = makeContext(global);
  setRobot(2.25, 2.25);

  auto rec = std::make_shared<CostmapBackUpRecovery>();
  rec->configure(ctxOf("backup.json"), ctx->getSharedConstPtr(),
                 makeClearPort(global, nullptr, "base_link"));

  rec->start();
  ayanami::RecoveryCycleData cycle;
  const ResultT<void> outcome = rec->tick(cycle);
  ASSERT_FALSE(outcome.ok);
  EXPECT_EQ(outcome.error, ErrorCode::RECOVERY_FAILED);
  EXPECT_NE(outcome.message.find("collision detected"), std::string::npos)
      << outcome.message;
}

TEST(CostmapRecoveryModulesTest, backUpContactAtRearEscapesForward) {
  auto global = makeWrapper(20, 0.5);
  {
    std::unique_lock<std::shared_mutex> lock(global->mutex());
    costmap_2d::Costmap2D* grid = global->get().getCostmap();
    // Rear contact belt one cell behind the robot cell (10,10).
    for (unsigned int y = 9; y <= 11; ++y) {
      grid->setCost(9, y, costmap_2d::LETHAL_OBSTACLE);
    }
    global->updateMapUnlocked(5.0, 5.0, 0.0);
  }
  auto ctx = makeContext(global);
  setRobot(5.0, 5.0);

  auto rec = std::make_shared<CostmapBackUpRecovery>();
  rec->configure(ctxOf("backup.json"), ctx->getSharedConstPtr(),
                 makeClearPort(global, nullptr, "base_link"));

  rec->start();
  ayanami::RecoveryCycleData cycle;
  const ResultT<void> outcome = rec->tick(cycle);
  ASSERT_TRUE(outcome.ok) << outcome.message;
  EXPECT_EQ(cycle.status, ayanami::RecoveryStatus::RUNNING);
  ASSERT_TRUE(cycle.velocity.has_value());
  // Rear contact: escape FORWARD (positive x).
  EXPECT_GT(cycle.velocity->twist.linear.x(), 0.0);
}

TEST(CostmapRecoveryModulesTest, backUpContactBoxedFailsOnStall) {
  auto global = makeWrapper(20, 0.5);
  {
    std::unique_lock<std::shared_mutex> lock(global->mutex());
    costmap_2d::Costmap2D* grid = global->get().getCostmap();
    for (unsigned int y = 9; y <= 11; ++y) {
      grid->setCost(9, y, costmap_2d::LETHAL_OBSTACLE);
      grid->setCost(11, y, costmap_2d::LETHAL_OBSTACLE);
      grid->setCost(7, y, costmap_2d::LETHAL_OBSTACLE);
      grid->setCost(13, y, costmap_2d::LETHAL_OBSTACLE);
    }
    global->updateMapUnlocked(5.0, 5.0, 0.0);
  }
  auto ctx = makeContext(global);
  setRobot(5.0, 5.0);

  auto rec = std::make_shared<CostmapBackUpRecovery>();
  rec->configure(ctxOf("backup.json"), ctx->getSharedConstPtr(),
                 makeClearPort(global, nullptr, "base_link"));
  rec->setCyclePeriod(0.01);

  rec->start();
  ayanami::RecoveryCycleData cycle;
  ResultT<void> outcome;
  bool failed = false;
  for (int i = 0; i < 30 && !failed; ++i) {
    outcome = rec->tick(cycle);
    if (!outcome.ok) {
      failed = true;
      EXPECT_EQ(outcome.error, ErrorCode::RECOVERY_FAILED);
      EXPECT_NE(outcome.message.find("escape stalled"), std::string::npos)
          << outcome.message;
    } else {
      EXPECT_EQ(cycle.status, ayanami::RecoveryStatus::RUNNING);
    }
  }
  EXPECT_TRUE(failed);
}

TEST(CostmapRecoveryModulesTest, backUpCompletesOnFreeMapViaExecution) {
  attachVelocityCapture();
  setRobot(5.0, 5.0);
  auto ctx = makeContext(makeWrapper(20, 0.5));
  auto rec = std::make_shared<CostmapBackUpRecovery>();
  rec->configure(ctxOf("backup.json"), ctx->getSharedConstPtr(),
                 makeClearPort(ctx->globalCostmap(), nullptr, "base_link"));
  RecoveryExecution exec(rec);
  exec.setCyclePeriod(std::chrono::milliseconds(10));
  // The escape verdict is physical progress: drift the robot backward
  // while the execution runs.
  std::atomic<bool> mover_run{true};
  std::thread mover([&] {
    double x = 5.0;
    while (mover_run.load()) {
      setRobot(x, 5.0);
      x -= 0.002;
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  });
  std::thread exec_th([&] { exec.runBlocking(); });
  ASSERT_TRUE(waitExecutionState(exec, RecoveryExecution::RECOVERY_DONE));
  EXPECT_EQ(exec.getError(), ErrorCode::NONE);
  exec_th.join();
  mover_run.store(false);
  mover.join();

  EXPECT_TRUE(anyNonZeroLinear());
  EXPECT_NEAR(lastEmitted().twist.linear.x(), 0.0, 1e-9);
  detachVelocityCapture();
}

// ---------- CostmapWaitRecovery ----------

TEST(CostmapRecoveryModulesTest, waitCompletesAfterDurationViaExecution) {
  auto ctx = makeContext(makeWrapper(20, 0.5));
  auto rec = std::make_shared<CostmapWaitRecovery>();
  rec->configure(ctxOf("wait.json"), ctx->getSharedConstPtr(),
                 makeClearPort(ctx->globalCostmap(), nullptr, "base_link"));
  RecoveryExecution exec(rec);
  exec.setCyclePeriod(std::chrono::milliseconds(10));
  const auto start = std::chrono::steady_clock::now();
  std::thread exec_th([&] { exec.runBlocking(); });
  ASSERT_TRUE(waitExecutionState(exec, RecoveryExecution::RECOVERY_DONE));
  const auto elapsed = std::chrono::duration<double>(
                           std::chrono::steady_clock::now() - start)
                           .count();
  EXPECT_EQ(exec.getError(), ErrorCode::NONE);
  exec_th.join();
  EXPECT_GE(elapsed, 0.05);
}

TEST(CostmapRecoveryModulesTest, waitCancelReturnsImmediatelyViaExecution) {
  auto ctx = makeContext(makeWrapper(20, 0.5));
  auto rec = std::make_shared<CostmapWaitRecovery>();
  rec->configure(ctxOf("wait.json"), ctx->getSharedConstPtr(),
                 makeClearPort(ctx->globalCostmap(), nullptr, "base_link"));
  RecoveryExecution exec(rec);
  exec.setCyclePeriod(std::chrono::milliseconds(10));
  std::thread exec_th([&] { exec.runBlocking(); });
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  exec.cancel();
  ASSERT_TRUE(waitExecutionState(exec, RecoveryExecution::CANCELED));
  exec_th.join();
  EXPECT_EQ(exec.getError(), ErrorCode::CANCELED);
}

// ---------- CostmapCleanupRecovery ----------

TEST(CostmapRecoveryModulesTest, clearCostmapClearsRadiusOnBothMaps) {
  auto global = makeWrapper(20, 0.5);
  auto local = makeWrapper(20, 0.5);
  auto global_obstacles =
      std::make_shared<costmap_2d::SimpleObstacleLayer>();
  auto local_obstacles =
      std::make_shared<costmap_2d::SimpleObstacleLayer>();
  global->get().addLayer(global_obstacles);
  local->get().addLayer(local_obstacles);
  {
    std::unique_lock<std::shared_mutex> lock(global->mutex());
    global_obstacles->markObstacles(*global->get().getCostmap(), 2.25, 2.25);
    global_obstacles->markObstacles(*global->get().getCostmap(), 2.75, 2.75);
    global_obstacles->markObstacles(*global->get().getCostmap(), 7.75, 7.75);
    global->updateMapUnlocked(2.25, 2.25, 0.0);
  }
  {
    std::unique_lock<std::shared_mutex> lock(local->mutex());
    local_obstacles->markObstacles(*local->get().getCostmap(), 2.25, 2.25);
    local_obstacles->markObstacles(*local->get().getCostmap(), 2.75, 2.75);
    local_obstacles->markObstacles(*local->get().getCostmap(), 7.75, 7.75);
    local->updateMapUnlocked(2.25, 2.25, 0.0);
  }
  auto ctx = makeContext(global, local);
  setRobot(2.25, 2.25);  // center of cell (4,4)

  auto rec = std::make_shared<CostmapCleanupRecovery>();
  rec->configure(ctxOf("clear.json"), ctx->getSharedConstPtr(),
                 makeClearPort(global, local, "base_link"));

  rec->start();
  ayanami::RecoveryCycleData cycle;
  const ResultT<void> outcome = rec->tick(cycle);
  ASSERT_TRUE(outcome.ok) << outcome.message;

  // Near the robot (within the radius) -> FREE_SPACE; distant marks stay.
  for (const auto& wrapper : {global, local}) {
    EXPECT_EQ(cellCost(wrapper, 4, 4), costmap_2d::FREE_SPACE);
    EXPECT_EQ(cellCost(wrapper, 5, 5), costmap_2d::FREE_SPACE);
    EXPECT_EQ(cellCost(wrapper, 15, 15), costmap_2d::LETHAL_OBSTACLE);
  }
}

// ---------- module.cpp factory symbols ----------

TEST(CostmapRecoveryModulesTest, moduleFactoriesCreateCorrectTypes) {
  ASSERT_FALSE(g_cleanup_so.empty());
  ASSERT_FALSE(g_spin_so.empty());
  ASSERT_FALSE(g_wait_so.empty());
  ASSERT_FALSE(g_backup_so.empty());

  EXPECT_NE(std::dynamic_pointer_cast<CostmapCleanupRecovery>(
                ayanami::ExtensionModule::loadModule(g_cleanup_so)),
            nullptr);
  EXPECT_NE(std::dynamic_pointer_cast<CostmapSpinRecovery>(
                ayanami::ExtensionModule::loadModule(g_spin_so)),
            nullptr);
  EXPECT_NE(std::dynamic_pointer_cast<CostmapWaitRecovery>(
                ayanami::ExtensionModule::loadModule(g_wait_so)),
            nullptr);
  EXPECT_NE(std::dynamic_pointer_cast<CostmapBackUpRecovery>(
                ayanami::ExtensionModule::loadModule(g_backup_so)),
            nullptr);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  if (argc > 1) g_cleanup_so = argv[1];
  if (argc > 2) g_spin_so = argv[2];
  if (argc > 3) g_wait_so = argv[3];
  if (argc > 4) g_backup_so = argv[4];
  if (argc > 5) g_modules_src = argv[5];

  // Temp config dir holding each module's own json (short durations ease
  // cycle-driven testing).
  const std::filesystem::path dir =
      std::filesystem::temp_directory_path() / "ayanami_costmap_recovery_test";
  std::filesystem::remove_all(dir);
  std::filesystem::create_directories(dir);
  g_config_dir = dir.string();
  {
    std::ofstream ofs(dir / "spin.json");
    ofs << R"({"costmap_spin_recovery": {
      "spin_dist": 0.03, "max_w": 0.3, "collision_radius": 0.3
    }})";
  }
  {
    std::ofstream ofs(dir / "backup.json");
    ofs << R"({"costmap_back_up_recovery": {
      "backup_dist": 0.02, "max_v": 0.2, "collision_radius": 0.3
    }})";
  }
  {
    std::ofstream ofs(dir / "wait.json");
    ofs << R"({"costmap_wait_recovery": {
      "wait_duration": 0.1
    }})";
  }
  {
    std::ofstream ofs(dir / "clear.json");
    ofs << R"({"costmap_cleanup_recovery": {
      "clear_radius": 1.0, "affected_maps": "both"
    }})";
  }
  ayanami::GlobalConfig::instance(dir.string(), /*override_path=*/true);

  return RUN_ALL_TESTS();
}
