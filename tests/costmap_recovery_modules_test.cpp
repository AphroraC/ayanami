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

#include "test_transform_source.hpp"

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
#include <costmap_cleanup_recovery/costmap_cleanup_recovery.hpp>
#include <costmap_spin_recovery/costmap_spin_recovery.hpp>
#include <costmap_wait_recovery/costmap_wait_recovery.hpp>
#include <ayanami/utility/config.hpp>
#include <ayanami/utility/load_module.hpp>

using ayanami::testing::testTransforms;
namespace {

using ayanami::ErrorCode;
using ayanami::ResultT;
using ayanami::costmap::ClearCostmapCommand;
using ayanami::costmap::CostmapCommandPort;
using ayanami::costmap::CostmapMapContext;
using ayanami::geometry_msgs::TwistStamped;
using ayanami::modules::CostmapBackUpRecovery;
using ayanami::modules::CostmapCleanupRecovery;
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
  std::thread exec_th([&] { exec.runBlocking(); });
  waitExecutionState(exec, ayanami::components::RecoveryExecution::RECOVERY_DONE);
  EXPECT_EQ(exec.getError(), ErrorCode::NONE);
  exec_th.join();

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
  std::thread exec_th([&] { exec.runBlocking(); });
  // Wait until at least one angular velocity frame is emitted, then cancel.
  for (int i = 0; i < 100 && !anyNonZeroAngular(); ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  EXPECT_TRUE(anyNonZeroAngular());
  exec.cancel();
  waitExecutionState(exec, ayanami::components::RecoveryExecution::CANCELED);
  exec_th.join();
  EXPECT_NEAR(lastEmitted().twist.angular.z(), 0.0, 1e-9);
  detachVelocityCapture();
}

// ---------- Single-publisher safety checks: finite + max_vel caps (over-limit zeroed) ----------

TEST(CostmapRecoveryModulesTest, spinVelocityExceedingMaxThetaIsClamped) {
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
  // max_w=0.3 > the injected cap max_vel_theta=0.1 -> commands are clamped
  // into the cap (policy: over-limit suggestions are clamped + warned, never
  // zeroed; only non-finite values are zeroed).
  exec.setMaximumVelocity(1.0, 1.0, 0.1);
  std::thread exec_th([&] { exec.runBlocking(); });
  waitExecutionState(exec, ayanami::components::RecoveryExecution::RECOVERY_DONE);
  exec_th.join();
  EXPECT_TRUE(anyNonZeroAngular())
      << "the in-progress spin suggestion (0.3 rad/s) must be clamped to the "
         "cap, not zeroed";
  EXPECT_NEAR(lastEmitted().twist.angular.z(), 0.0, 1e-9)
      << "the terminal state must still publish zero velocity";
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
    // The robot is at cell (4,4) (world 2.25,2.25); lethal on the two
    // REAR diagonal neighbors (centers 0.71m away): inside the near-obstacle
    // belt (<= 1.5*0.3 + 0.5) but OUTSIDE the contact belt (<= 0.15 + 0.5).
    // The robot is not touching anything, and backing up leads INTO the
    // near obstacles -- the direction-aware refusal fires.
    grid->setCost(3, 3, costmap_2d::LETHAL_OBSTACLE);
    grid->setCost(3, 5, costmap_2d::LETHAL_OBSTACLE);
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

// Contact on the REAR only (robot backing into an obstacle): the escape must
// pick the forward direction and emit a positive linear velocity instead of
// refusing forever (the old contract refused on any nearby lethal cell,
// deadlocking a touching robot against both the planner and the recoveries).
TEST(CostmapRecoveryModulesTest, backUpContactAtRearEscapesForward) {
  auto global = makeWrapper(20, 0.5);
  {
    std::unique_lock<std::shared_mutex> lock(global->mutex());
    costmap_2d::Costmap2D* grid = global->get().getCostmap();
    // Lethal belt one cell behind the robot cell (10,10): centers ~0.35m
    // away -- inside the contact-area exclusion, so not an escape blocker.
    for (unsigned int y = 9; y <= 11; ++y) {
      grid->setCost(9, y, costmap_2d::LETHAL_OBSTACLE);
    }
    global->updateMapUnlocked(5.0, 5.0, 0.0);
  }
  auto ctx = makeContext(global);
  setRobot(5.0, 5.0);

  auto rec = std::make_shared<CostmapBackUpRecovery>();
  rec->configure(ayanami::ModuleContext{"costmap_back_up_recovery",
                                        ayanami::Config(moduleConfig("backup.json"))},
                 ctx->getSharedConstPtr());

  rec->start();
  ayanami::RecoveryCycleData cycle;
  const ResultT<void> outcome = rec->tick(cycle);
  ASSERT_TRUE(outcome.ok) << outcome.message;
  EXPECT_EQ(cycle.status, ayanami::RecoveryStatus::RUNNING);
  ASSERT_TRUE(cycle.velocity.has_value());
  // Rear contact: escape FORWARD (positive x).
  EXPECT_GT(cycle.velocity->twist.linear.x(), 0.0);
}

// Contact with lethal all around (boxed): the escape is still ATTEMPTED --
// around a touching robot the map is the least trustworthy input, so the
// verdict is physical progress instead. The robot pose never moves here, so
// the run ends in the stall failure once the escape frames run out.
TEST(CostmapRecoveryModulesTest, backUpContactBoxedFailsOnStall) {
  auto global = makeWrapper(20, 0.5);
  {
    std::unique_lock<std::shared_mutex> lock(global->mutex());
    costmap_2d::Costmap2D* grid = global->get().getCostmap();
    for (unsigned int y = 9; y <= 11; ++y) {
      grid->setCost(9, y, costmap_2d::LETHAL_OBSTACLE);   // rear contact belt
      grid->setCost(11, y, costmap_2d::LETHAL_OBSTACLE);  // front contact belt
      grid->setCost(7, y, costmap_2d::LETHAL_OBSTACLE);   // deep behind
      grid->setCost(13, y, costmap_2d::LETHAL_OBSTACLE);  // deep ahead
    }
    global->updateMapUnlocked(5.0, 5.0, 0.0);
  }
  auto ctx = makeContext(global);
  setRobot(5.0, 5.0);

  auto rec = std::make_shared<CostmapBackUpRecovery>();
  rec->configure(ayanami::ModuleContext{"costmap_back_up_recovery",
                                        ayanami::Config(moduleConfig("backup.json"))},
                 ctx->getSharedConstPtr());
  rec->setCyclePeriod(0.01);  // frames = ceil(0.02 / 0.2 / 0.01) = 10

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
  // The pose never moved: the escape must end in the stall failure.
  EXPECT_TRUE(failed);
}

TEST(CostmapRecoveryModulesTest, backUpCompletesOnFreeMapViaExecution) {
  attachVelocityCapture();
  setRobot(5.0, 5.0);
  auto ctx = makeContext(makeWrapper(20, 0.5));
  auto rec = std::make_shared<CostmapBackUpRecovery>();
  rec->configure(ayanami::ModuleContext{"costmap_back_up_recovery",
                                        ayanami::Config(moduleConfig("backup.json"))},
                 ctx->getSharedConstPtr());
  ayanami::costmap::CostmapRecoveryExecution exec("back_up", rec, ctx);
  // The execution owns the cadence (production: CostmapNavigationServer reads
  // it from the module json's recovery.sim_cycle_dt); the module takes it from here.
  exec.setCyclePeriod(std::chrono::milliseconds(10));
  // The escape verdict is physical progress: drift the robot pose backward
  // (-x, the back-up direction) while the execution runs.
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
  waitExecutionState(exec, ayanami::components::RecoveryExecution::RECOVERY_DONE);
  EXPECT_EQ(exec.getError(), ErrorCode::NONE);
  exec_th.join();
  mover_run.store(false);
  mover.join();

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
  std::thread exec_th([&] { exec.runBlocking(); });
  waitExecutionState(exec, ayanami::components::RecoveryExecution::RECOVERY_DONE);
  const auto elapsed = std::chrono::duration<double>(
                           std::chrono::steady_clock::now() - start)
                           .count();
  EXPECT_EQ(exec.getError(), ErrorCode::NONE);
  exec_th.join();

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
  std::thread exec_th([&] { exec.runBlocking(); });
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  exec.cancel();
  waitExecutionState(exec, ayanami::components::RecoveryExecution::CANCELED);
  exec_th.join();
  EXPECT_EQ(exec.getError(), ErrorCode::CANCELED);
}

// ---------- CostmapCleanupRecovery (via the command port, no motion) ----------

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
  setRobot(2.25, 2.25);  // Center of cell (4,4)

  auto port = std::make_shared<TestWriterPort>(
      global, local, &ayanami::components::RobotStateController::instance(),
      ctx->robotFrame());
  // The command port is exposed via the map context (fetched as map->commandPort inside configure).
  ctx->setCommandPort(port);
  auto rec = std::make_shared<CostmapCleanupRecovery>();
  rec->configure(ayanami::ModuleContext{"costmap_cleanup_recovery",
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
  EXPECT_NE(std::dynamic_pointer_cast<CostmapCleanupRecovery>(
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
      "config_cleanup_recovery": "clear.json"
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
      "costmap_cleanup_recovery": {
        "clear_radius": 1.0, "affected_maps": "both"
      }})");
  ayanami::GlobalConfig::instance(dir.string(), /*override_path=*/true);

  return RUN_ALL_TESTS();
}
