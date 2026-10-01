// Recovery vs obstacle concurrency (map changes go through the single-writer command port).
//
// New architecture (locked direct-read map access, no snapshots/epochs): recovery modules never touch the mutable
// wrapper/layers; the clear recovery only constructs a typed ClearCostmapCommand and submits it via
// CostmapCommandPort; the single writer (an equivalent writer simulated here) holds the wrapper
// unique lock to clear layers + publish via updateMapUnlocked. Readers read the master grid directly under a shared lock.
//
// Assertions:
// - the clear-command-port path truly works: after submission the master grid near the robot -> FREE,
// distant cells stay LETHAL;
// - alternating multiple rounds with the obstacle writer, readers/writers each hold their lock (unique/shared), and every step reads back
// deterministic values under the lock (no tearing / no deadlock); the command submission count strictly increases per round (count algebra).

#include <gtest/gtest.h>

#include "test_transform_source.hpp"

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
#include <costmap_cleanup_recovery/costmap_cleanup_recovery.hpp>

using ayanami::testing::testTransforms;
namespace {

using ayanami::ErrorCode;
using ayanami::ResultT;
using ayanami::costmap::ClearCostmapCommand;
using ayanami::costmap::CostmapCommandPort;
using ayanami::costmap::CostmapMapContext;
using ayanami::modules::CostmapCleanupRecovery;

// 20x20 @0.5m, origin (0,0): the center of cell (4,4) = world (2.25, 2.25).
constexpr double k_robot_x = 2.25;
constexpr double k_robot_y = 2.25;

/// Equivalent writer command port: submit immediately performs the writer-level map clear (clearCostmapArea clears
/// layers + publishes via updateMapUnlocked under the write lock). Recovery modules only hold this interface and never touch the wrapper.
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

  // A counter incremented when the write (writer submit) completes: expresses monotone "version/epoch" semantics.
  std::atomic<int> submissions{0};

 private:
  std::shared_ptr<costmap_2d::CostmapWrapper> global;
  std::shared_ptr<costmap_2d::CostmapWrapper> local;
  ayanami::components::RobotStateController* rsc;
  std::string robot_frame;
};

/// Test-only sync gate: set wakes all waiters.
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
  testTransforms().set(
      "map", "base_link", ayanami::Timestamp::latest(),
      Eigen::Translation3d(wx, wy, 0.0) * Eigen::Quaterniond::Identity());
}

/// Single-writer path: mark the dynamic obstacle layer + recompute/publish via updateMapUnlocked under the unique lock.
void feedObstacles(
    const std::shared_ptr<costmap_2d::CostmapWrapper>& wrapper,
    const std::shared_ptr<costmap_2d_layers::SimpleObstacleLayer>& layer) {
  std::unique_lock<std::shared_mutex> lock(wrapper->mutex());
  const auto* grid = wrapper->get().getCostmap();
  layer->markObstacles(*grid, k_robot_x, k_robot_y);
  layer->markObstacles(*grid, 7.75, 7.75);
  wrapper->updateMapUnlocked(k_robot_x, k_robot_y, 0.0);
}

/// Reader path: directly read a single master-grid cell under the shared lock.
unsigned char cellCost(
    const std::shared_ptr<costmap_2d::CostmapWrapper>& wrapper, unsigned int x,
    unsigned int y) {
  std::shared_lock<std::shared_mutex> lock(wrapper->mutex());
  return wrapper->get().getCostmap()->getCost(x, y);
}

}  // namespace

// The command port truly works: after submitting the clear command the area near the robot is cleared back to FREE while distant
// cells stay LETHAL; and only the writer may trigger changes (recovery modules have no mutable access at all).
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
      ctx->robotFrame());

  // The command port is exposed via the map context (fetched as map->commandPort inside configure).
  ctx->setCommandPort(port);
  auto rec = std::make_shared<CostmapCleanupRecovery>();
  rec->configure(ayanami::ModuleContext{"costmap_cleanup_recovery",
                                        ayanami::Config("")},
                 ctx->getSharedConstPtr());

  // First feed obstacles via the writer: both near and far cells LETHAL.
  feedObstacles(local, local_layer);
  feedObstacles(global, global_layer);
  EXPECT_EQ(cellCost(local, 4, 4), costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(cellCost(local, 15, 15), costmap_2d::LETHAL_OBSTACLE);

  // Recovery: submit the typed clear command via the command port (only via the command port); distant obstacles are not cleared.
  rec->start();
  ayanami::RecoveryCycleData cycle;
  const ResultT<void> outcome = rec->tick(cycle);
  ASSERT_TRUE(outcome.ok) << outcome.message;
  EXPECT_EQ(port->submissions.load(), 1)
      << "clear must be submitted through the command port to the sole writer";

  // Local + global: near the robot -> FREE, distant cells stay LETHAL.
  EXPECT_EQ(cellCost(local, 4, 4), costmap_2d::FREE_SPACE);
  EXPECT_EQ(cellCost(local, 5, 5), costmap_2d::FREE_SPACE);
  EXPECT_EQ(cellCost(local, 15, 15), costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(cellCost(global, 4, 4), costmap_2d::FREE_SPACE);
  EXPECT_EQ(cellCost(global, 15, 15), costmap_2d::LETHAL_OBSTACLE);
}

// Clean deterministic alternation (clear command vs obstacle feed) over multiple rounds: after each feed reads return
// LETHAL and after each clear reads return FREE -- all writes go through the single writer (unique lock), readers/writers each hold
// their own lock, no tearing/no deadlock; the command submission count increases per round.
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
      ctx->robotFrame());

  // The command port is exposed via the map context (fetched as map->commandPort inside configure).
  ctx->setCommandPort(port);
  auto rec = std::make_shared<CostmapCleanupRecovery>();
  rec->configure(ayanami::ModuleContext{"costmap_cleanup_recovery",
                                        ayanami::Config("")},
                 ctx->getSharedConstPtr());

  constexpr int k_rounds = 16;
  std::atomic<int> bad_reads{0};
  std::atomic<int> bad_after_clear{0};
  std::atomic<int> rounds_done{0};

  std::vector<Gate> go_feed(k_rounds);
  std::vector<Gate> feed_done(k_rounds);

  // Obstacle writer thread: each round drops obstacles + publishes via the writer (unique lock) and reports completion.
  std::thread feeder([&] {
    for (int i = 0; i < k_rounds; ++i) {
      go_feed[i].waitFor(std::chrono::seconds(5));
      {
        std::unique_lock<std::shared_mutex> lock(local->mutex());
        local_layer->markObstacles(*local->get().getCostmap(), k_robot_x,
                                   k_robot_y);
        local->updateMapUnlocked(k_robot_x, k_robot_y, 0.0);
        // The read-back under the lock must see its own write (writer serialization, no tearing).
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
    // Recovery: submit the typed clear via the command port -> the single writer clears back to FREE (start resets
    // submitted; one command is resubmitted each round).
    rec->start();
    ayanami::RecoveryCycleData cycle;
  const ResultT<void> outcome = rec->tick(cycle);
    ASSERT_TRUE(outcome.ok) << outcome.message;
    if (cellCost(local, 4, 4) != costmap_2d::FREE_SPACE) {
      ++bad_after_clear;
    }
    ++rounds_done;
  }

  feeder.join();
  EXPECT_EQ(rounds_done.load(), k_rounds);
  EXPECT_EQ(port->submissions.load(), k_rounds)
      << "clear submission count must strictly grow per round (all commands via the sole writer)";
  EXPECT_EQ(bad_reads.load(), 0)
      << "writer read-back observed a concurrent clear tearing (writer serialization broken)";
  EXPECT_EQ(bad_after_clear.load(), 0) << "cell was not written back to LETHAL after clear";
  // No deadlock: both threads finished (the writer thread join returned + the main thread completed all k_rounds rounds).
}