#include <gtest/gtest.h>

#include "test_transform_source.hpp"

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include <Eigen/Geometry>

#include <costmap_2d/costmap_2d.hpp>
#include <costmap_2d/layered_costmap.hpp>
#include <costmap_server/costmap_wrapper.hpp>

#include <ayanami/abstract/components/robot_state_controller.hpp>
#include <ayanami/abstract/core/abstract_goal_checker.hpp>
#include <ayanami/costmap/components/costmap_map_context.hpp>
#include <ayanami/costmap/core/costmap_controller.hpp>
#include <ayanami/costmap/core/costmap_planner.hpp>
#include <ayanami/costmap/core/costmap_recovery.hpp>
#include <ayanami/costmap/core/costmap_smoother.hpp>
#include <ayanami/messages/failure.hpp>
#include <ayanami/types.hpp>
#include <ayanami/utility/config.hpp>

using ayanami::testing::testTransforms;
namespace {

using ayanami::ErrorCode;
using ayanami::ResultT;
using ayanami::RecoveryCycleData;
using ayanami::RecoveryStatus;
using ayanami::costmap::CostmapMapContext;
using ayanami::costmap::CostmapReader;

/// @brief Feed one robot pose edge into the RobotStateController singleton
/// (the context queries it directly -- the framework-wide TF surface).
void feedRobotPose(const std::string& frame, const std::string& robot_frame,
                   const Eigen::Vector3d& translation) {
  testTransforms().set(
      frame, robot_frame, ayanami::Timestamp::latest(),
      Eigen::Translation3d(translation) * Eigen::Quaterniond::Identity());
}

std::shared_ptr<costmap_2d::LayeredCostmap> makeGrid() {
  auto lm = std::make_shared<costmap_2d::LayeredCostmap>(
      "map", /*rolling_window=*/true, /*track_unknown=*/false);
  lm->resizeMap(10, 10, 0.5, 0.0, 0.0);
  return lm;
}

std::shared_ptr<costmap_2d::CostmapWrapper> makeWrapper(
    const std::shared_ptr<costmap_2d::LayeredCostmap>& lm) {
  return std::make_shared<costmap_2d::CostmapWrapper>(lm, "map");
}

/// @brief A 10x10 @0.5m wrapper in an arbitrary frame (origin at (0,0)).
std::shared_ptr<costmap_2d::CostmapWrapper> makeWrapperFrame(
    const std::string& frame) {
  auto lm = std::make_shared<costmap_2d::LayeredCostmap>(
      frame, /*rolling_window=*/true, /*track_unknown=*/false);
  lm->resizeMap(10, 10, 0.5, 0.0, 0.0);
  return std::make_shared<costmap_2d::CostmapWrapper>(lm, frame);
}

// ---- 4 Costmap* derivatives: implement the new configure(context, map) + lock-read view ----

class TestCostmapPlanner : public ayanami::costmap::CostmapPlanner {
 public:
  std::string init_name;
  uint32_t plan_calls{0};
  bool got_view{false};

  virtual ~TestCostmapPlanner() override = default;

  void configure(const ayanami::ModuleContext& context,
                 ayanami::costmap::CostmapMapContext::ConstPtr map) override {
    init_name = context.name;
    map_context = std::move(map);
  }

  ResultT<void> makePlan(
      const ayanami::geometry_msgs::PoseStamped&,
      const ayanami::geometry_msgs::PoseStamped&, double, ayanami::nav_msgs::Path& plan_out) override {
    ++plan_calls;
    got_view = map_context && map_context->readGlobal(
                                 [](const CostmapReader& r) { return r.valid(); });
    plan_out = ayanami::nav_msgs::Path{};
    return ResultT<void>::success();
  }

 private:
  ayanami::costmap::CostmapMapContext::ConstPtr map_context;
};

class TestCostmapSmoother : public ayanami::costmap::CostmapSmoother {
 public:
  std::string init_name;
  bool got_view{false};

  virtual ~TestCostmapSmoother() override = default;

  void configure(const ayanami::ModuleContext& context,
                 ayanami::costmap::CostmapMapContext::ConstPtr map) override {
    init_name = context.name;
    map_context = std::move(map);
  }

  ResultT<void> smoothPath(ayanami::nav_msgs::Path& path, double) override {
    got_view = map_context && map_context->readGlobal(
                                 [](const CostmapReader& r) { return r.valid(); });
    return ResultT<void>::success();
  }

 private:
  ayanami::costmap::CostmapMapContext::ConstPtr map_context;
};

class TestCostmapController : public ayanami::costmap::CostmapController {
 public:
  std::string init_name;
  bool got_view{false};

  virtual ~TestCostmapController() override = default;

  void configure(const ayanami::ModuleContext& context,
                 ayanami::costmap::CostmapMapContext::ConstPtr map) override {
    init_name = context.name;
    map_context = std::move(map);
  }

  ResultT<void> computeVelocityCommand(
      const ayanami::geometry_msgs::PoseStamped&,
      const ayanami::geometry_msgs::TwistStamped&, ayanami::geometry_msgs::TwistStamped& cmd) override {
    got_view = map_context && map_context->readLocal(
                                 [](const CostmapReader& r) { return r.valid(); });
    cmd = ayanami::geometry_msgs::TwistStamped{};
    return ResultT<void>::success();
  }
  bool setPlan(const ayanami::nav_msgs::Path&) override { return true; }

 private:
  ayanami::costmap::CostmapMapContext::ConstPtr map_context;
};

class TestCostmapRecovery : public ayanami::costmap::CostmapRecovery {
 public:
  std::string init_name;
  bool got_view{false};

  virtual ~TestCostmapRecovery() override = default;

  void configure(const ayanami::ModuleContext& context,
                 ayanami::costmap::CostmapMapContext::ConstPtr map) override {
    init_name = context.name;
    map_context = std::move(map);
  }

  ResultT<void> tick(RecoveryCycleData& cycle) override {
    got_view = map_context && map_context->readGlobal(
                                 [](const CostmapReader& r) { return r.valid(); });
    RecoveryCycleData data;
    data.status = RecoveryStatus::SUCCESS;
    cycle = std::move(data);
    return ResultT<void>::success();
  }

 private:
  ayanami::costmap::CostmapMapContext::ConstPtr map_context;
};

}  // namespace

// ---------- Frame-name getters ----------

TEST(CostmapMapContextTest, frameGettersReturnConfiguredFrames) {
  CostmapMapContext ctx;
  EXPECT_EQ(ctx.globalFrame(), "map");
  EXPECT_EQ(ctx.localFrame(), "odom");
  EXPECT_EQ(ctx.robotFrame(), "base_link");

  ctx.setFrames("map_a", "odom_a", "base_footprint");
  EXPECT_EQ(ctx.globalFrame(), "map_a");
  EXPECT_EQ(ctx.localFrame(), "odom_a");
  EXPECT_EQ(ctx.robotFrame(), "base_footprint");
}

TEST(CostmapMapContextTest, exposesConstSharedMapAndLockedReadAccess) {
  auto context = std::make_shared<CostmapMapContext>();
  context->setGlobalCostmap(makeWrapper(makeGrid()));

  std::shared_ptr<const CostmapMapContext> read_only =
      context->getSharedConstPtr();
  ASSERT_TRUE(read_only);
  EXPECT_EQ(read_only.get(), context.get());

  const auto size = read_only->readGlobal([](const CostmapReader& reader) {
    const costmap_2d::Costmap2D& grid = *reader.grid();
    return std::pair<unsigned int, unsigned int>{grid.getSizeInCellsX(),
                                                  grid.getSizeInCellsY()};
  });
  EXPECT_EQ(size.first, 10u);
  EXPECT_EQ(size.second, 10u);
}

// ---------- isCurrent ----------

TEST(CostmapMapContextTest, currentTrueWithoutWrappers) {
  CostmapMapContext ctx;
  EXPECT_TRUE(ctx.current());
}

TEST(CostmapMapContextTest, currentForwardsToWrappers) {
  auto global = makeWrapper(makeGrid());
  auto local = makeWrapper(makeGrid());

  CostmapMapContext ctx;
  ctx.setGlobalCostmap(global);
  ctx.setLocalCostmap(local);

  EXPECT_TRUE(ctx.current());
  global->setCurrent(false);
  EXPECT_FALSE(ctx.current());
  global->setCurrent(true);
  local->setCurrent(false);
  EXPECT_FALSE(ctx.current());
}

TEST(CostmapMapContextTest, isCurrentWithNullWrapperIgnoresIt) {
  auto local = makeWrapper(makeGrid());
  local->setCurrent(false);

  CostmapMapContext ctx;
  ctx.setLocalCostmap(local);
  EXPECT_FALSE(ctx.current());

  ctx.setLocalCostmap(nullptr);
  EXPECT_TRUE(ctx.current());
}

// ---------- Lock-read view (zero-copy direct read under a shared lock) ----------

TEST(CostmapMapContextTest, globalViewExposesGridData) {
  auto lm = makeGrid();
  lm->getCostmap()->setCost(1, 1, costmap_2d::LETHAL_OBSTACLE);
  auto global = makeWrapper(lm);
  CostmapMapContext ctx;
  ctx.setGlobalCostmap(global);

  bool valid = false;
  unsigned char cell = 0;
  std::string frame;
  ctx.readGlobal([&](const CostmapReader& reader) {
    valid = reader.valid();
    if (valid) {
      cell = reader.grid()->getCost(1, 1);
      frame = reader.frame();
    }
  });
  ASSERT_TRUE(valid);
  EXPECT_EQ(cell, costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(frame, "map");
}

TEST(CostmapMapContextTest, writerBlocksWhileReaderHoldsView) {
  auto lm = makeGrid();
  auto global = makeWrapper(lm);
  CostmapMapContext ctx;
  ctx.setGlobalCostmap(global);

  std::atomic<bool> reader_holding{false};
  std::atomic<bool> release_reader{false};
  std::atomic<bool> writer_done{false};

  std::thread reader([&] {
    // The read callback is the only way to hold the map lock, so the "reader
    // holds it while the writer waits" scenario is expressed by staying inside
    // the callback until the test says otherwise.
    ctx.readGlobal([&](const CostmapReader&) {
      reader_holding.store(true);
      while (!release_reader.load()) {
        std::this_thread::yield();
      }
    });
  });
  while (!reader_holding.load()) {
    std::this_thread::yield();
  }

  std::thread writer([&] {
    std::unique_lock<std::shared_mutex> lock(global->mutex());
    global->get().getCostmap()->setCost(2, 2, costmap_2d::LETHAL_OBSTACLE);
    writer_done.store(true);
  });

  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  EXPECT_FALSE(writer_done.load());

  release_reader.store(true);
  reader.join();
  writer.join();
  EXPECT_TRUE(writer_done.load());

  const auto cost = ctx.readGlobal([](const CostmapReader& reader) {
    const costmap_2d::Costmap2D& grid = *reader.grid();
    return grid.getCost(2, 2);
  });
  EXPECT_EQ(cost, costmap_2d::LETHAL_OBSTACLE);
}

TEST(CostmapMapContextTest, localReaderWithoutWrapperIsInvalid) {
  CostmapMapContext ctx;
  // A missing map still runs the callback, with an invalid reader: the caller
  // decides, rather than the framework silently skipping the read.
  bool called = false;
  ctx.readLocal([&](const CostmapReader& reader) {
    called = true;
    EXPECT_FALSE(reader.valid());
    EXPECT_EQ(reader.grid(), nullptr);
  });
  EXPECT_TRUE(called);
}

// ---------- robot state query (single facade = RobotStateController; the map context no longer exposes pose queries) ----------

TEST(CostmapMapContextTest, robotStateQueryGoesThroughRobotStateController) {
  auto& rsc = ayanami::components::RobotStateController::instance();
  rsc.initialize("map", "map", "base_link", 0.25);
  testTransforms().set(
      "map", "base_link", ayanami::Timestamp::latest(),
      Eigen::Translation3d(1.0, 2.0, 0.0) * Eigen::Quaterniond::Identity());

  const auto pose = rsc.getRobotPose();
  EXPECT_EQ(pose.header.frame_id, "map");
  EXPECT_DOUBLE_EQ(1.0, pose.pose.position.x());
  EXPECT_DOUBLE_EQ(2.0, pose.pose.position.y());
}

TEST(CostmapMapContextTest, viewResolvesRobotPose) {
  // Distinct frame names avoid interfering with latest entries under the same frame keys.
  feedRobotPose("pose_map", "pose_base", Eigen::Vector3d(3.0, 4.0, 0.0));

  auto global = std::make_shared<costmap_2d::CostmapWrapper>(
      std::make_shared<costmap_2d::LayeredCostmap>(
          "pose_map", /*rolling_window=*/true, /*track_unknown=*/false),
      "pose_map");
  global->get().resizeMap(10, 10, 0.5, 0.0, 0.0);
  CostmapMapContext ctx;
  ctx.setFrames("pose_map", "pose_odom", "pose_base");
  ctx.setGlobalCostmap(global);

  double pose_x = 0.0;
  double pose_y = 0.0;
  bool has_pose = false;
  ctx.readGlobal([&](const CostmapReader& reader) {
    has_pose = reader.hasRobotPose();
    if (has_pose) {
      pose_x = reader.robot_pose().pose.position.x();
      pose_y = reader.robot_pose().pose.position.y();
    }
  });
  ASSERT_TRUE(has_pose);
  EXPECT_DOUBLE_EQ(pose_x, 3.0);
  EXPECT_DOUBLE_EQ(pose_y, 4.0);
}

TEST(CostmapMapContextTest, viewRobotPoseStampedInOwnFrame) {
  // Distinct frame names avoid interference with other cases. Each map's
  // view resolves the robot pose in the wrapper's OWN frame (global = mapF,
  // local = locF). TF chain: locF <- mapF <- baseF (fast_tf mounts new
  // parents through existing intermediate frames only).
  auto& rsc = ayanami::components::RobotStateController::instance();
  testTransforms().set("mapF", "baseF", ayanami::Timestamp::latest(),
                   Eigen::Translation3d(3.0, 4.0, 0.0) *
                       Eigen::Quaterniond::Identity());
  testTransforms().set("locF", "mapF", ayanami::Timestamp::latest(),
                   Eigen::Translation3d(2.0, 0.0, 0.0) *
                       Eigen::Quaterniond::Identity());

  CostmapMapContext ctx;
  ctx.setFrames("mapF", "locF", "baseF");
  ctx.setLocalCostmap(makeWrapperFrame("locF"));
  ctx.setGlobalCostmap(makeWrapperFrame("mapF"));

  std::string local_frame;
  std::string local_pose_frame;
  double local_x = 0.0;
  double local_y = 0.0;
  ctx.readLocal([&](const CostmapReader& reader) {
    ASSERT_TRUE(reader.valid());
    local_frame = reader.frame();
    ASSERT_TRUE(reader.hasRobotPose());
    local_pose_frame = reader.robot_pose().header.frame_id;
    local_x = reader.robot_pose().pose.position.x();
    local_y = reader.robot_pose().pose.position.y();
  });
  EXPECT_EQ(local_frame, "locF");
  EXPECT_EQ(local_pose_frame, "locF");
  EXPECT_DOUBLE_EQ(local_x, 5.0);
  EXPECT_DOUBLE_EQ(local_y, 4.0);
  std::string global_frame;
  std::string global_pose_frame;
  double global_x = 0.0;
  ctx.readGlobal([&](const CostmapReader& reader) {
    ASSERT_TRUE(reader.valid());
    global_frame = reader.frame();
    ASSERT_TRUE(reader.hasRobotPose());
    global_pose_frame = reader.robot_pose().header.frame_id;
    global_x = reader.robot_pose().pose.position.x();
  });
  EXPECT_EQ(global_frame, "mapF");
  EXPECT_EQ(global_pose_frame, "mapF");
  EXPECT_DOUBLE_EQ(global_x, 3.0);
}

TEST(CostmapCoreTest, costmapPlannerInitializeAndViewMakePlan) {
  auto planner = std::make_shared<TestCostmapPlanner>();

  ayanami::abstract::AbstractPlanner::Ptr base = planner;
  auto cast =
      std::dynamic_pointer_cast<ayanami::costmap::CostmapPlanner>(base);
  ASSERT_NE(cast, nullptr);

  auto ctx = std::make_shared<CostmapMapContext>();
  ctx->setGlobalCostmap(makeWrapper(makeGrid()));
  cast->configure(ayanami::ModuleContext{"test_planner", ayanami::Config("")},
                  ctx->getSharedConstPtr());
  EXPECT_EQ(planner->init_name, "test_planner");

  ayanami::geometry_msgs::PoseStamped start;
  ayanami::geometry_msgs::PoseStamped goal;
  ayanami::nav_msgs::Path plan;
  auto result = cast->makePlan(start, goal, 0.1, plan);
  EXPECT_TRUE(result.ok);
  EXPECT_EQ(result.error, ErrorCode::NONE);
  EXPECT_EQ(planner->plan_calls, 1u);
  EXPECT_TRUE(planner->got_view);
}

TEST(CostmapCoreTest, costmapSmootherInitializeAndViewSmoothPath) {
  auto smoother = std::make_shared<TestCostmapSmoother>();

  ayanami::abstract::AbstractSmoother::Ptr base = smoother;
  auto cast =
      std::dynamic_pointer_cast<ayanami::costmap::CostmapSmoother>(base);
  ASSERT_NE(cast, nullptr);

  auto ctx = std::make_shared<CostmapMapContext>();
  ctx->setGlobalCostmap(makeWrapper(makeGrid()));
  cast->configure(ayanami::ModuleContext{"test_smoother", ayanami::Config("")},
                  ctx->getSharedConstPtr());
  EXPECT_EQ(smoother->init_name, "test_smoother");

  ayanami::nav_msgs::Path path;
  auto result = cast->smoothPath(path, 10.0);
  EXPECT_TRUE(result.ok);
  EXPECT_EQ(result.error, ErrorCode::NONE);
  EXPECT_TRUE(smoother->got_view);
}

TEST(CostmapCoreTest, costmapControllerInitializeAndViewCompute) {
  auto controller = std::make_shared<TestCostmapController>();

  ayanami::abstract::AbstractController::Ptr base = controller;
  auto cast =
      std::dynamic_pointer_cast<ayanami::costmap::CostmapController>(base);
  ASSERT_NE(cast, nullptr);

  auto ctx = std::make_shared<CostmapMapContext>();
  ctx->setLocalCostmap(makeWrapper(makeGrid()));
  cast->configure(ayanami::ModuleContext{"test_controller", ayanami::Config("")},
                  ctx->getSharedConstPtr());
  EXPECT_EQ(controller->init_name, "test_controller");

  ayanami::geometry_msgs::PoseStamped pose;
  ayanami::geometry_msgs::TwistStamped velocity;
  ayanami::geometry_msgs::TwistStamped cmd;
  auto result = cast->computeVelocityCommand(pose, velocity, cmd);
  EXPECT_TRUE(result.ok);
  EXPECT_EQ(result.error, ErrorCode::NONE);
  EXPECT_TRUE(controller->got_view);
}

TEST(CostmapCoreTest, costmapRecoveryInitializeAndViewRun) {
  auto recovery = std::make_shared<TestCostmapRecovery>();

  ayanami::abstract::AbstractRecovery::Ptr base = recovery;
  auto cast =
      std::dynamic_pointer_cast<ayanami::costmap::CostmapRecovery>(base);
  ASSERT_NE(cast, nullptr);

  auto ctx = std::make_shared<CostmapMapContext>();
  ctx->setGlobalCostmap(makeWrapper(makeGrid()));
  cast->configure(ayanami::ModuleContext{"test_recovery", ayanami::Config("")},
                  ctx->getSharedConstPtr());
  EXPECT_EQ(recovery->init_name, "test_recovery");

  RecoveryCycleData cycle;
  auto result = cast->tick(cycle);
  EXPECT_TRUE(result.ok);
  EXPECT_EQ(cycle.status, RecoveryStatus::SUCCESS);
  EXPECT_TRUE(recovery->got_view);
}

// ---------- CostmapMapContext has no capability / lock ----------

TEST(CostmapMapContextTest, costmapInterfaceOnly) {
  CostmapMapContext ctx;
  // Family-level surface: frame names / freshness -- no capability enum,
  // no locks, no robot-state queries on the abstract interface.
  EXPECT_EQ(ctx.globalFrame(), "map");
  EXPECT_EQ(ctx.localFrame(), "odom");
  EXPECT_EQ(ctx.robotFrame(), "base_link");
  EXPECT_TRUE(ctx.current());

  // Isolated frame names avoid interference with the map/base_link TF injected by other cases.
  ctx.setFrames("isolated_map", "isolated_odom", "isolated_base");
}
// ---------- isPathValid: map-family collision query capability (responsibility table "is the path stale") ----------

namespace {

ayanami::nav_msgs::Path makePathAt(std::vector<std::pair<double, double>> pts) {
  ayanami::nav_msgs::Path path;
  path.header.frame_id = "map";
  for (const auto& [x, y] : pts) {
    ayanami::geometry_msgs::PoseStamped p;
    p.header.frame_id = "map";
    p.pose.position = Eigen::Vector3d(x, y, 0.0);
    path.poses.push_back(p);
  }
  return path;
}

}  // namespace

TEST(CostmapMapContextTest, isPathValidRejectsEmptyUnknownLethalAndOutOfBounds) {
  auto lm = makeGrid();  // 10x10 @ 0.5m, origin (0,0) -> covers 0..5m
  auto* grid = lm->getCostmap();
  // world (1.0,1.0) -> cell (2,2) set lethal; cell (3,2) set to NO_INFORMATION.
  grid->setCost(2, 2, costmap_2d::LETHAL_OBSTACLE);
  grid->setCost(3, 2, costmap_2d::NO_INFORMATION);

  CostmapMapContext ctx;
  ctx.setGlobalCostmap(makeWrapper(lm));

  // An empty path is invalid.
  EXPECT_FALSE(ctx.isPathValid(makePathAt({})));
  // A free-space path is valid.
  EXPECT_TRUE(ctx.isPathValid(makePathAt({{0.5, 0.5}, {2.0, 0.5}})));
  // A lethal point is invalid.
  EXPECT_FALSE(ctx.isPathValid(makePathAt({{0.5, 0.5}, {1.0, 1.0}})));
  // An unknown point is invalid.
  EXPECT_FALSE(ctx.isPathValid(makePathAt({{1.5, 1.0}})));  // world(1.5,1.0)->cell(3,2)
  // An out-of-bounds point is invalid.
  EXPECT_FALSE(ctx.isPathValid(makePathAt({{9.0, 9.0}})));
  // High inflation (non-lethal) cost is not treated as invalid (the controller layer decides).
  grid->setCost(4, 4, 200);
  EXPECT_TRUE(ctx.isPathValid(makePathAt({{2.0, 2.0}})));
}

// ---------- path validation gates on the global+local fused overlay ----------

TEST(CostmapMapContextTest, isPathValidFusesLocalOverlay) {
  CostmapMapContext ctx;
  // Same-frame local (identity transform path).
  ctx.setFrames("map_p", "map_p", "base_p");

  // global: 10x10 @0.5m origin (0,0) covering [0,5)x[0,5), all free.
  auto global_lm = std::make_shared<costmap_2d::LayeredCostmap>(
      "map_p", false, false);
  global_lm->resizeMap(10, 10, 0.5, 0.0, 0.0);
  auto global =
      std::make_shared<costmap_2d::CostmapWrapper>(global_lm, "map_p");

  // local: a SMALL rolling window covering [0,2)x[0,2) only (cells 0..3);
  // the two grids share the frame so the overlay transform is identity.
  auto local_lm = std::make_shared<costmap_2d::LayeredCostmap>(
      "map_p", true, false);
  local_lm->resizeMap(4, 4, 0.5, 0.0, 0.0);
  auto local =
      std::make_shared<costmap_2d::CostmapWrapper>(local_lm, "map_p");

  ctx.setGlobalCostmap(global);
  ctx.setLocalCostmap(local);

  // 1. A fresh local mark INSIDE the local window gates the near path
  // segment: world (1.5,1.0) -> local cell (3,2) lethal, global free
  // (overlay planning keeps fresh dynamic marks in the validity verdict).
  local_lm->getCostmap()->setCost(3, 2, costmap_2d::LETHAL_OBSTACLE);
  EXPECT_FALSE(ctx.isPathValid(makePathAt({{1.5, 1.0}})));

  // 2. A global lethal gates regardless of the local window: world
  // (3.5,1.0) -> global cell (7,2) lethal while the point lies outside
  // the local window ([0,2)x[0,2)) -- the global grid gates alone there.
  global_lm->getCostmap()->setCost(7, 2, costmap_2d::LETHAL_OBSTACLE);
  EXPECT_FALSE(ctx.isPathValid(makePathAt({{3.5, 1.0}})));

  // 3. A local mark far OUTSIDE the local window does not gate: with the
  // global mark lifted, the far point (3.5,1.0) is valid again -- the
  // local lethal at (3,2) covers a different part of the map and must not
  // leak into the far segment's verdict.
  global_lm->getCostmap()->setCost(7, 2, costmap_2d::FREE_SPACE);
  EXPECT_TRUE(ctx.isPathValid(makePathAt({{3.5, 1.0}})));

  // 4. Clearing the local mark re-validates the near segment (the fusion
  // tracks the local map's own content).
  local_lm->getCostmap()->setCost(3, 2, costmap_2d::FREE_SPACE);
  EXPECT_TRUE(ctx.isPathValid(makePathAt({{1.5, 1.0}})));
}

TEST(CostmapMapContextTest, isPathValidDegeneratesWithoutMap) {
  CostmapMapContext ctx;  // No global wrapper
  EXPECT_FALSE(ctx.isPathValid(makePathAt({})));
  EXPECT_TRUE(ctx.isPathValid(makePathAt({{1.0, 1.0}})));
}

// ---------- robot-adjacent exemption: where the robot already is must not
// fail validation (a band-grazing robot would otherwise fail every future
// check and every replan -- the stop-into-recovery deadlock) ----------

TEST(CostmapMapContextTest, isPathValidExemptsRobotAdjacentPrefix) {
  auto lm = makeGrid();  // 10x10 @ 0.5m, origin (0,0)
  auto* grid = lm->getCostmap();
  // The robot stands on cell (1,1); world (0.75,0.5) is cell (1,1) too.
  grid->setCost(1, 1, costmap_2d::LETHAL_OBSTACLE);
  // A lethal cell 2 cells east: world (1.0,0.5) -> cell (2,1), 0.5m from the
  // robot center -- beyond the body radius, still gated.
  grid->setCost(2, 1, costmap_2d::LETHAL_OBSTACLE);

  CostmapMapContext ctx;
  ctx.setFrames("map", "odom", "base_link");
  ctx.setGlobalCostmap(makeWrapper(lm));
  // 16-gon circle approximation of robot_radius 0.3 -> inscribed ~0.294
  // (the configureCoreMaps footprint shape).
  costmap_2d::Footprint footprint;
  for (int i = 0; i < 16; ++i) {
    const double angle = 6.28318530717958647692 * i / 16.0;
    footprint.push_back({0.3 * std::cos(angle), 0.3 * std::sin(angle), 0.0});
  }
  ctx.setFootprint(footprint);
  // map<-base_link: the robot pose resolves to world (0.5, 0.5).
  Eigen::Isometry3d map_from_base = Eigen::Isometry3d::Identity();
  map_from_base.translation() = Eigen::Vector3d(0.5, 0.5, 0.0);
  testTransforms().set("map", "base_link", ayanami::Timestamp::latest(),
                       map_from_base);

  // 1. A point AT the robot on a lethal cell is exempt (the deadlock case).
  EXPECT_TRUE(ctx.isPathValid(makePathAt({{0.5, 0.5}})));
  // 2. The prefix within the body radius is exempt: (0.75,0.5) sits on the
  // same lethal cell, 0.25m from the robot.
  EXPECT_TRUE(ctx.isPathValid(makePathAt({{0.5, 0.5}, {0.75, 0.5}})));
  // 3. Beyond the body radius the check is full-strength again.
  EXPECT_FALSE(ctx.isPathValid(makePathAt({{0.5, 0.5}, {1.0, 0.5}})));
  // 4. The exemption needs the robot pose: an unresolvable robot frame
  // degrades to the conservative full-path check.
  auto lm_no_robot = makeGrid();
  lm_no_robot->getCostmap()->setCost(1, 1, costmap_2d::LETHAL_OBSTACLE);
  CostmapMapContext ctx_no_robot;
  ctx_no_robot.setFrames("map", "odom", "base_missing");
  ctx_no_robot.setGlobalCostmap(makeWrapper(lm_no_robot));
  ctx_no_robot.setFootprint(footprint);
  EXPECT_FALSE(ctx_no_robot.isPathValid(makePathAt({{0.5, 0.5}})));
}

