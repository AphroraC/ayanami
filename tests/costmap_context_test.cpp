#include <gtest/gtest.h>

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

namespace {

using ayanami::ErrorCode;
using ayanami::ResultT;
using ayanami::RecoveryCycleData;
using ayanami::RecoveryStatus;
using ayanami::costmap::CostmapMapContext;
using ayanami::costmap::CostmapView;

/// @brief Wire the context's transform lookup to the RobotStateController
/// singleton (production wiring is the same shape, injected by the host).
void wireRobotState(CostmapMapContext& ctx) {
  ctx.setTransformLookup(
      [](const std::string& target, const std::string& source,
         ayanami::Timestamp stamp, Eigen::Isometry3d& transform) {
        return ayanami::components::RobotStateController::instance()
            .lookupTransform(target, source, stamp, transform);
      });
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

// ---- 4 Costmap* derivatives: implement the new configure(context, map) + lock-read view ----

class TestCostmapPlanner : public ayanami::costmap::CostmapPlanner {
 public:
  std::string init_name;
  uint32_t plan_calls{0};
  bool got_view{false};

  virtual ~TestCostmapPlanner() override = default;

  void configure(const ayanami::PluginContext& context,
                 ayanami::costmap::CostmapMapContext::ConstPtr map) override {
    init_name = context.name;
    map_context = std::move(map);
  }

  ResultT<void> makePlan(
      const ayanami::geometry_msgs::PoseStamped&,
      const ayanami::geometry_msgs::PoseStamped&, double, ayanami::nav_msgs::Path& plan_out) override {
    ++plan_calls;
    got_view = map_context && map_context->lockGlobal().valid();
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

  void configure(const ayanami::PluginContext& context,
                 ayanami::costmap::CostmapMapContext::ConstPtr map) override {
    init_name = context.name;
    map_context = std::move(map);
  }

  ResultT<void> smoothPath(ayanami::nav_msgs::Path& path, double) override {
    got_view = map_context && map_context->lockGlobal().valid();
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

  void configure(const ayanami::PluginContext& context,
                 ayanami::costmap::CostmapMapContext::ConstPtr map) override {
    init_name = context.name;
    map_context = std::move(map);
  }

  ResultT<void> computeVelocityCommand(
      const ayanami::geometry_msgs::PoseStamped&,
      const ayanami::geometry_msgs::TwistStamped&, ayanami::geometry_msgs::TwistStamped& cmd) override {
    got_view = map_context && map_context->lockLocal().valid();
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

  void configure(const ayanami::PluginContext& context,
                 ayanami::costmap::CostmapMapContext::ConstPtr map) override {
    init_name = context.name;
    map_context = std::move(map);
  }

  ResultT<void> tick(RecoveryCycleData& cycle) override {
    got_view = map_context && map_context->lockGlobal().valid();
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
  EXPECT_EQ(ctx.getMapFrame(), "map");
  EXPECT_EQ(ctx.getRobotFrame(), "base_link");

  ctx.setFrames("odom", "base_footprint");
  EXPECT_EQ(ctx.getMapFrame(), "odom");
  EXPECT_EQ(ctx.getRobotFrame(), "base_footprint");
}

TEST(CostmapMapContextTest, exposesConstSharedMapAndLockedReadAccess) {
  auto context = std::make_shared<CostmapMapContext>();
  context->setGlobalCostmap(makeWrapper(makeGrid()));

  std::shared_ptr<const CostmapMapContext> read_only =
      context->getSharedConstPtr();
  ASSERT_TRUE(read_only);
  EXPECT_EQ(read_only.get(), context.get());

  const auto size = read_only->readGlobal([](const costmap_2d::Costmap2D& grid) {
    return std::pair<unsigned int, unsigned int>{grid.getSizeInCellsX(),
                                                  grid.getSizeInCellsY()};
  });
  EXPECT_EQ(size.first, 10u);
  EXPECT_EQ(size.second, 10u);
}

// ---------- isCurrent ----------

TEST(CostmapMapContextTest, isCurrentTrueWithoutWrappers) {
  CostmapMapContext ctx;
  EXPECT_TRUE(ctx.isCurrent());
}

TEST(CostmapMapContextTest, isCurrentForwardsToWrappers) {
  auto global = makeWrapper(makeGrid());
  auto local = makeWrapper(makeGrid());

  CostmapMapContext ctx;
  ctx.setGlobalCostmap(global);
  ctx.setLocalCostmap(local);

  EXPECT_TRUE(ctx.isCurrent());
  global->setCurrent(false);
  EXPECT_FALSE(ctx.isCurrent());
  global->setCurrent(true);
  local->setCurrent(false);
  EXPECT_FALSE(ctx.isCurrent());
}

TEST(CostmapMapContextTest, isCurrentWithNullWrapperIgnoresIt) {
  auto local = makeWrapper(makeGrid());
  local->setCurrent(false);

  CostmapMapContext ctx;
  ctx.setLocalCostmap(local);
  EXPECT_FALSE(ctx.isCurrent());

  ctx.setLocalCostmap(nullptr);
  EXPECT_TRUE(ctx.isCurrent());
}

// ---------- Lock-read view (zero-copy direct read under a shared lock) ----------

TEST(CostmapMapContextTest, globalViewExposesGridData) {
  auto lm = makeGrid();
  lm->getCostmap()->setCost(1, 1, costmap_2d::LETHAL_OBSTACLE);
  auto global = makeWrapper(lm);
  CostmapMapContext ctx;
  ctx.setGlobalCostmap(global);

  const CostmapView view = ctx.lockGlobal();
  ASSERT_TRUE(view.valid());
  EXPECT_EQ(view.grid()->getCost(1, 1), costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(view.frame(), "map");
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
    const CostmapView view = ctx.lockGlobal();
    reader_holding.store(true);
    while (!release_reader.load()) {
      std::this_thread::yield();
    }
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

  const auto cost = ctx.readGlobal([](const costmap_2d::Costmap2D& grid) {
    return grid.getCost(2, 2);
  });
  EXPECT_EQ(cost, costmap_2d::LETHAL_OBSTACLE);
}

TEST(CostmapMapContextTest, localViewWithoutWrapperIsInvalid) {
  CostmapMapContext ctx;
  const CostmapView view = ctx.lockLocal();
  EXPECT_FALSE(view.valid());
}

// ---------- robot state query (single facade = RobotStateController; the map context no longer exposes pose queries) ----------

TEST(CostmapMapContextTest, robotStateQueryGoesThroughRobotStateController) {
  auto& rsc = ayanami::components::RobotStateController::instance();
  rsc.initialize("map", "map", "base_link", 0.25);
  rsc.setTransform(
      "map", "base_link", ayanami::Timestamp::latest(),
      Eigen::Translation3d(1.0, 2.0, 0.0) * Eigen::Quaterniond::Identity());

  const auto pose = rsc.getRobotPose();
  EXPECT_EQ(pose.header.frame_id, "map");
  EXPECT_DOUBLE_EQ(1.0, pose.pose.position.x());
  EXPECT_DOUBLE_EQ(2.0, pose.pose.position.y());
}

TEST(CostmapMapContextTest, viewResolvesRobotPose) {
  auto& rsc = ayanami::components::RobotStateController::instance();
  // Use distinct timestamps to avoid interfering with latest entries under the same frame keys.
  rsc.setTransform(
      "map", "base_link", ayanami::timestampFromSeconds(5.0),
      Eigen::Translation3d(3.0, 4.0, 0.0) * Eigen::Quaterniond::Identity());

  auto global = makeWrapper(makeGrid());
  CostmapMapContext ctx;
  wireRobotState(ctx);
  ctx.setGlobalCostmap(global);

  const CostmapView view = ctx.lockGlobal();
  ASSERT_TRUE(view.has_robot_pose());
  EXPECT_DOUBLE_EQ(view.robot_pose().pose.position.x(), 3.0);
  EXPECT_DOUBLE_EQ(view.robot_pose().pose.position.y(), 4.0);
}

TEST(CostmapMapContextTest, viewRobotPoseStampedInMapFrame) {
  auto& rsc = ayanami::components::RobotStateController::instance();
  // Distinct frame names avoid interference with other cases.
  rsc.setTransform(
      "mapF", "baseF", ayanami::Timestamp::latest(),
      Eigen::Translation3d(3.0, 4.0, 0.0) * Eigen::Quaterniond::Identity());

  CostmapMapContext ctx;
  wireRobotState(ctx);
  ctx.setFrames("mapF", "baseF");
  ctx.setLocalCostmap(makeWrapper(makeGrid()));
  ctx.setGlobalCostmap(makeWrapper(makeGrid()));

  // Both views share the map frame: the pose must be stamped with it.
  const CostmapView lview = ctx.lockLocal();
  ASSERT_TRUE(lview.valid());
  EXPECT_EQ(lview.frame(), "mapF");
  ASSERT_TRUE(lview.has_robot_pose());
  EXPECT_EQ(lview.robot_pose().header.frame_id, "mapF");
  const CostmapView gview = ctx.lockGlobal();
  ASSERT_TRUE(gview.has_robot_pose());
  EXPECT_EQ(gview.robot_pose().header.frame_id, "mapF");
}

TEST(CostmapCoreTest, costmapPlannerInitializeAndViewMakePlan) {
  auto planner = std::make_shared<TestCostmapPlanner>();

  ayanami::abstract::AbstractPlanner::Ptr base = planner;
  auto cast =
      std::dynamic_pointer_cast<ayanami::costmap::CostmapPlanner>(base);
  ASSERT_NE(cast, nullptr);

  auto ctx = std::make_shared<CostmapMapContext>();
  ctx->setGlobalCostmap(makeWrapper(makeGrid()));
  cast->configure(ayanami::PluginContext{"test_planner", ayanami::Config("")},
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
  cast->configure(ayanami::PluginContext{"test_smoother", ayanami::Config("")},
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
  cast->configure(ayanami::PluginContext{"test_controller", ayanami::Config("")},
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
  cast->configure(ayanami::PluginContext{"test_recovery", ayanami::Config("")},
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
  EXPECT_EQ(ctx.getMapFrame(), "map");
  EXPECT_EQ(ctx.getRobotFrame(), "base_link");
  EXPECT_TRUE(ctx.isCurrent());

  // Isolated frame names avoid interference with the map/base_link TF injected by other cases.
  ctx.setFrames("isolated_map", "isolated_base");
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

// ---------- lockPlanning: planner family read contract (global+local overlay) ----------

TEST(CostmapPlanningViewTest, overlayMergesLocalWindowWithMax) {
  CostmapMapContext ctx;
  ctx.setFrames("map_a", "base_a");

  // global: 10x10 @0.5m origin (0,0); lethal at cell (1,1).
  auto global_lm = std::make_shared<costmap_2d::LayeredCostmap>(
      "map_a", false, false);
  global_lm->resizeMap(10, 10, 0.5, 0.0, 0.0);
  global_lm->getCostmap()->setCost(1, 1, costmap_2d::LETHAL_OBSTACLE);
  auto global =
      std::make_shared<costmap_2d::CostmapWrapper>(global_lm, "map_a");

  // local: 10x10 @0.5m origin (0,0), same map frame; inflated-cost at cell (5,5).
  auto local_lm = std::make_shared<costmap_2d::LayeredCostmap>(
      "map_a", true, false);
  local_lm->resizeMap(10, 10, 0.5, 0.0, 0.0);
  local_lm->getCostmap()->setCost(5, 5, 200);
  auto local =
      std::make_shared<costmap_2d::CostmapWrapper>(local_lm, "map_a");

  ctx.setGlobalCostmap(global);
  ctx.setLocalCostmap(local);

  auto view = ctx.lockPlanning();
  ASSERT_TRUE(view.valid());
  EXPECT_NE(view.globalGrid(), nullptr);

  // Window overlap: max(global=0, local=200) = 200.
  EXPECT_EQ(view.costAtGlobalCell(5, 5), 200);
  // Compose buffer agrees with the per-cell overlay.
  std::vector<unsigned char> buffer;
  view.composeOverlay(buffer);
  ASSERT_EQ(buffer.size(),
            static_cast<std::size_t>(10) * 10);
  EXPECT_EQ(buffer[static_cast<std::size_t>(5) * 10 + 5], 200);
  EXPECT_FALSE(view.blockedAtGlobalCell(5, 5));
  // Global lethal outside local distinct content stays.
  EXPECT_EQ(view.costAtGlobalCell(1, 1), costmap_2d::LETHAL_OBSTACLE);
  EXPECT_TRUE(view.blockedAtGlobalCell(1, 1));
  // Global unknown dominates local free (conservative).
  global_lm->getCostmap()->setCost(2, 2, costmap_2d::NO_INFORMATION);
  EXPECT_EQ(view.costAtGlobalCell(2, 2), costmap_2d::NO_INFORMATION);
}

TEST(CostmapPlanningViewTest, outsideLocalWindowFallsBackToGlobal) {
  CostmapMapContext ctx;
  ctx.setFrames("map_b", "base_b");
  auto global_lm = std::make_shared<costmap_2d::LayeredCostmap>(
      "map_b", false, false);
  global_lm->resizeMap(10, 10, 0.5, 0.0, 0.0);
  global_lm->getCostmap()->setCost(3, 3, 120);
  auto global =
      std::make_shared<costmap_2d::CostmapWrapper>(global_lm, "map_b");
  auto local_lm = std::make_shared<costmap_2d::LayeredCostmap>(
      "map_b", true, false);
  local_lm->resizeMap(10, 10, 0.5, 0.0, 0.0);
  local_lm->getCostmap()->setCost(3, 3, 250);
  auto local =
      std::make_shared<costmap_2d::CostmapWrapper>(local_lm, "map_b");
  ctx.setGlobalCostmap(global);
  ctx.setLocalCostmap(local);

  // Roll the local window far away (map frame): robot at (100, 0) moves the
  // window origin to ~(97.5, -2.5), so global cell (3,3) (world (1.75,1.75))
  // is far outside the local window.
  local->updateMap(100.0, 0.0, 0.0);

  auto view = ctx.lockPlanning();
  ASSERT_TRUE(view.valid());
  // Cell (3,3) is far outside the local window: global-only cost.
  EXPECT_EQ(view.costAtGlobalCell(3, 3), 120);
}

TEST(CostmapPlanningViewTest, missingLocalWrapperDegeneratesToGlobalOnly) {
  CostmapMapContext ctx;
  ctx.setFrames("iso_map", "base_link");
  auto global_lm = std::make_shared<costmap_2d::LayeredCostmap>(
      "iso_map", false, false);
  global_lm->resizeMap(10, 10, 0.5, 0.0, 0.0);
  global_lm->getCostmap()->setCost(4, 4, 250);
  auto global =
      std::make_shared<costmap_2d::CostmapWrapper>(global_lm, "iso_map");
  ctx.setGlobalCostmap(global);
  // No local wrapper: the overlay view stays valid (global-only).
  auto view = ctx.lockPlanning();
  ASSERT_TRUE(view.valid());
  EXPECT_EQ(view.costAtGlobalCell(4, 4), 250);
}

TEST(CostmapMapContextTest, isPathValidDegeneratesWithoutMap) {
  CostmapMapContext ctx;  // No global wrapper
  EXPECT_FALSE(ctx.isPathValid(makePathAt({})));
  EXPECT_TRUE(ctx.isPathValid(makePathAt({{1.0, 1.0}})));
}

