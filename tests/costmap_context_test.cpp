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

/// @brief Feed one robot pose edge into the RobotStateController singleton
/// (the context queries it directly -- the framework-wide TF surface).
void feedRobotPose(const std::string& frame, const std::string& robot_frame,
                   const Eigen::Vector3d& translation) {
  ayanami::components::RobotStateController::instance().setTransform(
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
  EXPECT_EQ(ctx.getGlobalFrame(), "map");
  EXPECT_EQ(ctx.getLocalFrame(), "odom");
  EXPECT_EQ(ctx.getRobotFrame(), "base_link");

  ctx.setFrames("map_a", "odom_a", "base_footprint");
  EXPECT_EQ(ctx.getGlobalFrame(), "map_a");
  EXPECT_EQ(ctx.getLocalFrame(), "odom_a");
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

  const CostmapView view = ctx.lockGlobal();
  ASSERT_TRUE(view.has_robot_pose());
  EXPECT_DOUBLE_EQ(view.robot_pose().pose.position.x(), 3.0);
  EXPECT_DOUBLE_EQ(view.robot_pose().pose.position.y(), 4.0);
}

TEST(CostmapMapContextTest, viewRobotPoseStampedInOwnFrame) {
  // Distinct frame names avoid interference with other cases. Each map's
  // view resolves the robot pose in the wrapper's OWN frame (global = mapF,
  // local = locF). TF chain: locF <- mapF <- baseF (fast_tf mounts new
  // parents through existing intermediate frames only).
  auto& rsc = ayanami::components::RobotStateController::instance();
  rsc.setTransform("mapF", "baseF", ayanami::Timestamp::latest(),
                   Eigen::Translation3d(3.0, 4.0, 0.0) *
                       Eigen::Quaterniond::Identity());
  rsc.setTransform("locF", "mapF", ayanami::Timestamp::latest(),
                   Eigen::Translation3d(2.0, 0.0, 0.0) *
                       Eigen::Quaterniond::Identity());

  CostmapMapContext ctx;
  ctx.setFrames("mapF", "locF", "baseF");
  ctx.setLocalCostmap(makeWrapperFrame("locF"));
  ctx.setGlobalCostmap(makeWrapperFrame("mapF"));

  const CostmapView lview = ctx.lockLocal();
  ASSERT_TRUE(lview.valid());
  EXPECT_EQ(lview.frame(), "locF");
  ASSERT_TRUE(lview.has_robot_pose());
  EXPECT_EQ(lview.robot_pose().header.frame_id, "locF");
  EXPECT_DOUBLE_EQ(lview.robot_pose().pose.position.x(), 5.0);
  EXPECT_DOUBLE_EQ(lview.robot_pose().pose.position.y(), 4.0);
  const CostmapView gview = ctx.lockGlobal();
  ASSERT_TRUE(gview.valid());
  EXPECT_EQ(gview.frame(), "mapF");
  ASSERT_TRUE(gview.has_robot_pose());
  EXPECT_EQ(gview.robot_pose().header.frame_id, "mapF");
  EXPECT_DOUBLE_EQ(gview.robot_pose().pose.position.x(), 3.0);
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
  EXPECT_EQ(ctx.getGlobalFrame(), "map");
  EXPECT_EQ(ctx.getLocalFrame(), "odom");
  EXPECT_EQ(ctx.getRobotFrame(), "base_link");
  EXPECT_TRUE(ctx.isCurrent());

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

// ---------- composePlanningOverlay: planner family read contract (global+local overlay) ----------

TEST(CostmapPlanningOverlayTest, overlayMergesLocalWindowWithMax) {
  CostmapMapContext ctx;
  // Same-frame local (identity transform path): overlay in the global frame.
  ctx.setFrames("map_a", "map_a", "base_a");

  // global: 10x10 @0.5m origin (0,0); lethal at cell (1,1).
  auto global_lm = std::make_shared<costmap_2d::LayeredCostmap>(
      "map_a", false, false);
  global_lm->resizeMap(10, 10, 0.5, 0.0, 0.0);
  global_lm->getCostmap()->setCost(1, 1, costmap_2d::LETHAL_OBSTACLE);
  auto global =
      std::make_shared<costmap_2d::CostmapWrapper>(global_lm, "map_a");

  // local: 10x10 @0.5m origin (0,0), same frame; inflated-cost at cell (5,5).
  auto local_lm = std::make_shared<costmap_2d::LayeredCostmap>(
      "map_a", true, false);
  local_lm->resizeMap(10, 10, 0.5, 0.0, 0.0);
  local_lm->getCostmap()->setCost(5, 5, 200);
  auto local =
      std::make_shared<costmap_2d::CostmapWrapper>(local_lm, "map_a");

  ctx.setGlobalCostmap(global);
  ctx.setLocalCostmap(local);

  ayanami::costmap::PlanningOverlay overlay;
  ASSERT_TRUE(ctx.composePlanningOverlay(overlay));
  ASSERT_TRUE(overlay.valid);
  EXPECT_EQ(overlay.frame, "map_a");
  EXPECT_EQ(overlay.size_x, 10u);
  EXPECT_TRUE(overlay.local_overlay_valid);

  // Window overlap: max(global=0, local=200) = 200.
  EXPECT_EQ(overlay.costAt(5, 5), 200);
  EXPECT_FALSE(overlay.blockedCell(5, 5));
  // Global lethal outside local distinct content stays.
  EXPECT_EQ(overlay.costAt(1, 1), costmap_2d::LETHAL_OBSTACLE);
  EXPECT_TRUE(overlay.blockedCell(1, 1));
  // Global unknown dominates local free (conservative).
  global_lm->getCostmap()->setCost(2, 2, costmap_2d::NO_INFORMATION);
  ayanami::costmap::PlanningOverlay overlay2;
  ASSERT_TRUE(ctx.composePlanningOverlay(overlay2));
  EXPECT_EQ(overlay2.costAt(2, 2), costmap_2d::NO_INFORMATION);
}

TEST(CostmapPlanningOverlayTest, overlayTransformsLocalFrameOncePerCompose) {
  CostmapMapContext ctx;
  // local frame = odom_a, shifted by (+10, 0) relative to global map_b
  // (p_local = p_global + (10, 0)): local marks live in the odometry frame.
  ctx.setFrames("map_b", "odom_a", "base_b");
  ayanami::components::RobotStateController::instance().setTransform(
      "odom_a", "map_b", ayanami::Timestamp::latest(),
      Eigen::Translation3d(10.0, 0.0, 0.0) * Eigen::Quaterniond::Identity());

  // global: 10x10 @0.5m origin (0,0); inflated-cost at cell (2,2)
  // (world (1.25,1.25) -> local world (11.25,1.25) -> local cell (22,2)).
  auto global_lm = std::make_shared<costmap_2d::LayeredCostmap>(
      "map_b", false, false);
  global_lm->resizeMap(10, 10, 0.5, 0.0, 0.0);
  auto global =
      std::make_shared<costmap_2d::CostmapWrapper>(global_lm, "map_b");

  // local: 30x30 @0.5m origin (0,0) covering local world 0..15m, so the
  // shifted marks land inside the window.
  auto local_lm = std::make_shared<costmap_2d::LayeredCostmap>(
      "odom_a", true, false);
  local_lm->resizeMap(30, 30, 0.5, 0.0, 0.0);
  local_lm->getCostmap()->setCost(22, 2, 200);
  auto local =
      std::make_shared<costmap_2d::CostmapWrapper>(local_lm, "odom_a");

  ctx.setGlobalCostmap(global);
  ctx.setLocalCostmap(local);

  ayanami::costmap::PlanningOverlay overlay;
  ASSERT_TRUE(ctx.composePlanningOverlay(overlay));
  ASSERT_TRUE(overlay.valid);
  EXPECT_TRUE(overlay.local_overlay_valid);
  // The local mark (local cell (22,2)) shows up at the GLOBAL cell (2,2):
  // one transform resolution per compose, per-cell math only.
  EXPECT_EQ(overlay.costAt(2, 2), 200);
  // Untouched global cells stay free.
  EXPECT_EQ(overlay.costAt(3, 3), 0);
}

TEST(CostmapPlanningOverlayTest, missingLocalTransformDegeneratesToGlobalOnly) {
  CostmapMapContext ctx;
  // No odom_b<-map_b edge is ever fed: the local map cannot participate.
  ctx.setFrames("map_c", "odom_b", "base_c");
  auto global_lm = std::make_shared<costmap_2d::LayeredCostmap>(
      "map_c", false, false);
  global_lm->resizeMap(10, 10, 0.5, 0.0, 0.0);
  global_lm->getCostmap()->setCost(3, 3, 120);
  auto global =
      std::make_shared<costmap_2d::CostmapWrapper>(global_lm, "map_c");
  auto local_lm = std::make_shared<costmap_2d::LayeredCostmap>(
      "odom_b", true, false);
  local_lm->resizeMap(10, 10, 0.5, 0.0, 0.0);
  local_lm->getCostmap()->setCost(3, 3, 250);
  auto local =
      std::make_shared<costmap_2d::CostmapWrapper>(local_lm, "odom_b");
  ctx.setGlobalCostmap(global);
  ctx.setLocalCostmap(local);

  ayanami::costmap::PlanningOverlay overlay;
  ASSERT_TRUE(ctx.composePlanningOverlay(overlay));
  ASSERT_TRUE(overlay.valid);
  EXPECT_FALSE(overlay.local_overlay_valid);
  // Global-only cost (the local 250 did NOT participate).
  EXPECT_EQ(overlay.costAt(3, 3), 120);
}

TEST(CostmapPlanningOverlayTest, outsideLocalWindowFallsBackToGlobal) {
  CostmapMapContext ctx;
  ctx.setFrames("map_d", "map_d", "base_d");
  auto global_lm = std::make_shared<costmap_2d::LayeredCostmap>(
      "map_d", false, false);
  global_lm->resizeMap(10, 10, 0.5, 0.0, 0.0);
  global_lm->getCostmap()->setCost(3, 3, 120);
  auto global =
      std::make_shared<costmap_2d::CostmapWrapper>(global_lm, "map_d");
  auto local_lm = std::make_shared<costmap_2d::LayeredCostmap>(
      "map_d", true, false);
  local_lm->resizeMap(10, 10, 0.5, 0.0, 0.0);
  local_lm->getCostmap()->setCost(3, 3, 250);
  auto local =
      std::make_shared<costmap_2d::CostmapWrapper>(local_lm, "map_d");
  ctx.setGlobalCostmap(global);
  ctx.setLocalCostmap(local);

  // Roll the local window far away (map frame): robot at (100, 0) moves the
  // window origin to ~(97.5, -2.5), so global cell (3,3) (world (1.75,1.75))
  // is far outside the local window.
  local->updateMap(100.0, 0.0, 0.0);

  ayanami::costmap::PlanningOverlay overlay;
  ASSERT_TRUE(ctx.composePlanningOverlay(overlay));
  ASSERT_TRUE(overlay.valid);
  // Cell (3,3) is far outside the local window: global-only cost.
  EXPECT_EQ(overlay.costAt(3, 3), 120);
}

TEST(CostmapPlanningOverlayTest, missingLocalWrapperDegeneratesToGlobalOnly) {
  CostmapMapContext ctx;
  ctx.setFrames("iso_map", "iso_odom", "base_link");
  auto global_lm = std::make_shared<costmap_2d::LayeredCostmap>(
      "iso_map", false, false);
  global_lm->resizeMap(10, 10, 0.5, 0.0, 0.0);
  global_lm->getCostmap()->setCost(4, 4, 250);
  auto global =
      std::make_shared<costmap_2d::CostmapWrapper>(global_lm, "iso_map");
  ctx.setGlobalCostmap(global);
  // No local wrapper: the overlay stays valid (global-only).
  ayanami::costmap::PlanningOverlay overlay;
  ASSERT_TRUE(ctx.composePlanningOverlay(overlay));
  ASSERT_TRUE(overlay.valid);
  EXPECT_EQ(overlay.costAt(4, 4), 250);
}

TEST(CostmapMapContextTest, isPathValidDegeneratesWithoutMap) {
  CostmapMapContext ctx;  // No global wrapper
  EXPECT_FALSE(ctx.isPathValid(makePathAt({})));
  EXPECT_TRUE(ctx.isPathValid(makePathAt({{1.0, 1.0}})));
}

