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
using ayanami::ControllerCommand;
using ayanami::ControllerResultT;
using ayanami::PlannerResultT;
using ayanami::RecoveryCycleData;
using ayanami::RecoveryResultT;
using ayanami::RecoveryStatus;
using ayanami::SmootherResultT;
using ayanami::costmap::CostmapMapContext;
using ayanami::costmap::CostmapView;

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

// ---- 4 个 Costmap* 派生: 实现新 configure(context, map) + 加锁读视图 ----

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

  PlannerResultT makePlan(
      const ayanami::geometry_msgs::PoseStamped&,
      const ayanami::geometry_msgs::PoseStamped&, double) override {
    ++plan_calls;
    got_view = map_context && map_context->lockGlobal().valid();
    return PlannerResultT::success(ayanami::nav_msgs::Path{});
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

  SmootherResultT smoothPath(ayanami::nav_msgs::Path& path, double,
                             bool) override {
    got_view = map_context && map_context->lockGlobal().valid();
    return SmootherResultT::success(std::move(path));
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

  ControllerResultT computeVelocityCommand(
      const ayanami::geometry_msgs::PoseStamped&,
      const ayanami::geometry_msgs::TwistStamped&) override {
    got_view = map_context && map_context->lockLocal().valid();
    return ControllerResultT::success(ControllerCommand{});
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

  RecoveryResultT tick() override {
    got_view = map_context && map_context->lockGlobal().valid();
    RecoveryCycleData data;
    data.status = RecoveryStatus::SUCCESS;
    return RecoveryResultT::success(std::move(data));
  }

 private:
  ayanami::costmap::CostmapMapContext::ConstPtr map_context;
};

}  // namespace

// ---------- 帧名 getters ----------

TEST(CostmapMapContextTest, frameGettersReturnConfiguredFrames) {
  CostmapMapContext ctx;
  EXPECT_EQ(ctx.getGlobalFrame(), "map");
  EXPECT_EQ(ctx.getRobotFrame(), "base_link");

  ctx.setFrames("odom", "odom", "base_footprint");
  EXPECT_EQ(ctx.getGlobalFrame(), "odom");
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

// ---------- 加锁读视图 (共享锁内零拷贝直读) ----------

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

// ---------- getRobotPose (RobotStateController) ----------

TEST(CostmapMapContextTest, getRobotPoseFailsWithoutTransform) {
  CostmapMapContext ctx;
  ayanami::geometry_msgs::PoseStamped pose;
  EXPECT_FALSE(ctx.getRobotPose(pose));
}

TEST(CostmapMapContextTest, getRobotPoseUsesTransform) {
  auto& rsc = ayanami::components::RobotStateController::instance();
  rsc.setTransform(
      "map", "base_link", ayanami::Timestamp::latest(),
      Eigen::Translation3d(1.0, 2.0, 0.0) * Eigen::Quaterniond::Identity());

  CostmapMapContext ctx;

  ayanami::geometry_msgs::PoseStamped pose;
  EXPECT_TRUE(ctx.getRobotPose(pose));
  EXPECT_EQ(pose.header.frame_id, "map");
  EXPECT_DOUBLE_EQ(1.0, pose.pose.position.x());
  EXPECT_DOUBLE_EQ(2.0, pose.pose.position.y());
}

TEST(CostmapMapContextTest, viewResolvesRobotPose) {
  auto& rsc = ayanami::components::RobotStateController::instance();
  // 用独立时间戳, 避免与同 frame 键的 latest 干扰。
  rsc.setTransform(
      "map", "base_link", ayanami::timestampFromSeconds(5.0),
      Eigen::Translation3d(3.0, 4.0, 0.0) * Eigen::Quaterniond::Identity());

  auto global = makeWrapper(makeGrid());
  CostmapMapContext ctx;
  ctx.setGlobalCostmap(global);

  const CostmapView view = ctx.lockGlobal();
  ASSERT_TRUE(view.has_robot_pose());
  EXPECT_DOUBLE_EQ(view.robot_pose().pose.position.x(), 3.0);
  EXPECT_DOUBLE_EQ(view.robot_pose().pose.position.y(), 4.0);
}

TEST(CostmapMapContextTest, localViewRobotPoseStampedInLocalFrame) {
  auto& rsc = ayanami::components::RobotStateController::instance();
  // 独立 frame 名避免与其它用例的 latest 变换互相干扰:
  // mapF<-odomF = (10,-5), odomF<-baseF = (1,2) (fast_tf 单亲树, 查询沿父链)。
  rsc.setTransform(
      "odomF", "baseF", ayanami::Timestamp::latest(),
      Eigen::Translation3d(1.0, 2.0, 0.0) * Eigen::Quaterniond::Identity());
  rsc.setTransform(
      "mapF", "odomF", ayanami::Timestamp::latest(),
      Eigen::Translation3d(10.0, -5.0, 0.0) * Eigen::Quaterniond::Identity());

  CostmapMapContext ctx;
  ctx.setFrames("mapF", "odomF", "baseF");
  ctx.setLocalCostmap(makeWrapper(makeGrid()));
  ctx.setGlobalCostmap(makeWrapper(makeGrid()));

  // local 视图: 位姿必须解析在视图自身 frame (odomF), 不允许盖 global 章。
  const CostmapView lview = ctx.lockLocal();
  ASSERT_TRUE(lview.valid());
  EXPECT_EQ(lview.frame(), "odomF");
  ASSERT_TRUE(lview.has_robot_pose());
  EXPECT_EQ(lview.robot_pose().header.frame_id, "odomF");
  EXPECT_DOUBLE_EQ(1.0, lview.robot_pose().pose.position.x());
  EXPECT_DOUBLE_EQ(2.0, lview.robot_pose().pose.position.y());

  // global 视图: 位姿仍在 global 系 (mapF), 沿父链复合 (10+1, -5+2)。
  const CostmapView gview = ctx.lockGlobal();
  ASSERT_TRUE(gview.valid());
  ASSERT_TRUE(gview.has_robot_pose());
  EXPECT_EQ(gview.robot_pose().header.frame_id, "mapF");
  EXPECT_DOUBLE_EQ(11.0, gview.robot_pose().pose.position.x());
  EXPECT_DOUBLE_EQ(-3.0, gview.robot_pose().pose.position.y());
}

// ---------- 4 个 costmap/core Costmap* 基类 (加锁读视图契约) ----------

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
  auto result = cast->makePlan(start, goal, 0.1);
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
  auto result = cast->smoothPath(path, 10.0, false);
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
  auto result = cast->computeVelocityCommand(pose, velocity);
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

  auto result = cast->tick();
  EXPECT_TRUE(result.ok);
  EXPECT_EQ(result.data.status, RecoveryStatus::SUCCESS);
  EXPECT_TRUE(recovery->got_view);
}

// ---------- CostmapMapContext 无 capability / lock ----------

TEST(CostmapMapContextTest, mapFamilyIgnorantInterfaceOnly) {
  CostmapMapContext ctx;
  // 核心接口只有帧名 / isCurrent / getRobotPose——无 capability 枚举、无锁。
  EXPECT_EQ(ctx.getGlobalFrame(), "map");
  EXPECT_EQ(ctx.getRobotFrame(), "base_link");
  EXPECT_TRUE(ctx.isCurrent());

  // 隔离帧名, 避免与其它用例注入的 map/base_link TF 干扰。
  ctx.setFrames("isolated_map", "isolated_map", "isolated_base");
  ayanami::geometry_msgs::PoseStamped pose;
  EXPECT_FALSE(ctx.getRobotPose(pose));
}
// ---------- isPathValid: 地图族碰撞查询能力 (职责表"路径是否过期") ----------

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
  auto lm = makeGrid();  // 10x10 @ 0.5m, origin (0,0) -> 覆盖 0..5m
  auto* grid = lm->getCostmap();
  // world (1.0,1.0) -> cell (2,2) 置 lethal; cell (3,2) 置 NO_INFORMATION。
  grid->setCost(2, 2, costmap_2d::LETHAL_OBSTACLE);
  grid->setCost(3, 2, costmap_2d::NO_INFORMATION);

  CostmapMapContext ctx;
  ctx.setGlobalCostmap(makeWrapper(lm));

  // 空路径无效。
  EXPECT_FALSE(ctx.isPathValid(makePathAt({})));
  // 自由空间路径有效。
  EXPECT_TRUE(ctx.isPathValid(makePathAt({{0.5, 0.5}, {2.0, 0.5}})));
  // lethal 点无效。
  EXPECT_FALSE(ctx.isPathValid(makePathAt({{0.5, 0.5}, {1.0, 1.0}})));
  // unknown 点无效。
  EXPECT_FALSE(ctx.isPathValid(makePathAt({{1.5, 1.0}})));  // world(1.5,1.0)->cell(3,2)
  // 出界点无效。
  EXPECT_FALSE(ctx.isPathValid(makePathAt({{9.0, 9.0}})));
  // 高 inflation (非 lethal) 代价不判无效 (controller 层裁决)。
  grid->setCost(4, 4, 200);
  EXPECT_TRUE(ctx.isPathValid(makePathAt({{2.0, 2.0}})));
}

TEST(CostmapMapContextTest, isPathValidDegeneratesWithoutMap) {
  CostmapMapContext ctx;  // 无 global wrapper
  EXPECT_FALSE(ctx.isPathValid(makePathAt({})));
  EXPECT_TRUE(ctx.isPathValid(makePathAt({{1.0, 1.0}})));
}

TEST(CostmapMapContextTest, abstractMapContextDefaultIsNonEmptySemantics) {
  ayanami::map::AbstractMapContext::Ptr ctx =
      std::make_shared<CostmapMapContext>();  // 未覆写前的地图族默认
  ayanami::nav_msgs::Path path;
  EXPECT_FALSE(ctx->isPathValid(path));
  path.poses.push_back(ayanami::geometry_msgs::PoseStamped{});
  EXPECT_TRUE(ctx->isPathValid(path));
}
