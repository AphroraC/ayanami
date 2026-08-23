#include <gtest/gtest.h>

#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include <Eigen/Geometry>

#include <costmap_2d/costmap_2d.hpp>
#include <costmap_2d/layered_costmap.hpp>
#include <costmap_server/costmap_wrapper.hpp>

#include <ayanami/abstract/core/map_context.hpp>
#include <ayanami/abstract/core/abstract_goal_checker.hpp>
#include <ayanami/costmap/components/costmap_context.hpp>
#include <ayanami/costmap/core/costmap_planner.hpp>
#include <ayanami/costmap/core/costmap_smoother.hpp>
#include <ayanami/costmap/core/costmap_controller.hpp>
#include <ayanami/costmap/core/costmap_recovery.hpp>
#include <ayanami/interfaces/transform_store.hpp>
#include <ayanami/messages/failure.hpp>

namespace {

using ayanami::FailureCode;
using ayanami::costmap::CostmapContext;

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

// ---- 4 个 Costmap* 派生: 实现 initialize + 纯虚, 补 UB-safe virtual 析构 ----

class TestCostmapPlanner : public ayanami::costmap::CostmapPlanner {
 public:
  std::string init_name_;
  CostmapContext::Ptr init_ctx_;
  uint32_t plan_calls_{0};

  virtual ~TestCostmapPlanner() override = default;

  void initialize(const std::string& name,
                  CostmapContext::Ptr ctx) override {
    init_name_ = name;
    init_ctx_ = std::move(ctx);
  }

  FailureCode makePlan(const ayanami::geometry_msgs::PoseStamped&,
                       const ayanami::geometry_msgs::PoseStamped&, double,
                       ayanami::nav_msgs::Path&, double&,
                       std::string&) override {
    ++plan_calls_;
    return FailureCode::None;
  }
};

class TestCostmapSmoother : public ayanami::costmap::CostmapSmoother {
 public:
  std::string init_name_;
  CostmapContext::Ptr init_ctx_;

  virtual ~TestCostmapSmoother() = default;

  void initialize(const std::string& name,
                  CostmapContext::Ptr ctx) override {
    init_name_ = name;
    init_ctx_ = std::move(ctx);
  }

  FailureCode smoothPath(const ayanami::nav_msgs::Path&,
                         ayanami::nav_msgs::Path&, double, bool,
                         std::string&) override {
    return FailureCode::None;
  }
};

class TestCostmapController : public ayanami::costmap::CostmapController {
 public:
  std::string init_name_;
  CostmapContext::Ptr init_ctx_;

  virtual ~TestCostmapController() = default;

  void initialize(const std::string& name,
                  CostmapContext::Ptr ctx) override {
    init_name_ = name;
    init_ctx_ = std::move(ctx);
  }

  FailureCode computeVelocityCommands(
      const ayanami::geometry_msgs::PoseStamped&,
      const ayanami::geometry_msgs::TwistStamped&,
      ayanami::geometry_msgs::TwistStamped&, std::string&,
      const ayanami::abstract::AbstractGoalChecker::Ptr&) override {
    return FailureCode::None;
  }
  bool setPlan(const ayanami::nav_msgs::Path&) override { return true; }
};

class TestCostmapRecovery : public ayanami::costmap::CostmapRecovery {
 public:
  std::string init_name_;
  CostmapContext::Ptr init_ctx_;

  virtual ~TestCostmapRecovery() = default;

  void initialize(const std::string& name,
                  CostmapContext::Ptr ctx) override {
    init_name_ = name;
    init_ctx_ = std::move(ctx);
  }

  FailureCode runRecovery(std::string&) override {
    return FailureCode::None;
  }
};

}  // namespace

// ---------- capability 查询 ----------

// 最小非 costmap 派生: 验证 AbstractMapContext 默认 capability 全不支持。
class StubMapContext : public ayanami::abstract::AbstractMapContext {
 public:
  const std::string& getGlobalFrame() const override {
    static const std::string frame = "map";
    return frame;
  }
  const std::string& getRobotFrame() const override {
    static const std::string frame = "base_link";
    return frame;
  }
  bool getRobotPose(ayanami::geometry_msgs::PoseStamped&) const override {
    return false;
  }
};

TEST(MapCapabilityTest, AbstractContextRejectsCostmapByDefault) {
  StubMapContext ctx;
  EXPECT_FALSE(ctx.hasCapability(ayanami::abstract::MapCapability::Costmap));
}

TEST(MapCapabilityTest, CostmapContextDeclaresCostmapCapability) {
  CostmapContext ctx;
  EXPECT_TRUE(ctx.hasCapability(ayanami::abstract::MapCapability::Costmap));
}

// ---------- 帧名 getters ----------

TEST(CostmapContextTest, FrameGettersReturnConfiguredFrames) {
  CostmapContext ctx;
  EXPECT_EQ(ctx.getGlobalFrame(), "map");
  EXPECT_EQ(ctx.getRobotFrame(), "base_link");

  ctx.global_frame_ = "odom";
  ctx.robot_frame_ = "base_footprint";
  EXPECT_EQ(ctx.getGlobalFrame(), "odom");
  EXPECT_EQ(ctx.getRobotFrame(), "base_footprint");
}

// ---------- isCurrent ----------

TEST(CostmapContextTest, IsCurrentTrueWithoutWrappers) {
  CostmapContext ctx;
  EXPECT_TRUE(ctx.isCurrent());
}

TEST(CostmapContextTest, IsCurrentForwardsToWrappers) {
  auto global = makeWrapper(makeGrid());
  auto local = makeWrapper(makeGrid());

  CostmapContext ctx;
  ctx.global_costmap_ = global;
  ctx.local_costmap_ = local;

  EXPECT_TRUE(ctx.isCurrent());
  global->setCurrent(false);
  EXPECT_FALSE(ctx.isCurrent());
  global->setCurrent(true);
  local->setCurrent(false);
  EXPECT_FALSE(ctx.isCurrent());
}

TEST(CostmapContextTest, IsCurrentWithNullWrapperIgnoresIt) {
  auto local = makeWrapper(makeGrid());
  local->setCurrent(false);

  CostmapContext ctx;
  ctx.local_costmap_ = local;
  EXPECT_FALSE(ctx.isCurrent());

  ctx.local_costmap_ = nullptr;
  EXPECT_TRUE(ctx.isCurrent());
}

// ---------- lock / unlock ----------

TEST(CostmapContextTest, LockUnlockForwardToWrapperMutex) {
  auto global = makeWrapper(makeGrid());
  CostmapContext ctx;
  ctx.global_costmap_ = global;

  ctx.lock();
  // 另一线程拿不到锁: ctx::lock 已真正锁定 wrapper 的 recursive_mutex
  std::atomic<bool> locked{true};
  std::thread t([&] { locked = global->mutex().try_lock(); });
  t.join();
  EXPECT_FALSE(locked.load());

  ctx.lock();  // 同线程二次 lock: 可重入
  ctx.unlock();
  ctx.unlock();

  // unlock 后另一线程可拿到
  std::thread t2([&] { locked = global->mutex().try_lock(); });
  t2.join();
  EXPECT_TRUE(locked.load());
  global->mutex().unlock();
}

TEST(CostmapContextTest, LockUnlockWithoutWrapperAreNoops) {
  CostmapContext ctx;
  ctx.lock();
  ctx.unlock();
}

// ---------- getRobotPose (TransformStore) ----------

TEST(CostmapContextTest, GetRobotPoseFailsWithoutTransformStore) {
  CostmapContext ctx;
  ayanami::geometry_msgs::PoseStamped pose;
  EXPECT_FALSE(ctx.getRobotPose(pose));
}

TEST(CostmapContextTest, GetRobotPoseUsesTransformStore) {
  auto& store = ayanami::interfaces::TransformStore::instance();
  store.setTransform(
      "map", "base_link", ayanami::Timestamp::latest(),
      Eigen::Translation3d(1.0, 2.0, 0.0) * Eigen::Quaterniond::Identity());

  CostmapContext ctx;
  ctx.transform_store_ = &store;

  ayanami::geometry_msgs::PoseStamped pose;
  EXPECT_TRUE(ctx.getRobotPose(pose));
  EXPECT_EQ(pose.header.frame_id, "map");
  EXPECT_DOUBLE_EQ(1.0, pose.pose.translation().x());
  EXPECT_DOUBLE_EQ(2.0, pose.pose.translation().y());
}

// ---------- 便利 accessor ----------

TEST(CostmapContextTest, Costmap2DAccessorsReturnMasterGrid) {
  auto lm = makeGrid();
  auto global = makeWrapper(lm);
  auto local = makeWrapper(makeGrid());

  CostmapContext ctx;
  EXPECT_EQ(ctx.globalCostmap2D(), nullptr);
  EXPECT_EQ(ctx.localCostmap2D(), nullptr);

  ctx.global_costmap_ = global;
  ctx.local_costmap_ = local;
  EXPECT_EQ(ctx.globalCostmap2D(), lm->getCostmap());
  EXPECT_EQ(ctx.localCostmap2D(), local->get().getCostmap());
}

// ---------- 4 个 costmap/core Costmap* 基类 ----------

TEST(CostmapCoreTest, CostmapPlannerInitializeInjectsContext) {
  auto planner = std::make_shared<TestCostmapPlanner>();
  auto ctx = std::make_shared<CostmapContext>();

  ayanami::abstract::AbstractPlanner::Ptr base = planner;
  auto cast =
      std::dynamic_pointer_cast<ayanami::costmap::CostmapPlanner>(base);
  ASSERT_NE(cast, nullptr);
  cast->initialize("test_planner", ctx);

  EXPECT_EQ(planner->init_name_, "test_planner");
  EXPECT_EQ(planner->init_ctx_, ctx);

  ayanami::geometry_msgs::PoseStamped start;
  ayanami::geometry_msgs::PoseStamped goal;
  ayanami::nav_msgs::Path plan;
  double cost = 0.0;
  std::string message;
  EXPECT_EQ(base->makePlan(start, goal, 0.1, plan, cost, message),
            FailureCode::None);
  EXPECT_EQ(planner->plan_calls_, 1u);
}

TEST(CostmapCoreTest, CostmapSmootherInitializeInjectsContext) {
  auto smoother = std::make_shared<TestCostmapSmoother>();
  auto ctx = std::make_shared<CostmapContext>();

  ayanami::abstract::AbstractSmoother::Ptr base = smoother;
  auto cast =
      std::dynamic_pointer_cast<ayanami::costmap::CostmapSmoother>(base);
  ASSERT_NE(cast, nullptr);
  cast->initialize("test_smoother", ctx);

  EXPECT_EQ(smoother->init_name_, "test_smoother");
  EXPECT_EQ(smoother->init_ctx_, ctx);
}

TEST(CostmapCoreTest, CostmapControllerInitializeInjectsContext) {
  auto controller = std::make_shared<TestCostmapController>();
  auto ctx = std::make_shared<CostmapContext>();

  ayanami::abstract::AbstractController::Ptr base = controller;
  auto cast =
      std::dynamic_pointer_cast<ayanami::costmap::CostmapController>(base);
  ASSERT_NE(cast, nullptr);
  cast->initialize("test_controller", ctx);

  EXPECT_EQ(controller->init_name_, "test_controller");
  EXPECT_EQ(controller->init_ctx_, ctx);
}

TEST(CostmapCoreTest, CostmapRecoveryInitializeInjectsContext) {
  auto recovery = std::make_shared<TestCostmapRecovery>();
  auto ctx = std::make_shared<CostmapContext>();

  ayanami::abstract::AbstractRecovery::Ptr base = recovery;
  auto cast =
      std::dynamic_pointer_cast<ayanami::costmap::CostmapRecovery>(base);
  ASSERT_NE(cast, nullptr);
  cast->initialize("test_recovery", ctx);

  EXPECT_EQ(recovery->init_name_, "test_recovery");
  EXPECT_EQ(recovery->init_ctx_, ctx);
}
