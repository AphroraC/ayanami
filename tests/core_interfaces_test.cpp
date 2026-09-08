#include <gtest/gtest.h>

#include <ayanami/abstract/core/abstract_planner.hpp>
#include <ayanami/abstract/core/abstract_controller.hpp>
#include <ayanami/abstract/core/abstract_goal_checker.hpp>
#include <ayanami/abstract/core/abstract_smoother.hpp>
#include <ayanami/abstract/core/abstract_recovery.hpp>
#include <ayanami/abstract/core/abstract_navigation.hpp>
#include <ayanami/abstract/components/robot_state_controller.hpp>
#include <ayanami/costmap/components/costmap_map_context.hpp>
#include <ayanami/messages/failure.hpp>
#include <ayanami/types.hpp>

#include <Eigen/Geometry>

#include <memory>
#include <string>
#include <type_traits>

namespace {

using ayanami::ErrorCode;
using ayanami::ResultT;
using ayanami::RecoveryCycleData;
using ayanami::RecoveryStatus;
using ayanami::costmap::CostmapMapContext;
using ayanami::geometry_msgs::PoseStamped;
using ayanami::geometry_msgs::TwistStamped;
using ayanami::nav_msgs::OccupancyGrid;
using ayanami::nav_msgs::Path;

// ---- Test-derived classes: implement only the pure virtuals, inherit the default implementations (cancel/setMapCallback/isCurrent) ----

class TestPlanner : public ayanami::abstract::AbstractPlanner {
public:
  ResultT<void> makePlan(const PoseStamped&, const PoseStamped&,
                          double, ayanami::nav_msgs::Path& plan_out) override {
    plan_out = Path{};
    return ResultT<void>::success();
  }
};

class TestController : public ayanami::abstract::AbstractController {
public:
  ResultT<void> computeVelocityCommand(const PoseStamped&,
                                           const TwistStamped&, ayanami::geometry_msgs::TwistStamped& cmd) override {
    cmd = TwistStamped{};
    return ResultT<void>::success();
  }
  bool setPlan(const Path&) override { return true; }
};

class TestSmoother : public ayanami::abstract::AbstractSmoother {
public:
  ResultT<void> smoothPath(Path&, double) override {
    return ResultT<void>::success();
  }
};

class TestRecovery : public ayanami::abstract::AbstractRecovery {
public:
  ResultT<void> tick(RecoveryCycleData& cycle) override {
    RecoveryCycleData data;
    data.status = RecoveryStatus::SUCCESS;
    cycle = std::move(data);
    return ResultT<void>::success();
  }
};

class TestNavigation : public ayanami::abstract::AbstractNavigation {
public:
  void initialize(const std::string&) override {}
  void start() override {}
  void stop() override {}
  void spinOnce() override {}
  void setGoal(const PoseStamped&) override {}
};

} // namespace

TEST(CoreAbstractPlanner, makePlanCallableAndCancelDefaultsFalse) {
  TestPlanner p;
  PoseStamped start, goal;
  Path plan;
  auto result = p.makePlan(start, goal, 0.1, plan);
  EXPECT_TRUE(result.ok);
  EXPECT_FALSE(p.cancel());
}

TEST(CoreAbstractController, computeGoalAndSetPlan) {
  TestController c;
  PoseStamped pose;
  TwistStamped vel;
  TwistStamped cmd;
  auto result = c.computeVelocityCommand(pose, vel, cmd);
  EXPECT_TRUE(result.ok);
  Path plan;
  EXPECT_TRUE(c.setPlan(plan));
  EXPECT_FALSE(c.cancel());
}

TEST(CoreAbstractSmoother, smoothPathCallableAndCancelDefaultsFalse) {
  TestSmoother s;
  Path path;
  auto result = s.smoothPath(path, 2.0);
  EXPECT_TRUE(result.ok);
  EXPECT_FALSE(s.cancel());
}

TEST(CoreAbstractRecovery, runRecoveryCallableAndCancelDefaultsFalse) {
  TestRecovery r;
  RecoveryCycleData cycle;
  auto result = r.tick(cycle);
  EXPECT_TRUE(result.ok);
  EXPECT_EQ(cycle.status, RecoveryStatus::SUCCESS);
  EXPECT_FALSE(r.cancel());
}

TEST(CoreAbstractNavigation, lifecycleAndSetGoal) {
  TestNavigation nav;
  nav.initialize("/tmp/nonexistent.json");
  nav.start();
  nav.spinOnce();
  nav.stop();
  PoseStamped goal;
  nav.setGoal(goal);
}

TEST(CoreCostmapMapContext, defaultCurrentTrue) {
  CostmapMapContext ctx;
  EXPECT_TRUE(ctx.isCurrent());
}

TEST(CoreCostmapMapContext, framesAndRobotPose) {
  CostmapMapContext ctx;
  ctx.setFrames("map", "base_link");
  ayanami::components::RobotStateController::instance().setTransform(
      "map", "base_link", ayanami::Timestamp::latest(),
      Eigen::Isometry3d::Identity());
  EXPECT_EQ(ctx.getMapFrame(), "map");
  EXPECT_EQ(ctx.getRobotFrame(), "base_link");
  // Robot state query single facade: RobotStateController (map context no
  // longer exposes pose queries).
  const auto pose =
      ayanami::components::RobotStateController::instance().getRobotPose();
  EXPECT_EQ(pose.header.frame_id, "map");
  EXPECT_TRUE(pose.pose.toIsometry().matrix().isIdentity());
}

TEST(CoreAbstract, ptrAliasesExist) {
  static_assert(
      std::is_same_v<ayanami::abstract::AbstractPlanner::Ptr,
                     std::shared_ptr<ayanami::abstract::AbstractPlanner>>);
  static_assert(
      std::is_same_v<ayanami::abstract::AbstractController::Ptr,
                     std::shared_ptr<ayanami::abstract::AbstractController>>);
  static_assert(
      std::is_same_v<ayanami::abstract::AbstractSmoother::Ptr,
                     std::shared_ptr<ayanami::abstract::AbstractSmoother>>);
  static_assert(
      std::is_same_v<ayanami::abstract::AbstractRecovery::Ptr,
                     std::shared_ptr<ayanami::abstract::AbstractRecovery>>);
  static_assert(
      std::is_same_v<ayanami::abstract::AbstractNavigation::Ptr,
                     std::shared_ptr<ayanami::abstract::AbstractNavigation>>);
  static_assert(
      std::is_same_v<ayanami::costmap::CostmapMapContext::Ptr,
                     std::shared_ptr<ayanami::costmap::CostmapMapContext>>);
}
