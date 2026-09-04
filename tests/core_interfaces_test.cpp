#include <gtest/gtest.h>

#include <ayanami/abstract/core/abstract_planner.hpp>
#include <ayanami/abstract/core/abstract_controller.hpp>
#include <ayanami/abstract/core/abstract_goal_checker.hpp>
#include <ayanami/abstract/core/abstract_smoother.hpp>
#include <ayanami/abstract/core/abstract_recovery.hpp>
#include <ayanami/abstract/core/abstract_navigation.hpp>
#include <ayanami/abstract/components/robot_state_controller.hpp>
#include <ayanami/costmap/components/costmap_context.hpp>
#include <ayanami/messages/failure.hpp>
#include <ayanami/types.hpp>

#include <Eigen/Geometry>

#include <memory>
#include <string>
#include <type_traits>

namespace {

using ayanami::ErrorCode;
using ayanami::ControllerCommand;
using ayanami::ControllerResultT;
using ayanami::PlannerResultT;
using ayanami::RecoveryCycleData;
using ayanami::RecoveryResultT;
using ayanami::RecoveryStatus;
using ayanami::SmootherResultT;
using ayanami::costmap::CostmapContext;
using ayanami::geometry_msgs::PoseStamped;
using ayanami::geometry_msgs::TwistStamped;
using ayanami::nav_msgs::OccupancyGrid;
using ayanami::nav_msgs::Path;

// ---- 测试派生类: 只实现纯虚, 继承默认实现 (cancel/setMapCallback/isCurrent) ----

class TestPlanner : public ayanami::abstract::AbstractPlanner {
public:
  PlannerResultT makePlan(const PoseStamped&, const PoseStamped&,
                          double) override {
    return PlannerResultT::success(Path{});
  }
};

class TestController : public ayanami::abstract::AbstractController {
public:
  ControllerResultT computeVelocityCommand(const PoseStamped&,
                                           const TwistStamped&) override {
    return ControllerResultT::success(ControllerCommand{});
  }
  bool setPlan(const Path&) override { return true; }
};

class TestSmoother : public ayanami::abstract::AbstractSmoother {
public:
  SmootherResultT smoothPath(Path&, double, bool) override {
    return SmootherResultT::success(Path{});
  }
};

class TestRecovery : public ayanami::abstract::AbstractRecovery {
public:
  RecoveryResultT tick() override {
    RecoveryCycleData data;
    data.status = RecoveryStatus::SUCCESS;
    return RecoveryResultT::success(std::move(data));
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
  auto result = p.makePlan(start, goal, 0.1);
  EXPECT_TRUE(result.ok);
  EXPECT_FALSE(p.cancel());
}

TEST(CoreAbstractController, computeGoalAndSetPlan) {
  TestController c;
  PoseStamped pose;
  TwistStamped vel;
  auto result = c.computeVelocityCommand(pose, vel);
  EXPECT_TRUE(result.ok);
  Path plan;
  EXPECT_TRUE(c.setPlan(plan));
  EXPECT_FALSE(c.cancel());
}

TEST(CoreAbstractSmoother, smoothPathCallableAndCancelDefaultsFalse) {
  TestSmoother s;
  Path path;
  auto result = s.smoothPath(path, 2.0, true);
  EXPECT_TRUE(result.ok);
  EXPECT_FALSE(s.cancel());
}

TEST(CoreAbstractRecovery, runRecoveryCallableAndCancelDefaultsFalse) {
  TestRecovery r;
  auto result = r.tick();
  EXPECT_TRUE(result.ok);
  EXPECT_EQ(result.data.status, RecoveryStatus::SUCCESS);
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

TEST(CoreCostmapContext, defaultCurrentTrue) {
  CostmapContext ctx;
  EXPECT_TRUE(ctx.isCurrent());
}

TEST(CoreCostmapContext, framesAndRobotPose) {
  CostmapContext ctx;
  ctx.setFrames("map", "map", "base_link");
  ayanami::components::RobotStateController::instance().setTransform(
      "map", "base_link", ayanami::Timestamp::latest(),
      Eigen::Isometry3d::Identity());
  EXPECT_EQ(ctx.getGlobalFrame(), "map");
  EXPECT_EQ(ctx.getRobotFrame(), "base_link");
  PoseStamped pose;
  ASSERT_TRUE(ctx.getRobotPose(pose));
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
      std::is_same_v<ayanami::costmap::CostmapContext::Ptr,
                     std::shared_ptr<ayanami::costmap::CostmapContext>>);
}
