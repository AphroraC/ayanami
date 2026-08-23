#include <gtest/gtest.h>

#include <ayanami/abstract/core/abstract_planner.hpp>
#include <ayanami/abstract/core/abstract_controller.hpp>
#include <ayanami/abstract/core/abstract_goal_checker.hpp>
#include <ayanami/abstract/core/abstract_smoother.hpp>
#include <ayanami/abstract/core/abstract_recovery.hpp>
#include <ayanami/abstract/core/abstract_navigation.hpp>
#include <ayanami/abstract/core/map_context.hpp>
#include <ayanami/messages/failure.hpp>

#include <Eigen/Geometry>

#include <memory>
#include <string>
#include <type_traits>

namespace {

using ayanami::FailureCode;
using ayanami::geometry_msgs::PoseStamped;
using ayanami::geometry_msgs::TwistStamped;
using ayanami::nav_msgs::OccupancyGrid;
using ayanami::nav_msgs::Path;

// ---- 测试派生类: 只实现纯虚, 继承默认实现 (cancel/setMapCallback/lock/unlock/isCurrent) ----

class TestPlanner : public ayanami::abstract::AbstractPlanner {
public:
  FailureCode makePlan(const PoseStamped&, const PoseStamped&, double,
                       Path&, double&, std::string&) override {
    return FailureCode::None;
  }
};

class TestController : public ayanami::abstract::AbstractController {
public:
  FailureCode computeVelocityCommands(const PoseStamped&, const TwistStamped&,
                                      TwistStamped&, std::string&,
                                      const ayanami::abstract::AbstractGoalChecker::Ptr&) override {
    return FailureCode::None;
  }
  bool setPlan(const Path&) override { return true; }
};

class TestSmoother : public ayanami::abstract::AbstractSmoother {
public:
  FailureCode smoothPath(const Path&, Path&, double, bool,
                         std::string&) override {
    return FailureCode::None;
  }
};

class TestRecovery : public ayanami::abstract::AbstractRecovery {
public:
  FailureCode runRecovery(std::string&) override { return FailureCode::None; }
};

class TestNavigation : public ayanami::abstract::AbstractNavigation {
public:
  void initialize(const std::string&) override {}
  void start() override {}
  void stop() override {}
  void spinOnce() override {}
  void setGoal(const PoseStamped&) override {}
};

class TestMapContext : public ayanami::abstract::AbstractMapContext {
public:
  const std::string& getGlobalFrame() const override { return global_; }
  const std::string& getRobotFrame() const override { return robot_; }
  bool getRobotPose(PoseStamped& pose) const override {
    pose.header.frame_id = global_;
    pose.pose = Eigen::Isometry3d::Identity();
    return true;
  }

private:
  std::string global_ = "map";
  std::string robot_ = "base_link";
};

} // namespace

TEST(CoreAbstractPlanner, MakePlanCallableAndCancelDefaultsFalse) {
  TestPlanner p;
  PoseStamped start, goal;
  Path plan;
  double cost = 0.0;
  std::string msg;
  EXPECT_EQ(p.makePlan(start, goal, 0.1, plan, cost, msg), FailureCode::None);
  EXPECT_FALSE(p.cancel());
}

TEST(CoreAbstractController, ComputeGoalAndSetPlan) {
  TestController c;
  PoseStamped pose;
  TwistStamped vel, cmd;
  std::string msg;
  ayanami::abstract::AbstractGoalChecker::Ptr checker;
  EXPECT_EQ(c.computeVelocityCommands(pose, vel, cmd, msg, checker),
            FailureCode::None);
  Path plan;
  EXPECT_TRUE(c.setPlan(plan));
  EXPECT_FALSE(c.cancel());
}

TEST(CoreAbstractSmoother, SmoothPathCallableAndCancelDefaultsFalse) {
  TestSmoother s;
  Path path, out;
  std::string msg;
  EXPECT_EQ(s.smoothPath(path, out, 2.0, true, msg), FailureCode::None);
  EXPECT_FALSE(s.cancel());
}

TEST(CoreAbstractRecovery, RunRecoveryCallableAndCancelDefaultsFalse) {
  TestRecovery r;
  std::string msg;
  EXPECT_EQ(r.runRecovery(msg), FailureCode::None);
  EXPECT_FALSE(r.cancel());
}

TEST(CoreAbstractNavigation, LifecycleAndSetGoal) {
  TestNavigation nav;
  nav.initialize("/tmp/nonexistent.json");
  nav.start();
  nav.spinOnce();
  nav.stop();
  PoseStamped goal;
  nav.setGoal(goal);
}

TEST(CoreAbstractNavigation, SetMapCallbackDefaultsToNoop) {
  TestNavigation nav;
  bool called = false;
  ayanami::abstract::AbstractNavigation::MapCallback cb =
      [&](const std::string&, const OccupancyGrid&) { called = true; };
  nav.setMapCallback(cb);
  EXPECT_FALSE(called);
}

TEST(CoreAbstractMapContext, DefaultCurrentAndLockUnlockNoop) {
  TestMapContext mc;
  EXPECT_TRUE(mc.isCurrent());
  mc.lock();
  mc.unlock();
}

TEST(CoreAbstractMapContext, FramesAndRobotPoseFallback) {
  TestMapContext mc;
  EXPECT_EQ(mc.getGlobalFrame(), "map");
  EXPECT_EQ(mc.getRobotFrame(), "base_link");
  PoseStamped pose;
  ASSERT_TRUE(mc.getRobotPose(pose));
  EXPECT_EQ(pose.header.frame_id, "map");
  EXPECT_TRUE(pose.pose.matrix().isIdentity());
}

TEST(CoreAbstract, PtrAliasesExist) {
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
      std::is_same_v<ayanami::abstract::AbstractMapContext::Ptr,
                     std::shared_ptr<ayanami::abstract::AbstractMapContext>>);
}
