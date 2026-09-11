// RobotStateController unit tests (bridge model):
// - TF queries delegate to the injected source (lookupTransform / canTransform /
//   lookupPose); queries before installation fail gracefully
// - getRobotState = latest TF pose (injected source) + latest odometry velocity
// - velocity older than velocity_timeout (or no velocity) -> zero velocity but the pose stays valid
// - latest-wins: setOdometry overwrites
// - frame names/timeout are cached only at initialize; the hot getRobotState path does not read Config
// - concurrent reads/writes are thread safe

#include <gtest/gtest.h>

#include "test_transform_source.hpp"

#include <atomic>
#include <chrono>
#include <thread>
#include <vector>

#include <Eigen/Geometry>

#include <ayanami/abstract/components/robot_state_controller.hpp>
#include <ayanami/messages/geometry_msgs.hpp>
#include <ayanami/utility/config.hpp>

using ayanami::testing::testTransforms;
namespace {

using ayanami::components::RobotStateController;
using ayanami::geometry_msgs::PoseStamped;
using ayanami::geometry_msgs::TwistStamped;

void setUpTransforms() {
  testTransforms().set("map", "base_link",
                       Eigen::Isometry3d::Identity());
}

void initController(double velocity_timeout = 0.25) {
  RobotStateController::instance().initialize(
      "map", "odom", "base_link", velocity_timeout);
}

TwistStamped makeTwist(double linear_x, double angular_z = 0.0) {
  TwistStamped t;
  t.header.frame_id = "base_link";
  t.twist.linear.x() = linear_x;
  t.twist.angular.z() = angular_z;
  return t;
}

bool isZero(const TwistStamped& t) {
  return t.twist.linear.norm() < 1e-12 && t.twist.angular.norm() < 1e-12;
}

}  // namespace

TEST(RobotStateControllerTest, lookupBeforeSourceInstalledFailsGracefully) {
  // A controller instance without an injected source: every query fails
  // (no crash, no stale data).
  RobotStateController::instance().initialize(
      "bridge_none_map", "bridge_none_odom", "bridge_none_base", 0.25);
  // Install + remove an empty source to guarantee the "none" state.
  RobotStateController::instance().setTransformSource({});
  Eigen::Isometry3d transform;
  EXPECT_FALSE(RobotStateController::instance().lookupTransform(
      "a", "b", ayanami::Timestamp::latest(), transform));
  EXPECT_FALSE(RobotStateController::instance().canTransform(
      "a", "b", ayanami::Timestamp::latest()));
  PoseStamped pose;
  EXPECT_FALSE(RobotStateController::instance().lookupPose(
      "a", "b", ayanami::Timestamp::latest(), pose));
  PoseStamped state_pose;
  TwistStamped velocity;
  EXPECT_FALSE(
      RobotStateController::instance().getRobotState(state_pose, velocity));
}

TEST(RobotStateControllerTest, injectedSourceServesQueries) {
  testTransforms().set("map", "base_link", Eigen::Isometry3d::Identity());
  testTransforms().set("base_link", "lidar",
                       Eigen::Translation3d(0.1, 0.0, 0.0) *
                           Eigen::Quaterniond::Identity());
  testTransforms().install();
  initController();

  RobotStateController& rsc = RobotStateController::instance();
  EXPECT_TRUE(rsc.canTransform("map", "lidar", ayanami::Timestamp::latest()));
  Eigen::Isometry3d transform;
  ASSERT_TRUE(rsc.lookupTransform("map", "lidar", ayanami::Timestamp::latest(),
                                  transform));
  EXPECT_DOUBLE_EQ(transform.translation().x(), 0.1);

  PoseStamped pose;
  ASSERT_TRUE(rsc.lookupPose("map", "lidar", ayanami::Timestamp::latest(),
                             pose));
  EXPECT_EQ(pose.header.frame_id, "map");
  EXPECT_DOUBLE_EQ(pose.pose.position.x(), 0.1);

  // Unresolvable pair: false.
  EXPECT_FALSE(rsc.canTransform("map", "unknown_frame",
                                ayanami::Timestamp::latest()));
}

TEST(RobotStateControllerTest, getRobotStateReturnsLatestPoseAndVelocity) {
  setUpTransforms();
  testTransforms().install();
  initController();

  RobotStateController::instance().setOdometry(makeTwist(0.5, 0.1));

  PoseStamped pose;
  TwistStamped velocity;
  ASSERT_TRUE(RobotStateController::instance().getRobotState(pose, velocity));
  EXPECT_EQ(pose.header.frame_id, "map");
  EXPECT_TRUE(pose.pose.position.isZero(1e-9));
  EXPECT_DOUBLE_EQ(velocity.twist.linear.x(), 0.5);
  EXPECT_DOUBLE_EQ(velocity.twist.angular.z(), 0.1);
}

TEST(RobotStateControllerTest, latestWinsVelocity) {
  setUpTransforms();
  testTransforms().install();
  initController();

  RobotStateController::instance().setOdometry(makeTwist(0.5));
  RobotStateController::instance().setOdometry(makeTwist(1.5, 0.2));

  TwistStamped velocity = RobotStateController::instance().getRobotVelocity();
  EXPECT_DOUBLE_EQ(velocity.twist.linear.x(), 1.5);
  EXPECT_DOUBLE_EQ(velocity.twist.angular.z(), 0.2);
}

TEST(RobotStateControllerTest, staleVelocityYieldsZeroButPoseStaysValid) {
  setUpTransforms();
  testTransforms().install();
  initController(/*velocity_timeout=*/0.01);

  RobotStateController::instance().setOdometry(makeTwist(2.0));

  PoseStamped pose;
  TwistStamped velocity;
  // Not expired: the real velocity is returned
  ASSERT_TRUE(RobotStateController::instance().getRobotState(pose, velocity));
  EXPECT_DOUBLE_EQ(velocity.twist.linear.x(), 2.0);

  std::this_thread::sleep_for(std::chrono::milliseconds(30));

  // After timeout: zero velocity, but the pose is still valid
  ASSERT_TRUE(RobotStateController::instance().getRobotState(pose, velocity));
  EXPECT_TRUE(isZero(velocity));
  EXPECT_TRUE(pose.pose.position.isZero(1e-9));
  EXPECT_FALSE(RobotStateController::instance().hasFreshVelocity());
}

TEST(RobotStateControllerTest, noPoseReturnsFalse) {
  // Use independent frame names (not sharing the stub tree with other tests)
  // and set no TF for them: getRobotState must return false (no pose).
  testTransforms().install();
  RobotStateController::instance().initialize(
      "no_pose_map", "no_pose_odom", "no_pose_base",
      /*velocity_timeout=*/0.25);
  RobotStateController::instance().setOdometry(makeTwist(1.0));

  PoseStamped pose;
  TwistStamped velocity;
  EXPECT_FALSE(RobotStateController::instance().getRobotState(pose, velocity));

  // Restore the main frame config (keeps later tests in this process independent).
  initController();
}

TEST(RobotStateControllerTest, hotPathNeverReadsConfig) {
  setUpTransforms();
  testTransforms().install();
  initController();
  RobotStateController::instance().setOdometry(makeTwist(0.7));

  // Point GlobalConfig at a nonexistent directory (empty config): the cached frame names/timeout are unaffected and
  // getRobotState does not read Config.
  ayanami::GlobalConfig::instance("/nonexistent_away_from_tests",
                                  /*override_path=*/true);
  RobotStateController& rsc = RobotStateController::instance();
  EXPECT_EQ(rsc.globalFrame(), "map");
  EXPECT_EQ(rsc.robotFrame(), "base_link");
  EXPECT_EQ(rsc.localFrame(), "odom");
  EXPECT_EQ(rsc.getVelocityTimeout(), 0.25);

  PoseStamped pose;
  TwistStamped velocity;
  ASSERT_TRUE(rsc.getRobotState(pose, velocity));
  EXPECT_TRUE(pose.pose.position.isZero(1e-9));
  EXPECT_DOUBLE_EQ(velocity.twist.linear.x(), 0.7);
}

TEST(RobotStateControllerTest, concurrentReadWriteIsThreadSafe) {
  setUpTransforms();
  testTransforms().install();
  initController();

  std::atomic<bool> running{true};
  std::vector<std::thread> writers;
  std::vector<std::thread> readers;
  for (int i = 0; i < 4; ++i) {
    writers.emplace_back([&running, i] {
      while (running.load()) {
        RobotStateController::instance().setOdometry(
            makeTwist(static_cast<double>(i), 1.0));
      }
    });
    readers.emplace_back([&running] {
      while (running.load()) {
        PoseStamped pose;
        TwistStamped velocity;
        RobotStateController::instance().getRobotState(pose, velocity);
        EXPECT_TRUE(pose.header.frame_id == "map");
      }
    });
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  running.store(false);
  for (auto& t : writers) t.join();
  for (auto& t : readers) t.join();
}

TEST(RobotStateControllerTest, resetVelocityClears) {
  setUpTransforms();
  testTransforms().install();
  initController();
  RobotStateController::instance().setOdometry(makeTwist(1.0));
  EXPECT_TRUE(RobotStateController::instance().hasFreshVelocity());

  RobotStateController::instance().resetVelocity();
  EXPECT_FALSE(RobotStateController::instance().hasFreshVelocity());
  EXPECT_TRUE(isZero(RobotStateController::instance().getRobotVelocity()));
}

TEST(RobotStateControllerTest, selectsClosestOdometryByTimestamp) {
  initController();
  RobotStateController& controller = RobotStateController::instance();

  auto first = makeTwist(1.0);
  first.header.stamp = ayanami::timestampFromSeconds(10.0);
  auto second = makeTwist(2.0);
  second.header.stamp = ayanami::timestampFromSeconds(12.0);
  controller.setOdometry(first);
  controller.setOdometry(second);

  TwistStamped selected;
  ASSERT_TRUE(controller.getOdometry(
      ayanami::timestampFromSeconds(11.8), selected));
  EXPECT_DOUBLE_EQ(selected.twist.linear.x(), 2.0);
}

TEST(RobotStateControllerTest, latestOdometryReturnsNewestFrame) {
  initController();
  RobotStateController& controller = RobotStateController::instance();
  auto first = makeTwist(3.0);
  first.header.stamp = ayanami::timestampFromSeconds(20.0);
  auto second = makeTwist(4.0);
  second.header.stamp = ayanami::timestampFromSeconds(21.0);
  controller.setOdometry(first);
  controller.setOdometry(second);

  TwistStamped selected;
  ASSERT_TRUE(controller.getOdometry(ayanami::Timestamp::latest(), selected));
  EXPECT_DOUBLE_EQ(selected.twist.linear.x(), 4.0);
}
