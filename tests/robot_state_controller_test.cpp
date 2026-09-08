// RobotStateController unit tests (extended):
// - getRobotState = latest TF pose (TransformStore latest) + latest velocity
// - velocity older than velocity_timeout (or no velocity) -> zero velocity but the pose stays valid
// - latest-wins: setVelocity overwrites
// - frame names/timeout are cached only at initialize; the hot getRobotState path does not read Config
// - concurrent reads/writes are thread safe

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <thread>
#include <vector>

#include <Eigen/Geometry>

#include <ayanami/abstract/components/robot_state_controller.hpp>
#include <ayanami/interfaces/transform_store.hpp>
#include <ayanami/messages/geometry_msgs.hpp>
#include <ayanami/utility/config.hpp>

namespace {

using ayanami::components::RobotStateController;
using ayanami::geometry_msgs::PoseStamped;
using ayanami::geometry_msgs::TwistStamped;

auto& store() { return ayanami::interfaces::TransformStore::instance(); }

void setUpTransforms() {
  store().setTransform("map", "base_link", ayanami::Timestamp::latest(),
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

TEST(RobotStateControllerTest, getRobotStateReturnsLatestPoseAndVelocity) {
  setUpTransforms();
  initController();

  RobotStateController::instance().setVelocity(makeTwist(0.5, 0.1));

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
  initController();

  RobotStateController::instance().setVelocity(makeTwist(0.5));
  RobotStateController::instance().setVelocity(makeTwist(1.5, 0.2));

  TwistStamped velocity = RobotStateController::instance().getRobotVelocity();
  EXPECT_DOUBLE_EQ(velocity.twist.linear.x(), 1.5);
  EXPECT_DOUBLE_EQ(velocity.twist.angular.z(), 0.2);
}

TEST(RobotStateControllerTest, staleVelocityYieldsZeroButPoseStaysValid) {
  setUpTransforms();
  initController(/*velocity_timeout=*/0.01);

  RobotStateController::instance().setVelocity(makeTwist(2.0));

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
  // Use independent frame names (not sharing the TransformStore with other tests) and set no TF at all:
  // getRobotState must return false (no pose available).
  RobotStateController::instance().initialize(
      "no_pose_map", "no_pose_odom", "no_pose_base",
      /*velocity_timeout=*/0.25);
  RobotStateController::instance().setVelocity(makeTwist(1.0));

  PoseStamped pose;
  TwistStamped velocity;
  EXPECT_FALSE(RobotStateController::instance().getRobotState(pose, velocity));

  // Restore the main frame config (keeps later tests in this process independent).
  initController();
}

TEST(RobotStateControllerTest, hotPathNeverReadsConfig) {
  setUpTransforms();
  initController();
  RobotStateController::instance().setVelocity(makeTwist(0.7));

  // Point GlobalConfig at a nonexistent directory (empty config): the cached frame names/timeout are unaffected and
  // getRobotState does not read Config.
  ayanami::GlobalConfig::instance("/nonexistent_away_from_tests",
                                  /*override_path=*/true);
  RobotStateController& rsc = RobotStateController::instance();
  EXPECT_EQ(rsc.getGlobalFrame(), "map");
  EXPECT_EQ(rsc.getRobotFrame(), "base_link");
  EXPECT_EQ(rsc.getLocalFrame(), "odom");
  EXPECT_EQ(rsc.getVelocityTimeout(), 0.25);

  PoseStamped pose;
  TwistStamped velocity;
  ASSERT_TRUE(rsc.getRobotState(pose, velocity));
  EXPECT_TRUE(pose.pose.position.isZero(1e-9));
  EXPECT_DOUBLE_EQ(velocity.twist.linear.x(), 0.7);
}

TEST(RobotStateControllerTest, concurrentReadWriteIsThreadSafe) {
  setUpTransforms();
  initController();

  std::atomic<bool> running{true};
  std::vector<std::thread> writers;
  std::vector<std::thread> readers;
  for (int i = 0; i < 4; ++i) {
    writers.emplace_back([&running, i] {
      while (running.load()) {
        RobotStateController::instance().setVelocity(
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
  initController();
  RobotStateController::instance().setVelocity(makeTwist(1.0));
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
