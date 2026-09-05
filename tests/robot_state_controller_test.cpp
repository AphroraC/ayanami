// RobotStateController 单测 (spec §5.3 扩展):
// - getRobotState = 最新 TF 位姿 (TransformStore latest) + 最新速度
// - 速度超过 velocity_timeout (或无速度) → 零速度但位姿仍有效
// - latest-wins: setVelocity 覆写
// - 帧名/超时只在 initialize() 缓存, hot getRobotState 不读 Config
// - 并发读写线程安全

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
  // 未过期: 速度返回真实值
  ASSERT_TRUE(RobotStateController::instance().getRobotState(pose, velocity));
  EXPECT_DOUBLE_EQ(velocity.twist.linear.x(), 2.0);

  std::this_thread::sleep_for(std::chrono::milliseconds(30));

  // 超时后: 零速度, 但位姿仍有效
  ASSERT_TRUE(RobotStateController::instance().getRobotState(pose, velocity));
  EXPECT_TRUE(isZero(velocity));
  EXPECT_TRUE(pose.pose.position.isZero(1e-9));
  EXPECT_FALSE(RobotStateController::instance().hasFreshVelocity());
}

TEST(RobotStateControllerTest, noPoseReturnsFalse) {
  // 用独立帧名 (不与其他测试共享 TransformStore), 且不设任何 TF:
  // getRobotState 必须返回 false (无可用位姿)。
  RobotStateController::instance().initialize(
      "no_pose_map", "no_pose_odom", "no_pose_base",
      /*velocity_timeout=*/0.25);
  RobotStateController::instance().setVelocity(makeTwist(1.0));

  PoseStamped pose;
  TwistStamped velocity;
  EXPECT_FALSE(RobotStateController::instance().getRobotState(pose, velocity));

  // 恢复主帧配置 (保持进程内后续测试独立)。
  initController();
}

TEST(RobotStateControllerTest, hotPathNeverReadsConfig) {
  setUpTransforms();
  initController();
  RobotStateController::instance().setVelocity(makeTwist(0.7));

  // 把 GlobalConfig 指向不存在的目录 (空配置): 已缓存的帧名/超时不受影响,
  // getRobotState 不读 Config。
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
