#include <gtest/gtest.h>

#include <ayanami/messages/geometry_msgs.hpp>
#include <ayanami/messages/nav_msgs.hpp>
#include <ayanami/messages/sensor_msgs.hpp>
#include <ayanami/messages/actions.hpp>
#include <ayanami/messages/failure.hpp>
#include <ayanami/types.hpp>

#include <Eigen/Geometry>

#include <type_traits>
#include <vector>

namespace {

using ayanami::actions::ExePathGoal;
using ayanami::geometry_msgs::PoseStamped;
using ayanami::nav_msgs::Path;

} // namespace

TEST(MessagesTest, poseStampedHasHeaderAndIsometry) {
  PoseStamped p = PoseStamped::fromIsometry(
      "map", Eigen::Translation3d(1.0, 0.0, 0.0) * Eigen::Quaterniond::Identity());
  EXPECT_EQ(p.header.frame_id, "map");
  EXPECT_TRUE(p.header.stamp.isLatest());
  EXPECT_DOUBLE_EQ(p.pose.position.x(), 1.0);
  EXPECT_DOUBLE_EQ(p.pose.position.y(), 0.0);
  EXPECT_EQ(p.toIsometry().matrix(), p.pose.toIsometry().matrix());
}

TEST(MessagesTest, abstractResultFactoryPreservesTypedOutcome) {
  auto result = ayanami::PlannerResultT::success(Path{});
  EXPECT_TRUE(result.ok);
  EXPECT_EQ(result.error, ayanami::ErrorCode::NONE);
  EXPECT_TRUE(static_cast<bool>(result));

  auto failure = ayanami::PlannerResultT::failure(
      ayanami::ErrorCode::PATH_NOT_FOUND, "no route");
  EXPECT_FALSE(failure.ok);
  EXPECT_EQ(failure.error, ayanami::ErrorCode::PATH_NOT_FOUND);
  EXPECT_EQ(failure.message, "no route");
}

TEST(MessagesTest, headerStampDefaultsToLatestSentinel) {
  ayanami::geometry_msgs::Header h;
  EXPECT_TRUE(h.stamp.isLatest());
  EXPECT_EQ(h.stamp.count(), 0);

  ayanami::sensor_msgs::LaserScan scan;
  EXPECT_TRUE(scan.header.stamp.isLatest());

  ayanami::sensor_msgs::PointCloud2 cloud;
  EXPECT_TRUE(cloud.header.stamp.isLatest());
}

TEST(MessagesTest, headerStampCarriesExplicitTimestamp) {
  ayanami::geometry_msgs::PoseStamped p;
  p.header.stamp = ayanami::timestampFromSeconds(12.5);
  EXPECT_FALSE(p.header.stamp.isLatest());
  EXPECT_EQ(p.header.stamp.count(), 12500000000LL);
  EXPECT_NEAR(p.header.stamp.toSeconds(), 12.5, 1e-9);
}

TEST(MessagesTest, pathPosesIsPoseStampedVector) {
  static_assert(std::is_same_v<Path::poses_type::value_type, PoseStamped>);
  Path path;
  path.poses.emplace_back();
  path.poses.back().header.frame_id = "map";
  path.poses.back().pose = ayanami::geometry_msgs::Pose::fromIsometry(Eigen::Isometry3d::Identity());
  EXPECT_EQ(path.poses.size(), 1u);
}

TEST(MessagesTest, failureCodeUnifiedEnumCoversRequiredCodes) {
  // 统一失败码: None=0, 必须覆盖规定的全部分量。
  EXPECT_EQ(static_cast<uint8_t>(ayanami::FailureCode::None), 0u);
  EXPECT_NE(static_cast<uint8_t>(ayanami::FailureCode::Canceled), 0u);
  static_assert(static_cast<int>(ayanami::FailureCode::None) <
                static_cast<int>(ayanami::FailureCode::InternalError));
  using FC = ayanami::FailureCode;
  static_assert(static_cast<int>(FC::None) < static_cast<int>(FC::Canceled));
  static_assert(static_cast<int>(FC::Canceled) < static_cast<int>(FC::Timeout));
  static_assert(static_cast<int>(FC::Timeout) < static_cast<int>(FC::InvalidInput));
  static_assert(static_cast<int>(FC::InvalidInput) <
                static_cast<int>(FC::TransformUnavailable));
  static_assert(static_cast<int>(FC::TransformUnavailable) <
                static_cast<int>(FC::MapUnavailable));
  static_assert(static_cast<int>(FC::MapUnavailable) <
                static_cast<int>(FC::MapStale));
  static_assert(static_cast<int>(FC::MapStale) < static_cast<int>(FC::StartInvalid));
  static_assert(static_cast<int>(FC::StartInvalid) < static_cast<int>(FC::GoalInvalid));
  static_assert(static_cast<int>(FC::GoalInvalid) < static_cast<int>(FC::PathNotFound));
  static_assert(static_cast<int>(FC::PathNotFound) < static_cast<int>(FC::PathInvalid));
  static_assert(static_cast<int>(FC::PathInvalid) <
                static_cast<int>(FC::SmoothingFailed));
  static_assert(static_cast<int>(FC::SmoothingFailed) <
                static_cast<int>(FC::NoValidCommand));
  static_assert(static_cast<int>(FC::NoValidCommand) < static_cast<int>(FC::NoProgress));
  static_assert(static_cast<int>(FC::NoProgress) < static_cast<int>(FC::GoalNotReached));
  static_assert(static_cast<int>(FC::GoalNotReached) <
                static_cast<int>(FC::RecoveryFailed));
  static_assert(static_cast<int>(FC::RecoveryFailed) <
                static_cast<int>(FC::InternalError));
  // 全部 17 个成员 (None + 16 失败码)。
  static_assert(static_cast<int>(FC::InternalError) == 16);
}

TEST(MessagesTest, actionResultsUseUniformFailureField) {
  ayanami::actions::GetPathResult path;
  ayanami::actions::SmoothPathResult smooth;
  ayanami::actions::ExePathResult exe;
  ayanami::actions::RecoveryResult recovery;
  EXPECT_EQ(path.failure, ayanami::FailureCode::None);
  EXPECT_EQ(smooth.failure, ayanami::FailureCode::None);
  EXPECT_EQ(exe.failure, ayanami::FailureCode::None);
  EXPECT_EQ(recovery.failure, ayanami::FailureCode::None);
  EXPECT_TRUE(path.message.empty());
  EXPECT_TRUE(smooth.message.empty());
  EXPECT_TRUE(exe.message.empty());
  EXPECT_TRUE(recovery.message.empty());
}

TEST(MessagesTest, exePathGoalHasPathAndSessionFields) {
  ExePathGoal g;
  EXPECT_EQ(g.session_id, 0u);
  EXPECT_EQ(g.plan_version, 0u);
  g.path.header.frame_id = "map";
  g.path.poses.emplace_back();
  g.path.poses.back().pose = ayanami::geometry_msgs::Pose::fromIsometry(Eigen::Isometry3d::Identity());
  EXPECT_EQ(g.path.poses.size(), 1u);
}

TEST(MessagesTest, msgPtrAliasesExist) {
  static_assert(std::is_same_v<PoseStamped::Ptr, std::shared_ptr<PoseStamped>>);
  static_assert(std::is_same_v<Path::Ptr, std::shared_ptr<Path>>);
  static_assert(std::is_same_v<ExePathGoal::Ptr, std::shared_ptr<ExePathGoal>>);
}
