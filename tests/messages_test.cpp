#include <gtest/gtest.h>

#include <ayanami/messages/geometry_msgs.hpp>
#include <ayanami/messages/nav_msgs.hpp>
#include <ayanami/messages/sensor_msgs.hpp>
#include <ayanami/messages/actions.hpp>

#include <Eigen/Geometry>

#include <type_traits>
#include <vector>

namespace {

using ayanami::actions::ExePathGoal;
using ayanami::actions::RecoveryResult;
using ayanami::geometry_msgs::PoseStamped;
using ayanami::nav_msgs::Path;

} // namespace

TEST(MessagesTest, PoseStampedHasHeaderAndIsometry) {
  PoseStamped p = PoseStamped::fromIsometry(
      "map", Eigen::Translation3d(1.0, 0.0, 0.0) * Eigen::Quaterniond::Identity());
  EXPECT_EQ(p.header.frame_id, "map");
  EXPECT_DOUBLE_EQ(p.pose.translation().x(), 1.0);
  EXPECT_DOUBLE_EQ(p.pose.translation().y(), 0.0);
  EXPECT_EQ(p.toIsometry().matrix(), p.pose.matrix());
}

TEST(MessagesTest, PathPosesIsPoseStampedVector) {
  static_assert(std::is_same_v<Path::poses_type::value_type, PoseStamped>);
  Path path;
  path.poses.emplace_back();
  path.poses.back().header.frame_id = "map";
  path.poses.back().pose = Eigen::Isometry3d::Identity();
  EXPECT_EQ(path.poses.size(), 1u);
}

TEST(MessagesTest, RecoveryResultErrorCodes) {
  EXPECT_EQ(RecoveryResult::SUCCESS, 0u);
  EXPECT_GE(RecoveryResult::INTERNAL_ERROR, 300u);
}

TEST(MessagesTest, ExePathGoalHasPathAndSessionFields) {
  ExePathGoal g;
  EXPECT_EQ(g.session_id, 0u);
  EXPECT_EQ(g.plan_version, 0u);
  g.path.header.frame_id = "map";
  g.path.poses.emplace_back();
  g.path.poses.back().pose = Eigen::Isometry3d::Identity();
  EXPECT_EQ(g.path.poses.size(), 1u);
}

TEST(MessagesTest, MsgPtrAliasesExist) {
  static_assert(std::is_same_v<PoseStamped::Ptr, std::shared_ptr<PoseStamped>>);
  static_assert(std::is_same_v<Path::Ptr, std::shared_ptr<Path>>);
  static_assert(std::is_same_v<ExePathGoal::Ptr, std::shared_ptr<ExePathGoal>>);
}
