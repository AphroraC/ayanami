#include <ayanami/interfaces/transform_store.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gtest/gtest.h>

using ayanami::interfaces::TransformStore;

TEST(TransformStoreTest, SetThenLookupSameStamp) {
  auto& store = TransformStore::instance();
  const Eigen::Isometry3d tf =
      Eigen::Translation3d(1.0, 0.0, 0.0) * Eigen::Quaterniond::Identity();
  store.setTransform("map", "odom", TransformStore::fromSeconds(0.0), tf);

  Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
  EXPECT_TRUE(store.lookupTransform("map", "odom",
                                    TransformStore::fromSeconds(0.0), out,
                                    0.0));
  EXPECT_DOUBLE_EQ(1.0, out.translation().x());
}

TEST(TransformStoreTest, LookupMissingEdgeReturnsFalse) {
  auto& store = TransformStore::instance();
  Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
  EXPECT_FALSE(store.lookupTransform("world", "odom",
                                     TransformStore::fromSeconds(5.0), out,
                                     0.0));
}
