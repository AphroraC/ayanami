#include <ayanami/interfaces/transform_store.hpp>
#include <ayanami/messages/timestamp.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gtest/gtest.h>

using ayanami::interfaces::TransformStore;

namespace {

Eigen::Isometry3d makeTranslation(double x) {
  return Eigen::Translation3d(x, 0.0, 0.0) * Eigen::Quaterniond::Identity();
}

}  // namespace

TEST(TransformStoreTest, setThenLookupExplicitLatest) {
  auto& store = TransformStore::instance();
  store.setTransform("map", "odom", ayanami::Timestamp::latest(),
                     makeTranslation(1.0));

  Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
  EXPECT_TRUE(store.lookupTransform("map", "odom",
                                    ayanami::Timestamp::latest(), out, 0.0));
  EXPECT_DOUBLE_EQ(1.0, out.translation().x());
}

TEST(TransformStoreTest, lookupMissingEdgeReturnsFalse) {
  auto& store = TransformStore::instance();
  Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
  EXPECT_FALSE(store.lookupTransform("world", "odom",
                                     ayanami::Timestamp::latest(), out, 0.0));
}

TEST(TransformStoreTest, fromSecondsCompatHelper) {
  // fromSeconds is only a compatibility helper: seconds -> nanosecond count.
  EXPECT_EQ(TransformStore::fromSeconds(5.0).count(), 5000000000LL);
  // 0 seconds == the latest sentinel (explicit latest semantics agree).
  EXPECT_TRUE(TransformStore::fromSeconds(0.0).isLatest());
  EXPECT_TRUE(ayanami::Timestamp().isLatest());
  EXPECT_TRUE(ayanami::Timestamp::latest().isLatest());
  EXPECT_FALSE(TransformStore::fromSeconds(1.0).isLatest());
}

TEST(TransformStoreTest, specificStampInterpolatesBetweenEntries) {
  auto& store = TransformStore::instance();
  const auto t1 = ayanami::timestampFromSeconds(0.1);
  const auto t2 = ayanami::timestampFromSeconds(0.3);
  store.setTransform("base_odom", "base_link_stamp", t1, makeTranslation(1.0));
  store.setTransform("base_odom", "base_link_stamp", t2, makeTranslation(3.0));

  Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
  // Intermediate time 0.2s: fast_tf interpolates linearly within [0.1, 0.3] -> x = 2.0.
  EXPECT_TRUE(store.lookupTransform("base_odom", "base_link_stamp",
                                    ayanami::timestampFromSeconds(0.2), out,
                                    0.0));
  EXPECT_DOUBLE_EQ(2.0, out.translation().x());

  // Exact hit on the newer entry.
  EXPECT_TRUE(store.lookupTransform("base_odom", "base_link_stamp", t2, out,
                                    0.0));
  EXPECT_DOUBLE_EQ(3.0, out.translation().x());
}

TEST(TransformStoreTest, explicitLatestReturnsNewestEntry) {
  auto& store = TransformStore::instance();
  store.setTransform("latest_odom", "latest_link", ayanami::timestampFromSeconds(0.1),
                     makeTranslation(1.0));
  store.setTransform("latest_odom", "latest_link", ayanami::timestampFromSeconds(0.3),
                     makeTranslation(3.0));

  Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
  // Explicit latest sentinel -> the newest entry (0.3s).
  EXPECT_TRUE(store.lookupTransform("latest_odom", "latest_link",
                                    ayanami::Timestamp::latest(), out, 0.0));
  EXPECT_DOUBLE_EQ(3.0, out.translation().x());
}

// ---- Time cache / lookup-by-stamp (fast_tf window config + tf2-style latest common time) ----

TEST(TransformStoreTest, configureCacheDurationExtendsLookupHorizon) {
  auto& store = TransformStore::instance();
 // Samples spaced 1s apart: with the default 500ms window old samples are pruned (the 1.0s query fails);
 // after configuring a 10s window, intermediate times of the same sample sequence can be hit by interpolation.
  store.configureCacheDuration(10.0);
  const auto t1 = ayanami::timestampFromSeconds(100.0);
  const auto t2 = ayanami::timestampFromSeconds(101.0);
  store.setTransform("win_odom", "win_link", t1, makeTranslation(1.0));
  store.setTransform("win_odom", "win_link", t2, makeTranslation(3.0));

  Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
 // Intermediate time within the window -> interpolation.
  EXPECT_TRUE(store.lookupTransform("win_odom", "win_link",
                                    ayanami::timestampFromSeconds(100.5), out,
                                    0.0));
  EXPECT_DOUBLE_EQ(2.0, out.translation().x());
 // Outside the configured window (older than oldest sample 100.0 - window) -> extrapolation into the past is still rejected.
  EXPECT_FALSE(store.lookupTransform("win_odom", "win_link",
                                     ayanami::timestampFromSeconds(90.0), out,
                                     0.0));
}

TEST(TransformStoreTest, outOfOrderInsertInterpolates) {
  auto& store = TransformStore::instance();
  store.configureCacheDuration(10.0);
  store.setTransform("ooo_odom", "ooo_link",
                     ayanami::timestampFromSeconds(5.0), makeTranslation(5.0));
 // Out-of-order writes: the older sample arrives later (the fast_tf map storage supports insertion in any order).
  store.setTransform("ooo_odom", "ooo_link",
                     ayanami::timestampFromSeconds(4.0), makeTranslation(4.0));

  Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
  EXPECT_TRUE(store.lookupTransform("ooo_odom", "ooo_link",
                                    ayanami::timestampFromSeconds(4.5), out,
                                    0.0));
  EXPECT_DOUBLE_EQ(4.5, out.translation().x());
}

TEST(TransformStoreTest, latestCommonTimeIsMinAcrossChain) {
  auto& store = TransformStore::instance();
  store.configureCacheDuration(10.0);  // Reset the buffer at the same time to keep tests isolated
 // map->odom newest sample 2.0s; odom->base_link newest sample 2.5s.
  store.setTransform("map", "odom", ayanami::timestampFromSeconds(1.0),
                     makeTranslation(1.0));
  store.setTransform("map", "odom", ayanami::timestampFromSeconds(2.0),
                     makeTranslation(2.0));
  store.setTransform("odom", "base_link", ayanami::timestampFromSeconds(1.5),
                     makeTranslation(10.0));
  store.setTransform("odom", "base_link", ayanami::timestampFromSeconds(2.5),
                     makeTranslation(20.0));

  ayanami::Timestamp common;
  ASSERT_TRUE(store.latestCommonTime("map", "base_link", common));
  EXPECT_EQ(common.count(), ayanami::timestampFromSeconds(2.0).count());

 // A multi-hop latest query == an explicit query at common time (a tf2-style consistent snapshot,
 // no longer a mix of per-edge newest lookups).
  Eigen::Isometry3d via_latest = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d via_common = Eigen::Isometry3d::Identity();
  ASSERT_TRUE(store.lookupTransform("map", "base_link",
                                    ayanami::Timestamp::latest(), via_latest,
                                    0.0));
  ASSERT_TRUE(store.lookupTransform("map", "base_link", common, via_common,
                                    0.0));
  EXPECT_DOUBLE_EQ(via_latest.translation().x(),
                   via_common.translation().x());
 // Common-time sampling: map<-odom @2.0 (x=2) * odom<-base_link @2.0
 // (interpolated within [1.5, 2.5] -> x=15) -> total x = 17 (taking each edge newest would give 2+20=22).
  EXPECT_DOUBLE_EQ(via_latest.translation().x(), 17.0);

 // Unconnected frames -> false.
  Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
  EXPECT_FALSE(store.lookupTransform("map", "world_undef",
                                     ayanami::Timestamp::latest(), out, 0.0));
  ayanami::Timestamp unused;
  EXPECT_FALSE(store.latestCommonTime("map", "world_undef", unused));
}

TEST(TransformStoreTest, lookupBeforeFirstStampReturnsFalse) {
  auto& store = TransformStore::instance();
  store.setTransform("early_odom", "early_link", ayanami::timestampFromSeconds(0.5),
                     makeTranslation(1.0));

  Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
  // Earlier than the first data time -> extrapolation fails (false), not an implicit latest.
  EXPECT_FALSE(store.lookupTransform("early_odom", "early_link",
                                     ayanami::timestampFromSeconds(0.1), out,
                                     0.0));
  // The data time itself can be hit.
  EXPECT_TRUE(store.lookupTransform("early_odom", "early_link",
                                    ayanami::timestampFromSeconds(0.5), out,
                                    0.0));
}