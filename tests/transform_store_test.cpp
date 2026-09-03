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
  // fromSeconds 仅作兼容 helper: 指定秒 → 纳秒计数。
  EXPECT_EQ(TransformStore::fromSeconds(5.0).count(), 5000000000LL);
  // 0 秒 == latest 哨兵 (显式 latest() 语义一致)。
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
  // 中间时刻 0.2s: fast_tf 在 [0.1, 0.3] 间线性插值 → x = 2.0。
  EXPECT_TRUE(store.lookupTransform("base_odom", "base_link_stamp",
                                    ayanami::timestampFromSeconds(0.2), out,
                                    0.0));
  EXPECT_DOUBLE_EQ(2.0, out.translation().x());

  // 精确命中较新条目。
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
  // 显式 latest 哨兵 → 最新条目 (0.3s)。
  EXPECT_TRUE(store.lookupTransform("latest_odom", "latest_link",
                                    ayanami::Timestamp::latest(), out, 0.0));
  EXPECT_DOUBLE_EQ(3.0, out.translation().x());
}

TEST(TransformStoreTest, lookupBeforeFirstStampReturnsFalse) {
  auto& store = TransformStore::instance();
  store.setTransform("early_odom", "early_link", ayanami::timestampFromSeconds(0.5),
                     makeTranslation(1.0));

  Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
  // 早于首条数据时刻 → 外推失败 (false), 非隐式 latest。
  EXPECT_FALSE(store.lookupTransform("early_odom", "early_link",
                                     ayanami::timestampFromSeconds(0.1), out,
                                     0.0));
  // 数据时刻本身可命中。
  EXPECT_TRUE(store.lookupTransform("early_odom", "early_link",
                                    ayanami::timestampFromSeconds(0.5), out,
                                    0.0));
}