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
  // 0 秒 == latest 哨兵 (显式 latest 语义一致)。
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

// ---- 时间缓存/按戳回溯 (fast_tf 窗口配置 + tf2 式 latest common time) ----

TEST(TransformStoreTest, configureCacheDurationExtendsLookupHorizon) {
  auto& store = TransformStore::instance();
 // 样本间隔 1s: 默认 500ms 窗下旧样本会被剪枝 (1.0s 查询失败);
 // 配置 10s 窗后同样本序列的中间时刻可插值命中。
  store.configureCacheDuration(10.0);
  const auto t1 = ayanami::timestampFromSeconds(100.0);
  const auto t2 = ayanami::timestampFromSeconds(101.0);
  store.setTransform("win_odom", "win_link", t1, makeTranslation(1.0));
  store.setTransform("win_odom", "win_link", t2, makeTranslation(3.0));

  Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
 // 窗口内中间时刻 → 插值。
  EXPECT_TRUE(store.lookupTransform("win_odom", "win_link",
                                    ayanami::timestampFromSeconds(100.5), out,
                                    0.0));
  EXPECT_DOUBLE_EQ(2.0, out.translation().x());
 // 超出配置窗口 (早于 最老样本 100.0 - 窗口) → 仍拒绝过去外推。
  EXPECT_FALSE(store.lookupTransform("win_odom", "win_link",
                                     ayanami::timestampFromSeconds(90.0), out,
                                     0.0));
}

TEST(TransformStoreTest, outOfOrderInsertInterpolates) {
  auto& store = TransformStore::instance();
  store.configureCacheDuration(10.0);
  store.setTransform("ooo_odom", "ooo_link",
                     ayanami::timestampFromSeconds(5.0), makeTranslation(5.0));
 // 乱序写入: 旧样本后到 (fast_tf map 存储支持任意顺序插入)。
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
  store.configureCacheDuration(10.0);  // 同时重置 buffer, 保证测试隔离
 // map→odom 最新样本 2.0s; odom→base_link 最新样本 2.5s。
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

 // 多跳 latest 查询 == 在 common time 的显式查询 (tf2 式一致性快照,
 // 不再是各边各取最新的混合)。
  Eigen::Isometry3d via_latest = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d via_common = Eigen::Isometry3d::Identity();
  ASSERT_TRUE(store.lookupTransform("map", "base_link",
                                    ayanami::Timestamp::latest(), via_latest,
                                    0.0));
  ASSERT_TRUE(store.lookupTransform("map", "base_link", common, via_common,
                                    0.0));
  EXPECT_DOUBLE_EQ(via_latest.translation().x(),
                   via_common.translation().x());
 // common time 采样: map←odom @2.0 (x=2) * odom←base_link @2.0
 // (在 [1.5, 2.5] 间插值 → x=15) → 总 x = 17 (各边取最新会是 2+20=22)。
  EXPECT_DOUBLE_EQ(via_latest.translation().x(), 17.0);

 // 未连通帧 → false。
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
  // 早于首条数据时刻 → 外推失败 (false), 非隐式 latest。
  EXPECT_FALSE(store.lookupTransform("early_odom", "early_link",
                                     ayanami::timestampFromSeconds(0.1), out,
                                     0.0));
  // 数据时刻本身可命中。
  EXPECT_TRUE(store.lookupTransform("early_odom", "early_link",
                                    ayanami::timestampFromSeconds(0.5), out,
                                    0.0));
}