#include <gtest/gtest.h>

#include <memory>

#include <costmap_2d/cost_values.hpp>
#include <costmap_2d/costmap_2d.hpp>
#include <costmap_2d/layered_costmap.hpp>
#include <costmap_2d_layers/static_layer.hpp>
#include <costmap_server/costmap_wrapper.hpp>

namespace {

// rolling_window=true, origin=(0,0) 的小地图, 便于按 cell 推世界坐标。
std::shared_ptr<costmap_2d::LayeredCostmap> makeRollingGrid(
    unsigned int size, double resolution) {
  auto lm = std::make_shared<costmap_2d::LayeredCostmap>(
      "map", /*rolling_window=*/true, /*track_unknown=*/false);
  lm->resizeMap(size, size, resolution, 0.0, 0.0);
  return lm;
}

}  // namespace

// ---------- 构造 ----------

TEST(CostmapServerTest, constructionHoldsLayeredCostmapWithStaticLayer) {
  auto lm = makeRollingGrid(20, 0.5);
  auto layer = std::make_shared<costmap_2d_layers::StaticLayer>();
  costmap_2d::Costmap2D layer_map(10, 10, 0.5, 0.0, 0.0);
  layer_map.setCost(2, 2, costmap_2d::LETHAL_OBSTACLE);
  layer->setMap(layer_map);
  lm->addLayer(layer);

  costmap_2d::CostmapWrapper wrapper(lm, "map");
  EXPECT_EQ(&wrapper.get(), lm.get());
  EXPECT_EQ(wrapper.getGlobalFrame(), "map");
  EXPECT_TRUE(wrapper.isRollingWindow());
  EXPECT_EQ(wrapper.get().getGlobalFrameID(), "map");
}

// ---------- 生命周期 ----------

TEST(CostmapServerTest, checkActivateIsIdempotent) {
  auto lm = makeRollingGrid(10, 0.5);
  costmap_2d::CostmapWrapper wrapper(lm, "map");

  // 首次: inactive -> active 翻转, 返回 true
  EXPECT_TRUE(wrapper.checkActivate());
  EXPECT_TRUE(wrapper.isActive());
  // 重复调用: 已是 active, 不再重复 act, 返回 false (幂等)
  EXPECT_FALSE(wrapper.checkActivate());
  EXPECT_FALSE(wrapper.checkActivate());
  EXPECT_TRUE(wrapper.isActive());
}

TEST(CostmapServerTest, checkDeactivateIsIdempotent) {
  auto lm = makeRollingGrid(10, 0.5);
  costmap_2d::CostmapWrapper wrapper(lm, "map");

  EXPECT_TRUE(wrapper.checkActivate());
  // 首次: active -> inactive 翻转, 返回 true
  EXPECT_TRUE(wrapper.checkDeactivate());
  EXPECT_FALSE(wrapper.isActive());
  // 重复调用: 已是 inactive, 不再重复 act, 返回 false (对偶)
  EXPECT_FALSE(wrapper.checkDeactivate());
  EXPECT_FALSE(wrapper.checkDeactivate());
  EXPECT_FALSE(wrapper.isActive());
}

// ---------- 线程安全 ----------

TEST(CostmapServerTest, mutexIsSharedExclusive) {
  auto lm = makeRollingGrid(10, 0.5);
  costmap_2d::CostmapWrapper wrapper(lm, "map");

  auto& m = wrapper.mutex();
  // 读者可并发持 shared 锁。
  {
    std::shared_lock<std::shared_mutex> r1(m);
    std::shared_lock<std::shared_mutex> r2(m);
  }
  // 写者 unique 锁期间读者不可进入 (语义由 shared_mutex 保证)。
  {
    std::unique_lock<std::shared_mutex> w(m);
    // 同一线程内 updateMap 会再次取 unique 锁 -> shared_mutex 非递归, 写者
    // 复合更新应使用 updateMapUnlocked (与 clearCostmapArea 一致)。
    wrapper.updateMapUnlocked(1.0, 2.0, 0.0);
  }
  wrapper.updateMap(1.0, 2.0, 0.0);
}

// ---------- updateMap / rolling window ----------

TEST(CostmapServerTest, updateMapRollingWindowPreservesOverlap) {
  auto lm = makeRollingGrid(10, 0.5);  // 10x10 cells, 覆盖 [0,5) x [0,5)
  lm->getCostmap()->setCost(5, 5, costmap_2d::LETHAL_OBSTACLE);

  costmap_2d::CostmapWrapper wrapper(lm, "map");
  // 新 origin = (1.0-2.5, 2.0-2.5) = (-1.5, -0.5) -> 数据右移 3 cells, 上移 1 cell
  wrapper.updateMap(1.0, 2.0, 0.0);

  auto* cm = lm->getCostmap();
  EXPECT_NEAR(cm->getOriginX(), -1.5, 1e-9);
  EXPECT_NEAR(cm->getOriginY(), -0.5, 1e-9);
  // 重叠数据保留: 原 (5,5) 移到 (8,6)
  EXPECT_EQ(cm->getCost(8, 6), costmap_2d::LETHAL_OBSTACLE);
  // 移出区域恢复默认
  EXPECT_EQ(cm->getCost(5, 5), costmap_2d::FREE_SPACE);
}

// ---------- isCurrent / getGlobalFrame ----------

TEST(CostmapServerTest, isCurrentAndGetGlobalFrame) {
  auto lm = makeRollingGrid(10, 0.5);
  costmap_2d::CostmapWrapper wrapper(lm, "map");

  EXPECT_TRUE(wrapper.isCurrent());
  EXPECT_EQ(wrapper.getGlobalFrame(), "map");
  wrapper.setCurrent(false);
  EXPECT_FALSE(wrapper.isCurrent());
  wrapper.setCurrent(true);
  EXPECT_TRUE(wrapper.isCurrent());
}
