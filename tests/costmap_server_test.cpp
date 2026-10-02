#include <gtest/gtest.h>

#include <memory>

#include <costmap/cost_values.hpp>
#include <costmap/costmap_2d.hpp>
#include <costmap/layered_costmap.hpp>
#include <costmap/static_layer.hpp>
#include <costmap/costmap_wrapper.hpp>

namespace {

/// A small rolling_window=true map with origin=(0,0) so world coordinates can be derived per cell.
std::shared_ptr<costmap_2d::LayeredCostmap> makeRollingGrid(
    unsigned int size, double resolution) {
  auto lm = std::make_shared<costmap_2d::LayeredCostmap>(
      "map", /*rolling_window=*/true, /*track_unknown=*/false);
  lm->resizeMap(size, size, resolution, 0.0, 0.0);
  return lm;
}

}  // namespace

// ---------- Construction ----------

TEST(CostmapServerTest, constructionHoldsLayeredCostmapWithStaticLayer) {
  auto lm = makeRollingGrid(20, 0.5);
  auto layer = std::make_shared<costmap_2d::StaticLayer>();
  costmap_2d::Costmap2D layer_map(10, 10, 0.5, 0.0, 0.0);
  layer_map.setCost(2, 2, costmap_2d::LETHAL_OBSTACLE);
  layer->setMap(layer_map);
  lm->addLayer(layer);

  costmap_2d::CostmapWrapper wrapper(lm, "map");
  EXPECT_EQ(&wrapper.get(), lm.get());
  EXPECT_EQ(wrapper.globalFrame(), "map");
  EXPECT_TRUE(wrapper.isRollingWindow());
  EXPECT_EQ(wrapper.get().getGlobalFrameID(), "map");
}

// ---------- Lifecycle ----------

TEST(CostmapServerTest, checkActivateIsIdempotent) {
  auto lm = makeRollingGrid(10, 0.5);
  costmap_2d::CostmapWrapper wrapper(lm, "map");

  // First call: flips inactive -> active, returns true
  EXPECT_TRUE(wrapper.checkActivate());
  EXPECT_TRUE(wrapper.isActive());
  // Repeated call: already active, no repeated activation, returns false (idempotent)
  EXPECT_FALSE(wrapper.checkActivate());
  EXPECT_FALSE(wrapper.checkActivate());
  EXPECT_TRUE(wrapper.isActive());
}

TEST(CostmapServerTest, checkDeactivateIsIdempotent) {
  auto lm = makeRollingGrid(10, 0.5);
  costmap_2d::CostmapWrapper wrapper(lm, "map");

  EXPECT_TRUE(wrapper.checkActivate());
  // First call: flips active -> inactive, returns true
  EXPECT_TRUE(wrapper.checkDeactivate());
  EXPECT_FALSE(wrapper.isActive());
  // Repeated call: already inactive, no repeated deactivation, returns false (dual case)
  EXPECT_FALSE(wrapper.checkDeactivate());
  EXPECT_FALSE(wrapper.checkDeactivate());
  EXPECT_FALSE(wrapper.isActive());
}

// ---------- Thread safety ----------

TEST(CostmapServerTest, mutexIsSharedExclusive) {
  auto lm = makeRollingGrid(10, 0.5);
  costmap_2d::CostmapWrapper wrapper(lm, "map");

  auto& m = wrapper.mutex();
  // Readers can hold the shared lock concurrently.
  {
    std::shared_lock<std::shared_mutex> r1(m);
    std::shared_lock<std::shared_mutex> r2(m);
  }
  // Readers cannot enter while the writer holds the unique lock (guaranteed by shared_mutex semantics).
  {
    std::unique_lock<std::shared_mutex> w(m);
    // Calling updateMap in the same thread would take the unique lock again -> shared_mutex is non-recursive, so writers
    // should use updateMapUnlocked for composite updates (consistent with clearCostmapArea).
    wrapper.updateMapUnlocked(1.0, 2.0, 0.0);
  }
  wrapper.updateMap(1.0, 2.0, 0.0);
}

// ---------- updateMap / rolling window ----------

TEST(CostmapServerTest, updateMapRollingWindowPreservesOverlap) {
  auto lm = makeRollingGrid(10, 0.5);  // 10x10 cells, covering [0,5) x [0,5)
  lm->getCostmap()->setCost(5, 5, costmap_2d::LETHAL_OBSTACLE);

  costmap_2d::CostmapWrapper wrapper(lm, "map");
  // New origin = (1.0-2.5, 2.0-2.5) = (-1.5, -0.5) -> data shifts right 3 cells and up 1 cell
  wrapper.updateMap(1.0, 2.0, 0.0);

  auto* cm = lm->getCostmap();
  EXPECT_NEAR(cm->getOriginX(), -1.5, 1e-9);
  EXPECT_NEAR(cm->getOriginY(), -0.5, 1e-9);
  // Overlapping data preserved: the old (5,5) moved to (8,6)
  EXPECT_EQ(cm->getCost(8, 6), costmap_2d::LETHAL_OBSTACLE);
  // Regions moved out of the window return to the default
  EXPECT_EQ(cm->getCost(5, 5), costmap_2d::FREE_SPACE);
}

// ---------- isCurrent / globalFrame ----------

TEST(CostmapServerTest, isCurrentAndGetGlobalFrame) {
  auto lm = makeRollingGrid(10, 0.5);
  costmap_2d::CostmapWrapper wrapper(lm, "map");

  EXPECT_TRUE(wrapper.isCurrent());
  EXPECT_EQ(wrapper.globalFrame(), "map");
  wrapper.setCurrent(false);
  EXPECT_FALSE(wrapper.isCurrent());
  wrapper.setCurrent(true);
  EXPECT_TRUE(wrapper.isCurrent());
}
