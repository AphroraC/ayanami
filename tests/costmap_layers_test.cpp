#include <gtest/gtest.h>

#include <fstream>
#include <limits>

#include <costmap_2d/cost_values.hpp>
#include <costmap_2d/costmap_2d.hpp>
#include <costmap_2d/layer.hpp>
#include <costmap_2d/layered_costmap.hpp>
#include <costmap_2d_layers/inflation_layer.hpp>
#include <costmap_2d_layers/simple_obstacle_layer.hpp>
#include <costmap_2d_layers/static_layer.hpp>

namespace {

// 建一个 origin=(0,0) 的小地图, 便于按 cell 推世界坐标。
costmap_2d::Costmap2D makeGrid(unsigned int size_x, unsigned int size_y,
                               double resolution) {
  return costmap_2d::Costmap2D(size_x, size_y, resolution, 0.0, 0.0,
                               costmap_2d::FREE_SPACE);
}

void driveBounds(costmap_2d::Layer& layer, double& min_x, double& min_y,
                 double& max_x, double& max_y) {
  layer.updateBounds(0.0, 0.0, 0.0, &min_x, &min_y, &max_x, &max_y);
}

}  // namespace

// ---------- StaticLayer ----------

TEST(CostmapLayersTest, StaticLayerInlineMapCoversFullMap) {
  constexpr double res = 0.5;
  auto master = makeGrid(20, 20, res);

  auto layer_map = makeGrid(10, 10, res);
  layer_map.setCost(0, 0, costmap_2d::FREE_SPACE);
  layer_map.setCost(5, 5, costmap_2d::LETHAL_OBSTACLE);
  layer_map.setCost(9, 9, costmap_2d::NO_INFORMATION);

  costmap_2d_layers::StaticLayer layer;
  layer.setMap(layer_map);

  double min_x = std::numeric_limits<double>::max();
  double min_y = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double max_y = std::numeric_limits<double>::lowest();
  driveBounds(layer, min_x, min_y, max_x, max_y);
  EXPECT_DOUBLE_EQ(min_x, 0.0);
  EXPECT_DOUBLE_EQ(min_y, 0.0);
  EXPECT_DOUBLE_EQ(max_x, 10.0 * res);
  EXPECT_DOUBLE_EQ(max_y, 10.0 * res);

  layer.updateCosts(master, 0, 0, static_cast<int>(master.getSizeInCellsX()),
                    static_cast<int>(master.getSizeInCellsY()));
  EXPECT_EQ(master.getCost(0, 0), costmap_2d::FREE_SPACE);
  EXPECT_EQ(master.getCost(5, 5), costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(master.getCost(9, 9), costmap_2d::NO_INFORMATION);

  // 层地图之外的区域保持默认值 (不受 StaticLayer 影响)
  EXPECT_EQ(master.getCost(15, 15), costmap_2d::FREE_SPACE);

  // 三种 cost 值都要出现在 master grid 中
  const unsigned char* cmap = master.getCharMap();
  bool has_free = false;
  bool has_lethal = false;
  bool has_unknown = false;
  for (unsigned int i = 0; i < master.getSizeInCellsX() * master.getSizeInCellsY();
       ++i) {
    if (cmap[i] == costmap_2d::FREE_SPACE) {
      has_free = true;
    }
    if (cmap[i] == costmap_2d::LETHAL_OBSTACLE) {
      has_lethal = true;
    }
    if (cmap[i] == costmap_2d::NO_INFORMATION) {
      has_unknown = true;
    }
  }
  EXPECT_TRUE(has_free);
  EXPECT_TRUE(has_lethal);
  EXPECT_TRUE(has_unknown);
}

TEST(CostmapLayersTest, StaticLayerLoadsPgm) {
  // 写一个 3x3 P5 PGM: 0(black)->LETHAL, 255(white)->FREE, 128->NO_INFORMATION
  const char* path = "/tmp/opencode/static_layer_test.pgm";
  {
    std::ofstream out(path, std::ios::binary);
    out << "P5\n# ayanami test\n3 3\n255\n";
    const unsigned char pixels[9] = {0, 255, 128, 128, 0, 255, 255, 128, 0};
    out.write(reinterpret_cast<const char*>(pixels), sizeof(pixels));
  }

  auto master = makeGrid(10, 10, 0.5);
  costmap_2d_layers::StaticLayer layer;
  layer.setMapPath(path, 0.5, 0.0, 0.0);

  layer.updateCosts(master, 0, 0, static_cast<int>(master.getSizeInCellsX()),
                    static_cast<int>(master.getSizeInCellsY()));
  // PGM rows start at the image top; map rows start at world -Y. Match Nav2's
  // height-y-1 conversion so the map is not mirrored across the X axis.
  EXPECT_EQ(master.getCost(0, 0), costmap_2d::FREE_SPACE);
  EXPECT_EQ(master.getCost(1, 0), costmap_2d::NO_INFORMATION);
  EXPECT_EQ(master.getCost(2, 0), costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(master.getCost(0, 2), costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(master.getCost(1, 2), costmap_2d::FREE_SPACE);

  std::remove(path);
}

TEST(CostmapLayersTest, StaticLayerResizesNonRollingMasterFromMap) {
  auto layered =
      std::make_shared<costmap_2d::LayeredCostmap>("map", false, false);
  layered->resizeMap(3, 4, 1.0, 0.0, 0.0);
  auto layer = std::make_shared<costmap_2d_layers::StaticLayer>();
  layered->addLayer(layer);

  costmap_2d::Costmap2D first(7, 5, 0.25, -1.0, -2.0,
                              costmap_2d::FREE_SPACE);
  first.setCost(6, 4, costmap_2d::LETHAL_OBSTACLE);
  layer->setMap(first);
  layered->updateMap(0.0, 0.0, 0.0);
  EXPECT_EQ(layered->getCostmap()->getSizeInCellsX(), 7u);
  EXPECT_EQ(layered->getCostmap()->getSizeInCellsY(), 5u);
  EXPECT_DOUBLE_EQ(layered->getCostmap()->getResolution(), 0.25);
  EXPECT_DOUBLE_EQ(layered->getCostmap()->getOriginX(), -1.0);
  EXPECT_EQ(layered->getCostmap()->getCost(6, 4),
            costmap_2d::LETHAL_OBSTACLE);

  costmap_2d::Costmap2D replacement(2, 3, 0.5, 4.0, 5.0,
                                    costmap_2d::FREE_SPACE);
  replacement.setCost(1, 2, costmap_2d::LETHAL_OBSTACLE);
  layer->setMap(replacement);
  layered->updateMap(4.0, 5.0, 0.0);
  EXPECT_EQ(layered->getCostmap()->getSizeInCellsX(), 2u);
  EXPECT_EQ(layered->getCostmap()->getSizeInCellsY(), 3u);
  EXPECT_DOUBLE_EQ(layered->getCostmap()->getOriginY(), 5.0);
  EXPECT_EQ(layered->getCostmap()->getCost(1, 2),
            costmap_2d::LETHAL_OBSTACLE);
}

TEST(CostmapLayersTest, StaticLayerQueuedMapAppliesInUpdateCycle) {
  auto layered =
      std::make_shared<costmap_2d::LayeredCostmap>("map", false, false);
  auto layer = std::make_shared<costmap_2d_layers::StaticLayer>();
  layered->addLayer(layer);
  costmap_2d::Costmap2D first(2, 2, 1.0, 0.0, 0.0,
                              costmap_2d::FREE_SPACE);
  layer->setMap(first);
  layered->updateMap(0.0, 0.0, 0.0);

  costmap_2d::Costmap2D queued(4, 3, 0.25, -2.0, -1.0,
                               costmap_2d::FREE_SPACE);
  queued.setCost(3, 2, costmap_2d::LETHAL_OBSTACLE);
  layer->queueMap(queued);
  EXPECT_EQ(layered->getCostmap()->getSizeInCellsX(), 2u);
  layered->updateMap(0.0, 0.0, 0.0);
  EXPECT_EQ(layered->getCostmap()->getSizeInCellsX(), 4u);
  EXPECT_EQ(layered->getCostmap()->getSizeInCellsY(), 3u);
  EXPECT_EQ(layered->getCostmap()->getCost(3, 2),
            costmap_2d::LETHAL_OBSTACLE);
}

// ---------- SimpleObstacleLayer ----------

TEST(CostmapLayersTest, SimpleObstacleLayerMarkThenClear) {
  constexpr double res = 0.5;
  auto master = makeGrid(20, 20, res);

  costmap_2d_layers::SimpleObstacleLayer layer;

  double wx = 0.0;
  double wy = 0.0;
  master.mapToWorld(5, 5, wx, wy);
  layer.markObstacles(wx, wy);
  double nwx = 0.0;
  double nwy = 0.0;
  master.mapToWorld(4, 5, nwx, nwy);
  layer.markObstacles(nwx, nwy);

  layer.updateCosts(master, 0, 0, static_cast<int>(master.getSizeInCellsX()),
                    static_cast<int>(master.getSizeInCellsY()));
  EXPECT_EQ(master.getCost(5, 5), costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(master.getCost(4, 5), costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(master.getCost(0, 0), costmap_2d::FREE_SPACE);

  // clear 恰好盖住 cell (5,5) 的世界矩形 [2.5,3.0) x [2.5,3.0)
  layer.clearArea(2.5, 2.5, 3.0, 3.0);
  layer.updateCosts(master, 0, 0, static_cast<int>(master.getSizeInCellsX()),
                    static_cast<int>(master.getSizeInCellsY()));
  EXPECT_EQ(master.getCost(5, 5), costmap_2d::FREE_SPACE);
  EXPECT_EQ(master.getCost(4, 5), costmap_2d::LETHAL_OBSTACLE);
}

// ---------- InflationLayer ----------

TEST(CostmapLayersTest, InflationLayerInflatesAroundLethal) {
  constexpr double res = 0.5;
  auto master = makeGrid(30, 30, res);
  master.setCost(5, 5, costmap_2d::LETHAL_OBSTACLE);

  costmap_2d_layers::InflationLayer layer;
  layer.setInflationRadius(3.0 * res);  // 3 cells
  layer.setInscribedRadius(1.0 * res);  // 1 cell
  layer.setCostScalingFactor(2.0);

  double min_x = std::numeric_limits<double>::max();
  double min_y = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double max_y = std::numeric_limits<double>::lowest();
  driveBounds(layer, min_x, min_y, max_x, max_y);

  layer.updateCosts(master, 0, 0, static_cast<int>(master.getSizeInCellsX()),
                    static_cast<int>(master.getSizeInCellsY()));

  // 障碍 cell 本身保持 lethal
  EXPECT_EQ(master.getCost(5, 5), costmap_2d::LETHAL_OBSTACLE);
  // 正交 4 邻域 (距离 1 cell) 落入 inscribed band -> 253
  EXPECT_EQ(master.getCost(5, 6), costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  EXPECT_EQ(master.getCost(4, 5), costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  EXPECT_EQ(master.getCost(6, 5), costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  EXPECT_EQ(master.getCost(5, 4), costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  // 对角 8 邻域 (距离 sqrt(2) cell) 有非零膨胀值
  EXPECT_GE(master.getCost(4, 4), 100);
  EXPECT_GE(master.getCost(6, 6), 100);
  // 衰减: 离障碍越远 cost 越低
  EXPECT_LT(master.getCost(5, 7), master.getCost(5, 6));
  EXPECT_GT(master.getCost(5, 7), 0);
  // 超过 inflation_radius (3 cells) 的 cell 保持默认 0
  EXPECT_EQ(master.getCost(5, 12), costmap_2d::FREE_SPACE);
}

TEST(CostmapLayersTest, InflationRadiusShrinkRemovesOldBand) {
  auto lm = std::make_shared<costmap_2d::LayeredCostmap>("map", false, false);
  lm->resizeMap(30, 30, 0.5, 0.0, 0.0);
  auto obstacles =
      std::make_shared<costmap_2d_layers::SimpleObstacleLayer>();
  auto inflation = std::make_shared<costmap_2d_layers::InflationLayer>();
  inflation->setInflationRadius(2.0);
  inflation->setInscribedRadius(0.5);
  lm->addLayer(obstacles);
  lm->addLayer(inflation);
  obstacles->markObstacles(7.75, 7.75);
  lm->updateMap(0.0, 0.0, 0.0);
  ASSERT_GT(lm->getCostmap()->getCost(17, 15), 0);

  inflation->setInflationRadius(0.5);
  lm->updateMap(0.0, 0.0, 0.0);
  EXPECT_EQ(lm->getCostmap()->getCost(17, 15), costmap_2d::FREE_SPACE);
}

TEST(CostmapLayersTest, ObstacleMaximumOverridesUnknown) {
  costmap_2d::Costmap2D master(10, 10, 1.0, 0.0, 0.0,
                               costmap_2d::NO_INFORMATION);
  costmap_2d_layers::SimpleObstacleLayer layer;
  layer.markObstacles(2.5, 2.5);
  layer.updateCosts(master, 0, 0, 10, 10);
  EXPECT_EQ(master.getCost(2, 2), costmap_2d::LETHAL_OBSTACLE);
}

TEST(CostmapLayersTest, InflationLayerClipsRadiusTo1000) {
  constexpr double res = 0.5;
  auto master = makeGrid(30, 30, res);
  master.setCost(5, 5, costmap_2d::LETHAL_OBSTACLE);

  costmap_2d_layers::InflationLayer layer;
  layer.setInflationRadius(1e6);  // 换算成 cells 远超 1000, 应 clamp 且警告
  layer.setInscribedRadius(0.5);
  layer.setCostScalingFactor(2.0);

  layer.updateCosts(master, 0, 0, static_cast<int>(master.getSizeInCellsX()),
                    static_cast<int>(master.getSizeInCellsY()));

  // 半径被 clamp 到 1000 cells 后仍能正常膨胀: 源保持 lethal, 正交邻域 inscribed
  EXPECT_EQ(master.getCost(5, 5), costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(master.getCost(5, 6), costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  // 远离障碍处 (距离 7 cells) 仅有极低衰减 cost, 不再是 lethal
  EXPECT_LT(master.getCost(5, 12), costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
}
