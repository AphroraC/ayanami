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

/// Builds a small map with origin=(0,0) so world coordinates can be derived per cell.
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

TEST(CostmapLayersTest, staticLayerInlineMapCoversFullMap) {
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

  // Regions outside the layer map keep the default value (unaffected by StaticLayer)
  EXPECT_EQ(master.getCost(15, 15), costmap_2d::FREE_SPACE);

  // All three cost values must appear in the master grid
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

TEST(CostmapLayersTest, staticLayerLoadsPgm) {
  // Write a 3x3 P5 PGM: 0(black)->LETHAL, 255(white)->FREE, 128->NO_INFORMATION
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

TEST(CostmapLayersTest, staticLayerResizesNonRollingMasterFromMap) {
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

TEST(CostmapLayersTest, staticLayerQueuedMapAppliesInUpdateCycle) {
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

TEST(CostmapLayersTest, simpleObstacleLayerMarkThenClear) {
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

  // clear covers exactly the world rectangle [2.5,3.0) x [2.5,3.0) over cell (5,5)
  layer.clearArea(2.5, 2.5, 3.0, 3.0);
  layer.updateCosts(master, 0, 0, static_cast<int>(master.getSizeInCellsX()),
                    static_cast<int>(master.getSizeInCellsY()));
  EXPECT_EQ(master.getCost(5, 5), costmap_2d::FREE_SPACE);
  EXPECT_EQ(master.getCost(4, 5), costmap_2d::LETHAL_OBSTACLE);
}

// ---------- InflationLayer ----------

TEST(CostmapLayersTest, inflationLayerInflatesAroundLethal) {
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

  // The obstacle cell itself stays lethal
  EXPECT_EQ(master.getCost(5, 5), costmap_2d::LETHAL_OBSTACLE);
  // Orthogonal 4-neighborhood (distance 1 cell) falls into the inscribed band -> 253
  EXPECT_EQ(master.getCost(5, 6), costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  EXPECT_EQ(master.getCost(4, 5), costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  EXPECT_EQ(master.getCost(6, 5), costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  EXPECT_EQ(master.getCost(5, 4), costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  // Diagonal 8-neighborhood (distance sqrt(2) cells) has a non-zero inflation value
  EXPECT_GE(master.getCost(4, 4), 100);
  EXPECT_GE(master.getCost(6, 6), 100);
  // Decay: the farther from the obstacle, the lower the cost
  EXPECT_LT(master.getCost(5, 7), master.getCost(5, 6));
  EXPECT_GT(master.getCost(5, 7), 0);
  // Cells beyond inflation_radius (3 cells) keep the default 0
  EXPECT_EQ(master.getCost(5, 12), costmap_2d::FREE_SPACE);
}

TEST(CostmapLayersTest, inflationLayerSmoothGradientWithRealisticParams) {
  // Real config parameters (costmap_navigation.json): res 0.05, inflation 0.5,
  // inscribed 0.3, csf 10.0. Assert the cost decays strictly beyond the inscribed band and tapers smoothly to zero at the end
  // (a gradient rather than a single color when visualized).
  constexpr double res = 0.05;
  auto master = makeGrid(60, 60, res);
  master.setCost(30, 30, costmap_2d::LETHAL_OBSTACLE);

  costmap_2d_layers::InflationLayer layer;
  layer.setInflationRadius(0.5);
  layer.setInscribedRadius(0.3);
  layer.setCostScalingFactor(10.0);

  layer.updateCosts(master, 0, 0, static_cast<int>(master.getSizeInCellsX()),
                    static_cast<int>(master.getSizeInCellsY()));

  // The inscribed band (< 0.3m) is constantly 253; the boundary cell (6*0.05, slightly over 0.3 in float) may be 252.
  for (int d = 1; d <= 5; ++d) {
    EXPECT_EQ(master.getCost(30 + d, 30),
              costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
        << "d=" << d;
  }
  EXPECT_GE(master.getCost(36, 30), 252);
  // The decay band (0.3m to 0.5m) decreases strictly.
  int prev = costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
  for (int d = 7; d <= 10; ++d) {
    const int cost = master.getCost(30 + d, 30);
    EXPECT_LT(cost, prev) << "d=" << d << " cost=" << cost;
    prev = cost;
  }
  // At the end (0.5m) it tapers smoothly to zero: csf=10 -> 252*e^-2 ~ 34.
  const int edge_cost = master.getCost(30 + 10, 30);
  EXPECT_GT(edge_cost, 0);
  EXPECT_LT(edge_cost, 60);
  // Beyond the radius stays free.
  EXPECT_EQ(master.getCost(30 + 11, 30), costmap_2d::FREE_SPACE);
}

TEST(CostmapLayersTest, inflationRadiusShrinkRemovesOldBand) {
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

TEST(CostmapLayersTest, obstacleMaximumOverridesUnknown) {
  costmap_2d::Costmap2D master(10, 10, 1.0, 0.0, 0.0,
                               costmap_2d::NO_INFORMATION);
  costmap_2d_layers::SimpleObstacleLayer layer;
  layer.markObstacles(2.5, 2.5);
  layer.updateCosts(master, 0, 0, 10, 10);
  EXPECT_EQ(master.getCost(2, 2), costmap_2d::LETHAL_OBSTACLE);
}

TEST(CostmapLayersTest, inflationLayerClipsRadiusTo1000) {
  constexpr double res = 0.5;
  auto master = makeGrid(30, 30, res);
  master.setCost(5, 5, costmap_2d::LETHAL_OBSTACLE);

  costmap_2d_layers::InflationLayer layer;
  layer.setInflationRadius(1e6);  // Converts to far more than 1000 cells; should clamp and warn
  layer.setInscribedRadius(0.5);
  layer.setCostScalingFactor(2.0);

  layer.updateCosts(master, 0, 0, static_cast<int>(master.getSizeInCellsX()),
                    static_cast<int>(master.getSizeInCellsY()));

  // After the radius is clamped to 1000 cells inflation still works: the source stays lethal, orthogonal neighbors inscribed
  EXPECT_EQ(master.getCost(5, 5), costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(master.getCost(5, 6), costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  // Far from the obstacle (distance 7 cells) there is only a very low decay cost, no longer lethal
  EXPECT_LT(master.getCost(5, 12), costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
}
