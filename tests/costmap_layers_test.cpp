#include <gtest/gtest.h>

#include <fstream>

#include <chrono>
#include <thread>
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

namespace {
const char* k_static_pgm_path = "/tmp/opencode/static_layer_test.pgm";

void writeStaticTestPgm() {
  // 3x3 P5 PGM: black(0) / white(255) / mid-gray(128) cells.
  std::ofstream out(k_static_pgm_path, std::ios::binary);
  out << "P5\n# ayanami test\n3 3\n255\n";
  const unsigned char pixels[9] = {0, 255, 128, 128, 0, 255, 255, 128, 0};
  out.write(reinterpret_cast<const char*>(pixels), sizeof(pixels));
}
}  // namespace

TEST(CostmapLayersTest, staticLayerLoadsPgm) {
  // Map-server trinary semantics (defaults: occupied 0.65 / free 0.196):
  // 0 (black, occupancy 1.0) -> LETHAL, 255 (white, occupancy 0.0) -> FREE,
  // 128 (mid-gray, occupancy ~0.5) -> between thresholds -> resolves through
  // track_unknown (default false -> FREE, so antialiased wall edges no
  // longer become blocked-unknown cells).
  writeStaticTestPgm();

  auto master = makeGrid(10, 10, 0.5);
  costmap_2d_layers::StaticLayer layer;
  layer.setMapPath(k_static_pgm_path, 0.5, 0.0, 0.0);

  layer.updateCosts(master, 0, 0, static_cast<int>(master.getSizeInCellsX()),
                    static_cast<int>(master.getSizeInCellsY()));
  // PGM rows start at the image top; map rows start at world -Y. Match Nav2's
  // height-y-1 conversion so the map is not mirrored across the X axis.
  // Row layout (after the flip): map row 0 = image bottom row {255,128,0},
  // map row 2 = image top row {0,255,128}.
  EXPECT_EQ(master.getCost(0, 0), costmap_2d::FREE_SPACE);
  EXPECT_EQ(master.getCost(1, 0), costmap_2d::FREE_SPACE);
  EXPECT_EQ(master.getCost(2, 0), costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(master.getCost(0, 2), costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(master.getCost(1, 2), costmap_2d::FREE_SPACE);

  std::remove(k_static_pgm_path);
}

TEST(CostmapLayersTest, staticLayerPgmSemanticsVariants) {
  writeStaticTestPgm();

  {
    // trinary + track_unknown: the between-threshold band (pixel 128 at
    // map cells (1,0) and (2,2)) -> NO_INFORMATION; black/white resolve as
    // always and (0,0) is white regardless.
    auto master = makeGrid(10, 10, 0.5);
    costmap_2d_layers::StaticLayer layer;
    costmap_2d_layers::StaticMapSemantics semantics;
    semantics.track_unknown = true;
    layer.setMapPath(k_static_pgm_path, 0.5, 0.0, 0.0, semantics);
    layer.updateCosts(master, 0, 0,
                      static_cast<int>(master.getSizeInCellsX()),
                      static_cast<int>(master.getSizeInCellsY()));
    EXPECT_EQ(master.getCost(0, 0), costmap_2d::FREE_SPACE);
    EXPECT_EQ(master.getCost(1, 0), costmap_2d::NO_INFORMATION);
    EXPECT_EQ(master.getCost(0, 2), costmap_2d::LETHAL_OBSTACLE);
  }
  {
    // negate: white becomes occupied, black becomes free.
    auto master = makeGrid(10, 10, 0.5);
    costmap_2d_layers::StaticLayer layer;
    costmap_2d_layers::StaticMapSemantics semantics;
    semantics.negate = true;
    layer.setMapPath(k_static_pgm_path, 0.5, 0.0, 0.0, semantics);
    layer.updateCosts(master, 0, 0,
                      static_cast<int>(master.getSizeInCellsX()),
                      static_cast<int>(master.getSizeInCellsY()));
    // Under negate the black/white cells swap roles; mid-gray stays
    // between the thresholds either way.
    EXPECT_EQ(master.getCost(0, 0), costmap_2d::LETHAL_OBSTACLE);
    EXPECT_EQ(master.getCost(2, 0), costmap_2d::FREE_SPACE);
    EXPECT_EQ(master.getCost(1, 2), costmap_2d::LETHAL_OBSTACLE);
  }
  {
    // scale: the between-threshold band maps linearly to a sub-lethal cost.
    auto master = makeGrid(10, 10, 0.5);
    costmap_2d_layers::StaticLayer layer;
    costmap_2d_layers::StaticMapSemantics semantics;
    semantics.trinary = false;
    layer.setMapPath(k_static_pgm_path, 0.5, 0.0, 0.0, semantics);
    layer.updateCosts(master, 0, 0,
                      static_cast<int>(master.getSizeInCellsX()),
                      static_cast<int>(master.getSizeInCellsY()));
    const auto cost = master.getCost(1, 0);
    EXPECT_GT(cost, costmap_2d::FREE_SPACE);
    EXPECT_LT(cost, costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  }

  std::remove(k_static_pgm_path);
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
  layer.markObstacles(master, wx, wy);
  double nwx = 0.0;
  double nwy = 0.0;
  master.mapToWorld(4, 5, nwx, nwy);
  layer.markObstacles(master, nwx, nwy);

  layer.updateCosts(master, 0, 0, static_cast<int>(master.getSizeInCellsX()),
                    static_cast<int>(master.getSizeInCellsY()));
  EXPECT_EQ(master.getCost(5, 5), costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(master.getCost(4, 5), costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(master.getCost(0, 0), costmap_2d::FREE_SPACE);

  // clear covers exactly the world rectangle [2.5,3.0) x [2.5,3.0) over cell (5,5)
  layer.clearArea(2.5, 2.5, 3.0, 3.0);
  // Cleared-cell restoration is the pipeline's job (resetMap wipes the
  // window; a StaticLayer beneath rewrites): mirror it with an explicit
  // resetMap before the rewrite.
  master.resetMap(0, 0, master.getSizeInCellsX(), master.getSizeInCellsY());
  layer.updateCosts(master, 0, 0, static_cast<int>(master.getSizeInCellsX()),
                    static_cast<int>(master.getSizeInCellsY()));
  EXPECT_EQ(master.getCost(5, 5), costmap_2d::FREE_SPACE);
  EXPECT_EQ(master.getCost(4, 5), costmap_2d::LETHAL_OBSTACLE);
}

TEST(CostmapLayersTest, obstacleDecayDropsStaleMarks) {
  constexpr double res = 0.5;
  auto lm = std::make_shared<costmap_2d::LayeredCostmap>(
      "map", /*rolling_window=*/false, /*track_unknown=*/false);
  lm->resizeMap(20, 20, res, 0.0, 0.0);
  auto layer = std::make_shared<costmap_2d_layers::SimpleObstacleLayer>();
  layer->setDecayTime(0.05);          // 50 ms
  layer->setDecaySweepInterval(0.01);  // sweep every 10 ms
  lm->addLayer(layer);

  double wx = 0.0;
  double wy = 0.0;
  lm->getCostmap()->mapToWorld(5, 5, wx, wy);
  layer->markObstacles(*lm->getCostmap(), wx, wy);
  lm->updateMap(0.0, 0.0, 0.0);
  EXPECT_EQ(lm->getCostmap()->getCost(5, 5), costmap_2d::LETHAL_OBSTACLE);

  // Older than decay_time: the throttled sweep in updateBounds drops the
  // mark and the window rewrite restores the default cost.
  std::this_thread::sleep_for(std::chrono::milliseconds(80));
  lm->updateMap(0.0, 0.0, 0.0);
  EXPECT_EQ(lm->getCostmap()->getCost(5, 5), costmap_2d::FREE_SPACE);
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
  // inscribed 0.3, csf 3.0 (the nav2-deployment value; a gentle exponential
  // gradient rather than a step). Assert the cost decays strictly beyond the
  // inscribed band and tapers smoothly to zero at the end (a gradient rather
  // than a single color when visualized). Values are nav2-exact: the decay
  // term is truncated like nav2's static_cast.
  constexpr double res = 0.05;
  auto master = makeGrid(60, 60, res);
  master.setCost(30, 30, costmap_2d::LETHAL_OBSTACLE);

  costmap_2d_layers::InflationLayer layer;
  layer.setInflationRadius(0.5);
  layer.setInscribedRadius(0.3);
  layer.setCostScalingFactor(3.0);

  layer.updateCosts(master, 0, 0, static_cast<int>(master.getSizeInCellsX()),
                    static_cast<int>(master.getSizeInCellsY()));

  // The inscribed band (< 0.3m) is constantly 253.
  for (int d = 1; d <= 5; ++d) {
    EXPECT_EQ(master.getCost(30 + d, 30),
              costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
        << "d=" << d;
  }
  // The first decay cell (6*0.05 vs 0.3 is float-above) truncates to 251 --
  // exactly what nav2's static_cast produces for the same inputs.
  EXPECT_EQ(master.getCost(36, 30), 251);
  // The decay band (0.3m to 0.5m) decreases strictly: 252*e^(-3*k*0.05).
  int prev = 251;
  for (int d = 7; d <= 10; ++d) {
    const int cost = master.getCost(30 + d, 30);
    EXPECT_LT(cost, prev) << "d=" << d << " cost=" << cost;
    prev = cost;
  }
  // At the radius edge (0.5m): 252*e^(-0.6) ~ 138 -- a gentle tail, not a
  // near-zero cliff.
  const int edge_cost = master.getCost(30 + 10, 30);
  EXPECT_GT(edge_cost, 100);
  EXPECT_LT(edge_cost, 160);
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
  obstacles->markObstacles(*lm->getCostmap(), 7.75, 7.75);
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
  layer.markObstacles(master, 2.5, 2.5);
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

TEST(CostmapLayersTest, inflationHaloClearsFullyAfterObstacleRemoval) {
  // P2 regression: the inflation report expands ONLY the current dirty
  // area by the radius. After an obstacle is removed, the exact window
  // (cleared cell +- inflation radius) must cover and wipe the whole halo
  // in the very next cycle -- no sticky last-union residue.
  auto layered =
      std::make_shared<costmap_2d::LayeredCostmap>("map", false, false);
  layered->resizeMap(40, 40, 0.5, 0.0, 0.0);
  auto obstacle = std::make_shared<costmap_2d_layers::SimpleObstacleLayer>();
  auto inflation = std::make_shared<costmap_2d_layers::InflationLayer>();
  layered->addLayer(obstacle);
  layered->addLayer(inflation);
  inflation->setInflationRadius(1.0);
  inflation->setInscribedRadius(0.1);
  inflation->setCostScalingFactor(10.0);

  // Cycle 1: obstacle at world (5,5) -> cell (10,10); halo present.
  obstacle->markObstacles(*layered->getCostmap(), 5.0, 5.0);
  layered->updateMap(0.0, 0.0, 0.0);
  auto* master = layered->getCostmap();
  EXPECT_EQ(master->getCost(10, 10), costmap_2d::LETHAL_OBSTACLE);
  EXPECT_GT(master->getCost(11, 10), 0);  // 1 cell away: inside the halo

  // Cycle 2: obstacle removed -> the entire halo (all cells within the
  // inflation radius) returns to FREE in one cycle.
  obstacle->clearArea(4.9, 4.9, 5.1, 5.1);
  layered->updateMap(0.0, 0.0, 0.0);
  EXPECT_EQ(master->getCost(10, 10), costmap_2d::FREE_SPACE);
  for (unsigned y = 0; y < master->getSizeInCellsY(); ++y) {
    for (unsigned x = 0; x < master->getSizeInCellsX(); ++x) {
      const double dx = (static_cast<double>(x) + 0.5) * 0.5 - 5.0;
      const double dy = (static_cast<double>(y) + 0.5) * 0.5 - 5.0;
      if (std::hypot(dx, dy) <= 1.0) {
        EXPECT_EQ(master->getCost(x, y), costmap_2d::FREE_SPACE)
            << "stale halo at cell (" << x << "," << y << ")";
      }
    }
  }
}

TEST(CostmapLayersTest, inflationEpochSurvivesConsecutiveCycles) {
  // P3 regression: the epoch-based visited set must not leak visitation
  // flags between cycles -- rebuilding with a different source set (A
  // removed, B added elsewhere) yields independent results.
  auto layered =
      std::make_shared<costmap_2d::LayeredCostmap>("map", false, false);
  layered->resizeMap(40, 40, 0.5, 0.0, 0.0);
  auto obstacle = std::make_shared<costmap_2d_layers::SimpleObstacleLayer>();
  auto inflation = std::make_shared<costmap_2d_layers::InflationLayer>();
  layered->addLayer(obstacle);
  layered->addLayer(inflation);
  inflation->setInflationRadius(1.0);
  inflation->setInscribedRadius(0.1);
  inflation->setCostScalingFactor(10.0);

  obstacle->markObstacles(*layered->getCostmap(), 5.0, 5.0);   // cell (10,10)
  layered->updateMap(0.0, 0.0, 0.0);
  EXPECT_EQ(layered->getCostmap()->getCost(10, 10),
            costmap_2d::LETHAL_OBSTACLE);
  EXPECT_GT(layered->getCostmap()->getCost(11, 10), 0);

  obstacle->clearArea(4.9, 4.9, 5.1, 5.1);
  obstacle->markObstacles(*layered->getCostmap(), 15.0, 15.0);  // cell (30,30)
  layered->updateMap(0.0, 0.0, 0.0);
  EXPECT_EQ(layered->getCostmap()->getCost(30, 30),
            costmap_2d::LETHAL_OBSTACLE);
  EXPECT_GT(layered->getCostmap()->getCost(31, 30), 0);
  // A's halo is fully gone (and A's cell with it).
  EXPECT_EQ(layered->getCostmap()->getCost(10, 10), costmap_2d::FREE_SPACE);
  EXPECT_EQ(layered->getCostmap()->getCost(11, 10), costmap_2d::FREE_SPACE);
}

TEST(CostmapLayersTest, obstacleLayerRollingShiftKeepsAlignmentAndInflatesStrip) {
  // P6/R5 regression: whole-cell rolling shift moves presence with the
  // master (same floor quantization) and the newly exposed strip enters
  // the rebuild window so inflation reaches into the fresh area.
  auto layered =
      std::make_shared<costmap_2d::LayeredCostmap>("odom", true, false);
  layered->resizeMap(20, 20, 0.5, 0.0, 0.0);
  auto obstacle = std::make_shared<costmap_2d_layers::SimpleObstacleLayer>();
  auto inflation = std::make_shared<costmap_2d_layers::InflationLayer>();
  layered->addLayer(obstacle);
  layered->addLayer(inflation);
  inflation->setInflationRadius(1.0);
  inflation->setInscribedRadius(0.1);
  inflation->setCostScalingFactor(10.0);

  layered->updateMap(5.0, 5.0, 0.0);  // window [0,10) x [0,10)
  // Mark near the +x edge (9.5, 5) -> cell (19,10); halo at 0.75 m into
  // the strip the NEXT recenter will expose.
  obstacle->markObstacles(*layered->getCostmap(), 9.5, 5.0);
  layered->updateMap(5.0, 5.0, 0.0);
  EXPECT_EQ(layered->getCostmap()->getCost(19, 10),
            costmap_2d::LETHAL_OBSTACLE);

  // Recenter +2 m in x (robot 5 -> 7): window [2,12). The mark keeps its
  // world position (cell 19-4 = 15 for world 9.5) and the exposed strip
  // [10,12) receives the inflation band from the old-edge obstacle.
  layered->updateMap(7.0, 5.0, 0.0);
  EXPECT_EQ(layered->getCostmap()->getCost(15, 10),
            costmap_2d::LETHAL_OBSTACLE);
  EXPECT_GT(layered->getCostmap()->getCost(16, 10), 0);
  // The strip beyond the inflation radius stays free.
  EXPECT_EQ(layered->getCostmap()->getCost(19, 10), costmap_2d::FREE_SPACE);
}
