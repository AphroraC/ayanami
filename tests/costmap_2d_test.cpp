#include <gtest/gtest.h>

#include <costmap_2d/cost_values.hpp>
#include <costmap_2d/costmap_2d.hpp>
#include <costmap_2d/layer.hpp>
#include <costmap_2d/layered_costmap.hpp>

#include <memory>

namespace {

// 自定义测试层：记录 updateBounds/updateCosts 调用，并在主 grid 上写一个 lethal cell。
class TestLayer : public costmap_2d::Layer {
 public:
  int bounds_calls{0};
  int costs_calls{0};

  void updateBounds(double, double, double, double* min_x, double* min_y,
                    double* max_x, double* max_y) override {
    ++bounds_calls;
    *min_x = 0.0;
    *min_y = 0.0;
    *max_x = 1.0;
    *max_y = 1.0;
  }

  void updateCosts(costmap_2d::Costmap2D& master_grid, int, int, int,
                   int) override {
    ++costs_calls;
    master_grid.setCost(2, 3, costmap_2d::LETHAL_OBSTACLE);
  }
};

class ToggleLayer : public costmap_2d::Layer {
 public:
  bool mark{true};
  void updateBounds(double, double, double, double* min_x, double* min_y,
                    double* max_x, double* max_y) override {
    *min_x = 1.0;
    *min_y = 1.0;
    *max_x = 2.0;
    *max_y = 2.0;
  }
  void updateCosts(costmap_2d::Costmap2D& master, int, int, int, int) override {
    if (mark) master.setCost(1, 1, costmap_2d::LETHAL_OBSTACLE);
  }
};

}  // namespace

TEST(Costmap2DTest, resizeMapPopulatesMembers) {
  costmap_2d::Costmap2D map;
  map.resizeMap(10, 20, 0.05, 0, 0);
  EXPECT_EQ(map.getSizeInCellsX(), 10u);
  EXPECT_EQ(map.getSizeInCellsY(), 20u);
  EXPECT_DOUBLE_EQ(map.getResolution(), 0.05);
  EXPECT_NE(map.getCharMap(), nullptr);
}

TEST(Costmap2DTest, resizeMapOverwritesPreviousMembers) {
  // Ruling #6: resizeMap 参数不得遮蔽成员（size_x = size_x 自赋值 bug）
  costmap_2d::Costmap2D map;
  map.resizeMap(10, 20, 0.05, 0, 0);
  map.resizeMap(30, 40, 0.1, 1.0, 2.0);
  EXPECT_EQ(map.getSizeInCellsX(), 30u);
  EXPECT_EQ(map.getSizeInCellsY(), 40u);
  EXPECT_DOUBLE_EQ(map.getResolution(), 0.1);
  EXPECT_DOUBLE_EQ(map.getOriginX(), 1.0);
  EXPECT_DOUBLE_EQ(map.getOriginY(), 2.0);
}

TEST(Costmap2DTest, setAndGetCost) {
  costmap_2d::Costmap2D map;
  map.resizeMap(10, 20, 0.05, 0, 0);
  map.setCost(5, 10, 200);
  EXPECT_EQ(map.getCost(5, 10), 200);
  EXPECT_EQ(map.getCost(0, 0), costmap_2d::FREE_SPACE);
}

TEST(Costmap2DTest, worldToMapMapToWorldRoundTrip) {
  costmap_2d::Costmap2D map;
  map.resizeMap(10, 20, 0.05, 1.0, 2.0);
  unsigned int mx = 0;
  unsigned int my = 0;
  double wx = 0.0;
  double wy = 0.0;
  map.mapToWorld(3, 7, wx, wy);
  EXPECT_TRUE(map.worldToMap(wx, wy, mx, my));
  EXPECT_EQ(mx, 3u);
  EXPECT_EQ(my, 7u);
}

TEST(Costmap2DTest, layeredCostmapInvokesLayer) {
  costmap_2d::LayeredCostmap lcm("map", false, false);
  lcm.resizeMap(10, 20, 0.05, 0, 0);
  auto layer = std::make_shared<TestLayer>();
  lcm.addLayer(layer);
  lcm.updateMap(0.0, 0.0, 0.0);
  EXPECT_GT(layer->bounds_calls, 0);
  EXPECT_GT(layer->costs_calls, 0);
  EXPECT_EQ(lcm.getCostmap()->getCost(2, 3), costmap_2d::LETHAL_OBSTACLE);
}

TEST(Costmap2DTest, layeredCostmapRebuildsDirtyMasterWindow) {
  costmap_2d::LayeredCostmap lcm("map", false, false);
  lcm.resizeMap(10, 10, 1.0, 0.0, 0.0);
  auto layer = std::make_shared<ToggleLayer>();
  lcm.addLayer(layer);
  lcm.updateMap(0.0, 0.0, 0.0);
  ASSERT_EQ(lcm.getCostmap()->getCost(1, 1),
            costmap_2d::LETHAL_OBSTACLE);
  layer->mark = false;
  lcm.updateMap(0.0, 0.0, 0.0);
  EXPECT_EQ(lcm.getCostmap()->getCost(1, 1), costmap_2d::FREE_SPACE);
}

TEST(Costmap2DTest, rollingOriginStaysCellAligned) {
  costmap_2d::Costmap2D map(10, 10, 0.5, 0.0, 0.0);
  map.updateOrigin(0.24, 0.24);
  EXPECT_DOUBLE_EQ(map.getOriginX(), 0.0);
  EXPECT_DOUBLE_EQ(map.getOriginY(), 0.0);
  map.updateOrigin(0.76, -0.26);
  EXPECT_DOUBLE_EQ(map.getOriginX(), 0.5);
  EXPECT_DOUBLE_EQ(map.getOriginY(), -0.5);
}

TEST(Costmap2DTest, footprintComputesNav2Radii) {
  costmap_2d::LayeredCostmap lcm("map", false, false);
  lcm.setFootprint({{-0.2, -0.1, 0.0}, {-0.2, 0.1, 0.0},
                    {0.2, 0.1, 0.0}, {0.2, -0.1, 0.0}});
  EXPECT_NEAR(lcm.getInscribedRadius(), 0.1, 1e-9);
  EXPECT_NEAR(lcm.getCircumscribedRadius(), std::hypot(0.2, 0.1), 1e-9);
}

TEST(Costmap2DTest, raytraceLineUsesBresenhamCells) {
  costmap_2d::Costmap2D map(10, 10, 1.0, 0.0, 0.0);
  std::vector<unsigned int> cells;
  map.raytraceLine([&](unsigned int index) { cells.push_back(index); },
                   1, 1, 5, 3);
  ASSERT_FALSE(cells.empty());
  EXPECT_EQ(cells.front(), map.getIndex(1, 1));
  EXPECT_EQ(cells.back(), map.getIndex(5, 3));
  EXPECT_EQ(cells.size(), 5u);
}

TEST(Costmap2DTest, costValuesMatchSpec) {
  EXPECT_EQ(costmap_2d::FREE_SPACE, 0);
  EXPECT_EQ(costmap_2d::INSCRIBED_INFLATED_OBSTACLE, 253);
  EXPECT_EQ(costmap_2d::LETHAL_OBSTACLE, 254);
  EXPECT_EQ(costmap_2d::NO_INFORMATION, 255);
}

TEST(Costmap2DTest, isLethalIncludes253) {
  // Ruling #6: isLethal 必须把 253 (inscribed inflated) 判为 lethal,
  // 否则膨胀带被误判为非致命。
  EXPECT_TRUE(costmap_2d::isLethal(253));
  EXPECT_TRUE(costmap_2d::isLethal(254));
  EXPECT_FALSE(costmap_2d::isLethal(100));
  EXPECT_FALSE(costmap_2d::isLethal(0));
}
