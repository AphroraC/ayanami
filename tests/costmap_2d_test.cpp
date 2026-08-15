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

}  // namespace

TEST(Costmap2DTest, ResizeMapPopulatesMembers) {
  costmap_2d::Costmap2D map;
  map.resizeMap(10, 20, 0.05, 0, 0);
  EXPECT_EQ(map.getSizeInCellsX(), 10u);
  EXPECT_EQ(map.getSizeInCellsY(), 20u);
  EXPECT_DOUBLE_EQ(map.getResolution(), 0.05);
  EXPECT_NE(map.getCharMap(), nullptr);
}

TEST(Costmap2DTest, ResizeMapOverwritesPreviousMembers) {
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

TEST(Costmap2DTest, SetAndGetCost) {
  costmap_2d::Costmap2D map;
  map.resizeMap(10, 20, 0.05, 0, 0);
  map.setCost(5, 10, 200);
  EXPECT_EQ(map.getCost(5, 10), 200);
  EXPECT_EQ(map.getCost(0, 0), costmap_2d::FREE_SPACE);
}

TEST(Costmap2DTest, WorldToMapMapToWorldRoundTrip) {
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

TEST(Costmap2DTest, LayeredCostmapInvokesLayer) {
  costmap_2d::LayeredCostmap lcm("map", false, false);
  lcm.resizeMap(10, 20, 0.05, 0, 0);
  auto layer = std::make_shared<TestLayer>();
  lcm.addLayer(layer);
  lcm.updateMap(0.0, 0.0, 0.0);
  EXPECT_GT(layer->bounds_calls, 0);
  EXPECT_GT(layer->costs_calls, 0);
  EXPECT_EQ(lcm.getCostmap()->getCost(2, 3), costmap_2d::LETHAL_OBSTACLE);
}

TEST(Costmap2DTest, CostValuesMatchSpec) {
  EXPECT_EQ(costmap_2d::FREE_SPACE, 0);
  EXPECT_EQ(costmap_2d::INSCRIBED_INFLATED_OBSTACLE, 253);
  EXPECT_EQ(costmap_2d::LETHAL_OBSTACLE, 254);
  EXPECT_EQ(costmap_2d::NO_INFORMATION, 255);
}
