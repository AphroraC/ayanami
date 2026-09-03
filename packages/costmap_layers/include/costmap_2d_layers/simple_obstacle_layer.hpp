#pragma once

#include <algorithm>
#include <vector>

#include <costmap_2d/costmap_2d.hpp>
#include <costmap_2d/layer.hpp>

namespace costmap_2d_layers {

// 简单障碍层 (简洁版): 接受 markObstacles / clearArea API 写入障碍与清除区,
// 在 updateCosts 阶段把标记写回 master grid。不直接加载传感器 (observation_sources
// 留给上层把观测转成 markObstacles 调用)。
class SimpleObstacleLayer : public costmap_2d::Layer {
 public:
  enum class CombinationMethod { OVERWRITE = 0, MAXIMUM = 1, NONE = 2 };

  void setCombinationMethod(CombinationMethod method) {
    combination_method = method;
  }

  // Global maps restore the underlying StaticLayer before applying dynamic
  // marks, so they must not overwrite that restored value with FREE.
  void setClearToFree(bool clear_to_free) { clear_to_free = clear_to_free; }

  // 在世界坐标 (wx, wy) 标记一个障碍; 在 updateCosts 阶段写入 master grid。
  void markObstacles(double wx, double wy,
                     costmap_2d::CostValue cost = costmap_2d::LETHAL_OBSTACLE) {
    marks.push_back(Mark{wx, wy, cost});
    current = false;
  }

  // 清除世界矩形 [min_wx, max_wx] x [min_wy, max_wy] 覆盖的区域:
  // 移除该区域内的障碍标记, 并在 updateCosts 阶段把区域内 cell 写回 FREE。
  void clearArea(double min_wx, double min_wy, double max_wx, double max_wy);

  // Nav2-style freespace clearing: remove only dynamic marks intersected by
  // the clipped Bresenham ray and add its bounds to the dirty region.
  void raytraceFreespace(const costmap_2d::Costmap2D& grid, double origin_x,
                         double origin_y, double end_x, double end_y);

  void updateBounds(double robot_x, double robot_y, double robot_yaw,
                    double* min_x, double* min_y, double* max_x,
                    double* max_y) override;

  void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j,
                   int max_i, int max_j) override;

  void activate() override { current = true; }
  void deactivate() override {}
  void reset() override {
    marks.clear();
    clear_regions.clear();
    current = false;
  }
  void matchSize() override {}

 private:
  struct Mark {
    double wx{0.0};
    double wy{0.0};
    costmap_2d::CostValue cost{costmap_2d::LETHAL_OBSTACLE};
  };
  struct ClearRegion {
    double min_x{0.0};
    double min_y{0.0};
    double max_x{0.0};
    double max_y{0.0};
  };

  void applyCost(costmap_2d::Costmap2D& grid, unsigned int mx, unsigned int my,
                 costmap_2d::CostValue cost) const;
  static void touch(double x, double y, double* min_x, double* min_y,
                    double* max_x, double* max_y);

  CombinationMethod combination_method{CombinationMethod::MAXIMUM};
  bool clear_to_free{true};
  std::vector<Mark> marks;
  std::vector<ClearRegion> clear_regions;
};

}  // namespace costmap_2d_layers
