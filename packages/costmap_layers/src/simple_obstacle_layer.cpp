#include <costmap_2d_layers/simple_obstacle_layer.hpp>

namespace costmap_2d_layers {

void SimpleObstacleLayer::markObstacles(
    double wx, double wy, costmap_2d::CostValue cost) {
  marks_.push_back(Mark{wx, wy, cost});
}

void SimpleObstacleLayer::clearArea(double min_wx, double min_wy,
                                    double max_wx, double max_wy) {
  clear_regions_.emplace_back(
      ClearRegion{min_wx, min_wy, max_wx, max_wy});
  // 移除落在清除区域内的障碍标记, 避免下次 updateCosts 重新写回。
  marks_.erase(
      std::remove_if(
          marks_.begin(), marks_.end(),
          [&](const Mark& m) {
            return m.wx >= min_wx && m.wx <= max_wx && m.wy >= min_wy &&
                   m.wy <= max_wy;
          }),
      marks_.end());
}

void SimpleObstacleLayer::touch(double x, double y, double* min_x,
                                double* min_y, double* max_x,
                                double* max_y) {
  *min_x = std::min(*min_x, x);
  *min_y = std::min(*min_y, y);
  *max_x = std::max(*max_x, x);
  *max_y = std::max(*max_y, y);
}

void SimpleObstacleLayer::updateBounds(double, double, double, double* min_x,
                                       double* min_y, double* max_x,
                                       double* max_y) {
  if (!enabled_) {
    return;
  }
  for (const auto& mark : marks_) {
    touch(mark.wx, mark.wy, min_x, min_y, max_x, max_y);
  }
  for (const auto& region : clear_regions_) {
    touch(region.min_x, region.min_y, min_x, min_y, max_x, max_y);
    touch(region.max_x, region.max_y, min_x, min_y, max_x, max_y);
  }
}

void SimpleObstacleLayer::applyCost(costmap_2d::Costmap2D& grid,
                                    unsigned int mx, unsigned int my,
                                    costmap_2d::CostValue cost) const {
  switch (combination_method_) {
    case CombinationMethod::OVERWRITE:
      grid.setCost(mx, my, cost);
      break;
    case CombinationMethod::MAXIMUM:
      grid.setCost(mx, my, std::max(grid.getCost(mx, my), cost));
      break;
    case CombinationMethod::NONE:
      break;
  }
}

void SimpleObstacleLayer::updateCosts(costmap_2d::Costmap2D& master_grid,
                                      int min_i, int min_j, int max_i,
                                      int max_j) {
  if (!enabled_) {
    return;
  }

  const int size_x = static_cast<int>(master_grid.getSizeInCellsX());
  const int size_y = static_cast<int>(master_grid.getSizeInCellsY());
  const int window_x0 = std::max(0, min_i);
  const int window_y0 = std::max(0, min_j);
  const int window_x1 = std::min(size_x, max_i);
  const int window_y1 = std::min(size_y, max_j);

  // 1) 清除区: 把区域内 cell 写回 FREE
  for (const auto& region : clear_regions_) {
    int cx0 = 0;
    int cy0 = 0;
    int cx1 = 0;
    int cy1 = 0;
    master_grid.worldToMapNoBounds(region.min_x, region.min_y, cx0, cy0);
    master_grid.worldToMapNoBounds(region.max_x, region.max_y, cx1, cy1);
    cx0 = std::max(cx0, window_x0);
    cy0 = std::max(cy0, window_y0);
    cx1 = std::min(cx1, window_x1);
    cy1 = std::min(cy1, window_y1);
    for (int y = cy0; y < cy1; ++y) {
      for (int x = cx0; x < cx1; ++x) {
        master_grid.setCost(static_cast<unsigned int>(x),
                            static_cast<unsigned int>(y),
                            costmap_2d::FREE_SPACE);
      }
    }
  }

  // 2) 障碍标记: 写回 mark 点所在 cell
  for (const auto& mark : marks_) {
    unsigned int mx = 0;
    unsigned int my = 0;
    if (!master_grid.worldToMap(mark.wx, mark.wy, mx, my)) {
      continue;
    }
    const int ix = static_cast<int>(mx);
    const int iy = static_cast<int>(my);
    if (ix < window_x0 || iy < window_y0 || ix >= window_x1 ||
        iy >= window_y1) {
      continue;
    }
    applyCost(master_grid, mx, my, mark.cost);
  }
}

}  // namespace costmap_2d_layers
