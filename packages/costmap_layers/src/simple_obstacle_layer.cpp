#include <costmap_2d_layers/simple_obstacle_layer.hpp>
#include <unordered_set>

namespace costmap_2d_layers {

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
  current_ = false;
}

void SimpleObstacleLayer::raytraceFreespace(
    const costmap_2d::Costmap2D& grid, double origin_x, double origin_y,
    double end_x, double end_y) {
  unsigned int origin_mx = 0;
  unsigned int origin_my = 0;
  if (!grid.worldToMap(origin_x, origin_y, origin_mx, origin_my)) return;

  const double dx = end_x - origin_x;
  const double dy = end_y - origin_y;
  const double map_min_x = grid.getOriginX();
  const double map_min_y = grid.getOriginY();
  const double epsilon = std::max(grid.getResolution() * 1e-6, 1e-9);
  const double map_max_x =
      map_min_x + grid.getSizeInMetersX() - epsilon;
  const double map_max_y =
      map_min_y + grid.getSizeInMetersY() - epsilon;
  double scale = 1.0;
  if (dx > 0.0) scale = std::min(scale, (map_max_x - origin_x) / dx);
  if (dx < 0.0) scale = std::min(scale, (map_min_x - origin_x) / dx);
  if (dy > 0.0) scale = std::min(scale, (map_max_y - origin_y) / dy);
  if (dy < 0.0) scale = std::min(scale, (map_min_y - origin_y) / dy);
  scale = std::clamp(scale, 0.0, 1.0);
  end_x = origin_x + dx * scale;
  end_y = origin_y + dy * scale;

  unsigned int end_mx = 0;
  unsigned int end_my = 0;
  if (!grid.worldToMap(end_x, end_y, end_mx, end_my)) return;
  std::unordered_set<unsigned int> ray_cells;
  grid.raytraceLine(
      [&](unsigned int index) { ray_cells.insert(index); }, origin_mx,
      origin_my, end_mx, end_my);
  marks_.erase(
      std::remove_if(marks_.begin(), marks_.end(), [&](const Mark& mark) {
        unsigned int mx = 0;
        unsigned int my = 0;
        return grid.worldToMap(mark.wx, mark.wy, mx, my) &&
               ray_cells.count(grid.getIndex(mx, my)) != 0;
      }),
      marks_.end());
  clear_regions_.push_back(ClearRegion{
      std::min(origin_x, end_x), std::min(origin_y, end_y),
      std::max(origin_x, end_x), std::max(origin_y, end_y)});
  current_ = false;
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
      if (grid.getCost(mx, my) == costmap_2d::NO_INFORMATION) {
        grid.setCost(mx, my, cost);
      } else if (cost != costmap_2d::NO_INFORMATION) {
        grid.setCost(mx, my, std::max(grid.getCost(mx, my), cost));
      }
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

  // Collapse repeated observations of the same map cell. This keeps layer
  // state bounded by map area rather than sensor message count.
  std::vector<Mark> unique_marks;
  unique_marks.reserve(marks_.size());
  std::unordered_set<unsigned int> occupied;
  occupied.reserve(marks_.size());
  for (auto iter = marks_.rbegin(); iter != marks_.rend(); ++iter) {
    unsigned int mx = 0;
    unsigned int my = 0;
    if (!master_grid.worldToMap(iter->wx, iter->wy, mx, my)) continue;
    const auto index = master_grid.getIndex(mx, my);
    if (occupied.insert(index).second) {
      unique_marks.push_back(*iter);
    }
  }
  std::reverse(unique_marks.begin(), unique_marks.end());
  marks_.swap(unique_marks);

  // A global map can disable FREE writes because an earlier StaticLayer has
  // already restored the underlying cost. The default preserves the layer's
  // standalone/local-map clear contract.
  if (clear_to_free_ && master_ == nullptr) {
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
  clear_regions_.clear();
  current_ = true;
}

}  // namespace costmap_2d_layers
