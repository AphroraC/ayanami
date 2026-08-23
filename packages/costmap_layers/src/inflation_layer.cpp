#include <costmap_2d_layers/inflation_layer.hpp>
#include <costmap_2d/layered_costmap.hpp>

#include <spdlog/spdlog.h>

namespace costmap_2d_layers {

namespace {
constexpr unsigned int kMaxInflationCells = 1000;
}  // namespace

void InflationLayer::onFootprintChanged() {
  if (master_) {
    inscribed_radius_ = master_->getInscribedRadius();
  }
  caches_dirty_ = true;
  need_reinflation_ = true;
  current_ = false;
}

void InflationLayer::updateBounds(double, double, double, double* min_x,
                                  double* min_y, double* max_x,
                                  double* max_y) {
  if (need_reinflation_) {
    *min_x = std::numeric_limits<double>::lowest();
    *min_y = std::numeric_limits<double>::lowest();
    *max_x = std::numeric_limits<double>::max();
    *max_y = std::numeric_limits<double>::max();
    need_reinflation_ = false;
    has_last_bounds_ = false;
    return;
  }

  if (!has_last_bounds_) {
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;
    *min_x -= inflation_radius_;
    *min_y -= inflation_radius_;
    *max_x += inflation_radius_;
    *max_y += inflation_radius_;
    has_last_bounds_ = true;
    return;
  }

  const double previous_min_x = last_min_x_;
  const double previous_min_y = last_min_y_;
  const double previous_max_x = last_max_x_;
  const double previous_max_y = last_max_y_;
  last_min_x_ = *min_x;
  last_min_y_ = *min_y;
  last_max_x_ = *max_x;
  last_max_y_ = *max_y;
  *min_x = std::min(previous_min_x, *min_x) - inflation_radius_;
  *min_y = std::min(previous_min_y, *min_y) - inflation_radius_;
  *max_x = std::max(previous_max_x, *max_x) + inflation_radius_;
  *max_y = std::max(previous_max_y, *max_y) + inflation_radius_;
}

void InflationLayer::ensureCaches(double resolution) {
  const unsigned int radius = static_cast<unsigned int>(
      std::ceil(inflation_radius_ / std::max(resolution, 1e-9)));
  if (!caches_dirty_ && radius == cell_inflation_radius_) {
    return;
  }

  if (radius > kMaxInflationCells) {
    spdlog::warn(
        "InflationLayer radius too large: inflation_radius={} resolution={} "
        "cells={}, clamp to {}",
        inflation_radius_, resolution, radius, kMaxInflationCells);
  }
  cell_inflation_radius_ = std::min(radius, kMaxInflationCells);

  const unsigned int cache_dim = cell_inflation_radius_ + 2;
  cached_distances_.assign(cache_dim, std::vector<double>(cache_dim, 0.0));
  cached_costs_.assign(cache_dim,
                       std::vector<costmap_2d::CostValue>(cache_dim, 0));

  for (unsigned int dx = 0; dx < cache_dim; ++dx) {
    for (unsigned int dy = 0; dy < cache_dim; ++dy) {
      const double distance = std::hypot(static_cast<double>(dx),
                                         static_cast<double>(dy));
      cached_distances_[dx][dy] = distance;
      cached_costs_[dx][dy] = computeCost(distance, resolution);
    }
  }
  caches_dirty_ = false;
}

costmap_2d::CostValue InflationLayer::costLookup(
    unsigned int mx, unsigned int my, unsigned int src_x, unsigned int src_y,
    double resolution) const {
  const unsigned int dx = static_cast<unsigned int>(
      std::abs(static_cast<int>(mx) - static_cast<int>(src_x)));
  const unsigned int dy = static_cast<unsigned int>(
      std::abs(static_cast<int>(my) - static_cast<int>(src_y)));
  if (dx <= cell_inflation_radius_ && dy <= cell_inflation_radius_ &&
      dx < cached_costs_.size() && dy < cached_costs_[dx].size()) {
    return cached_costs_[dx][dy];
  }
  return computeCost(std::hypot(static_cast<double>(dx),
                                static_cast<double>(dy)),
                     resolution);
}

void InflationLayer::enqueue(
    const costmap_2d::Costmap2D& grid,
    std::priority_queue<QueueCell, std::vector<QueueCell>, std::greater<>>&
        queue,
    std::vector<bool>& seen, unsigned int mx, unsigned int my,
    unsigned int src_x, unsigned int src_y) const {
  if (!grid.inBounds(mx, my)) {
    return;
  }
  const auto index = grid.getIndex(mx, my);
  if (seen[index]) {
    return;
  }
  const unsigned int dx = static_cast<unsigned int>(
      std::abs(static_cast<int>(mx) - static_cast<int>(src_x)));
  const unsigned int dy = static_cast<unsigned int>(
      std::abs(static_cast<int>(my) - static_cast<int>(src_y)));
  if (dx > cell_inflation_radius_ || dy > cell_inflation_radius_) {
    return;
  }
  const double distance = cached_distances_[dx][dy];
  if (distance > cell_inflation_radius_) {
    return;
  }
  seen[index] = true;
  queue.push(QueueCell{distance, mx, my, src_x, src_y});
}

void InflationLayer::updateCosts(costmap_2d::Costmap2D& master_grid,
                                 int min_i, int min_j, int max_i, int max_j) {
  if (!enabled_ || inflation_radius_ <= 0.0) {
    current_ = true;
    return;
  }

  ensureCaches(master_grid.getResolution());

  const int size_x = static_cast<int>(master_grid.getSizeInCellsX());
  const int size_y = static_cast<int>(master_grid.getSizeInCellsY());
  if (cell_inflation_radius_ * 2 >
      static_cast<unsigned int>(std::max(size_x, size_y))) {
    spdlog::warn(
        "InflationLayer radius exceeds map dimensions: cell_radius={} "
        "map_dim={} (inflation_radius={})",
        cell_inflation_radius_, std::max(size_x, size_y), inflation_radius_);
  }

  const int base_min_i = min_i;
  const int base_min_j = min_j;
  const int base_max_i = max_i;
  const int base_max_j = max_j;
  const int start_x =
      std::max(0, min_i - static_cast<int>(cell_inflation_radius_));
  const int start_y =
      std::max(0, min_j - static_cast<int>(cell_inflation_radius_));
  const int end_x =
      std::min(size_x, max_i + static_cast<int>(cell_inflation_radius_));
  const int end_y =
      std::min(size_y, max_j + static_cast<int>(cell_inflation_radius_));

  std::priority_queue<QueueCell, std::vector<QueueCell>, std::greater<>>
      queue;
  const std::size_t map_size =
      static_cast<std::size_t>(size_x) * static_cast<std::size_t>(size_y);
  if (seen_.size() != map_size) {
    seen_.assign(map_size, false);
  } else {
    // Propagation can leave the source-search window by one inflation radius;
    // Nav2 clears the complete seen array to prevent stale visitation flags.
    std::fill(seen_.begin(), seen_.end(), false);
  }

  // 以 lethal 障碍 cell 为波前源
  for (int y = start_y; y < end_y; ++y) {
    for (int x = start_x; x < end_x; ++x) {
      const auto index = master_grid.getIndex(static_cast<unsigned int>(x),
                                              static_cast<unsigned int>(y));
      if (master_grid.getCost(index) == costmap_2d::LETHAL_OBSTACLE) {
        seen_[index] = true;
        queue.push(QueueCell{0.0, static_cast<unsigned int>(x),
                             static_cast<unsigned int>(y),
                             static_cast<unsigned int>(x),
                             static_cast<unsigned int>(y)});
      }
    }
  }

  while (!queue.empty()) {
    const auto cell = queue.top();
    queue.pop();

    if (static_cast<int>(cell.x) >= base_min_i &&
        static_cast<int>(cell.y) >= base_min_j &&
        static_cast<int>(cell.x) < base_max_i &&
        static_cast<int>(cell.y) < base_max_j) {
      const auto cost = costLookup(cell.x, cell.y, cell.src_x, cell.src_y,
                                   master_grid.getResolution());
      const auto old_cost = master_grid.getCost(cell.x, cell.y);
      if (old_cost == costmap_2d::NO_INFORMATION) {
        if (cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
          master_grid.setCost(cell.x, cell.y, cost);
        }
      } else if (cost > old_cost) {
        master_grid.setCost(cell.x, cell.y, cost);
      }
    }

    enqueue(master_grid, queue, seen_, cell.x - 1, cell.y, cell.src_x,
            cell.src_y);
    enqueue(master_grid, queue, seen_, cell.x + 1, cell.y, cell.src_x,
            cell.src_y);
    enqueue(master_grid, queue, seen_, cell.x, cell.y - 1, cell.src_x,
            cell.src_y);
    enqueue(master_grid, queue, seen_, cell.x, cell.y + 1, cell.src_x,
            cell.src_y);
  }
  current_ = true;
}

}  // namespace costmap_2d_layers
