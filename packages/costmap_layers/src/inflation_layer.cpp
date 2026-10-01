#include <costmap_2d_layers/inflation_layer.hpp>
#include <costmap_2d/layered_costmap.hpp>

#include <memory>
#include <spdlog/spdlog.h>

namespace costmap_2d_layers {

namespace {
constexpr unsigned int k_max_inflation_cells = 1000;
}  // namespace

void InflationLayer::onFootprintChanged() {
  if (master) {
    inscribed_radius = master->getInscribedRadius();
    if (inflation_radius < inscribed_radius) {
      // nav2-parity diagnostic: an inflation radius below the footprint's
      // inscribed radius leaves no exponential band at all -- the layer
      // degenerates to a binary obstacle map in rviz and in planning.
      spdlog::error(
          "InflationLayer: the configured inflation radius ({:.3f}) is "
          "smaller than the computed inscribed radius ({:.3f}) of your "
          "footprint; it is highly recommended to set the inflation radius "
          "to be at least as big as the inscribed radius to avoid collisions",
          inflation_radius, inscribed_radius);
    }
  }
  caches_dirty = true;
  need_reinflation = true;
  data_current = false;
}

void InflationLayer::updateBounds(double, double, double, double* min_x,
                                  double* min_y, double* max_x,
                                  double* max_y) {
  if (need_reinflation) {
    *min_x = std::numeric_limits<double>::lowest();
    *min_y = std::numeric_limits<double>::lowest();
    *max_x = std::numeric_limits<double>::max();
    *max_y = std::numeric_limits<double>::max();
    need_reinflation = false;
    return;
  }

  // Exact dirty-window reporting: expand ONLY this cycle's incoming change
  // area (the union of the layers beneath) by the inflation radius. The old
  // last-cycle union made the window sticky: one extra full reprocess of the
  // previous window ran after every change, and with frequent small marks
  // the rebuilt area never shrank back. Correctness: every source change is
  // in exactly one cycle's incoming bounds; the halo around it is covered by
  // this expansion, and resetMap wipes the window before inflation repaints.
  // Empty incoming bounds (no changes anywhere) therefore produce an empty
  // window and the whole updateMap cycle early-outs.
  const double lo_x = *min_x - inflation_radius;
  const double lo_y = *min_y - inflation_radius;
  const double hi_x = *max_x + inflation_radius;
  const double hi_y = *max_y + inflation_radius;
  touch(lo_x, lo_y, min_x, min_y, max_x, max_y);
  touch(hi_x, hi_y, min_x, min_y, max_x, max_y);
}

void InflationLayer::ensureCaches(double resolution) {
  const unsigned int radius = static_cast<unsigned int>(
      std::ceil(inflation_radius / std::max(resolution, 1e-9)));
  if (!caches_dirty && radius == cell_inflation_radius) {
    return;
  }

  if (radius > k_max_inflation_cells) {
    // Route via the host's "costmap" layer logger when registered;
    // this sub-library depends on spdlog only.
    std::shared_ptr<spdlog::logger> costmap_logger = spdlog::get("costmap");
    if (!costmap_logger) {
      costmap_logger = spdlog::default_logger();
    }
    costmap_logger->warn(
      "InflationLayer radius too large: inflation_radius={} resolution={} "
      "cells={}, clamp to {}",
      inflation_radius,
      resolution,
      radius,
      k_max_inflation_cells);
  }
  cell_inflation_radius = std::min(radius, k_max_inflation_cells);

  const unsigned int cache_length = cell_inflation_radius + 2;
  cached_distances.assign(
      static_cast<std::size_t>(cache_length) * cache_length, 0.0);
  cached_costs.assign(static_cast<std::size_t>(cache_length) * cache_length,
                      0);

  for (unsigned int dx = 0; dx < cache_length; ++dx) {
    for (unsigned int dy = 0; dy < cache_length; ++dy) {
      const double distance = std::hypot(static_cast<double>(dx),
                                         static_cast<double>(dy));
      const std::size_t index =
          static_cast<std::size_t>(dx) * cache_length + dy;
      cached_distances[index] = distance;
      cached_costs[index] = computeCost(distance, resolution);
    }
  }
  caches_dirty = false;
}

costmap_2d::CostValue InflationLayer::costLookup(
    unsigned int mx, unsigned int my, unsigned int src_x, unsigned int src_y,
    double resolution) const {
  const unsigned int dx = static_cast<unsigned int>(
      std::abs(static_cast<int>(mx) - static_cast<int>(src_x)));
  const unsigned int dy = static_cast<unsigned int>(
      std::abs(static_cast<int>(my) - static_cast<int>(src_y)));
  const unsigned int cache_length = cell_inflation_radius + 2;
  if (dx < cache_length && dy < cache_length) {
    return cached_costs[static_cast<std::size_t>(dx) * cache_length + dy];
  }
  return computeCost(std::hypot(static_cast<double>(dx),
                                static_cast<double>(dy)),
                     resolution);
}

void InflationLayer::enqueue(
    const costmap_2d::Costmap2D& grid,
    std::vector<std::vector<QueueCell>>& buckets, unsigned int mx,
    unsigned int my, unsigned int src_x, unsigned int src_y) {
  if (!grid.inBounds(mx, my)) {
    return;
  }
  const unsigned int dx = static_cast<unsigned int>(
      std::abs(static_cast<int>(mx) - static_cast<int>(src_x)));
  const unsigned int dy = static_cast<unsigned int>(
      std::abs(static_cast<int>(my) - static_cast<int>(src_y)));
  if (dx > cell_inflation_radius || dy > cell_inflation_radius) {
    return;
  }
  const unsigned int cache_length = cell_inflation_radius + 2;
  const double distance =
      cached_distances[static_cast<std::size_t>(dx) * cache_length + dy];
  if (distance > static_cast<double>(cell_inflation_radius)) {
    return;
  }
  const auto index = grid.getIndex(mx, my);
  if (seen_epoch[index] == epoch) {
    return;
  }
  seen_epoch[index] = epoch;
  // Squared integer distance is the bucket key: within the radius disc,
  // dx*dx+dy*dy <= r*r, so the bucket count r*r+1 suffices.
  buckets[static_cast<std::size_t>(dx) * dx + static_cast<std::size_t>(dy) * dy]
      .push_back(QueueCell{mx, my, src_x, src_y});
}

void InflationLayer::updateCosts(costmap_2d::Costmap2D& master_grid,
                                 int min_i, int min_j, int max_i, int max_j) {
  if (!enabled || inflation_radius <= 0.0) {
    data_current = true;
    return;
  }

  ensureCaches(master_grid.getResolution());

  const int size_x = static_cast<int>(master_grid.getSizeInCellsX());
  const int size_y = static_cast<int>(master_grid.getSizeInCellsY());
  if (cell_inflation_radius * 2 >
      static_cast<unsigned int>(std::max(size_x, size_y))) {
    // Route via the host's "costmap" layer logger when registered;
    // this sub-library depends on spdlog only.
    std::shared_ptr<spdlog::logger> costmap_logger = spdlog::get("costmap");
    if (!costmap_logger) {
      costmap_logger = spdlog::default_logger();
    }
    costmap_logger->warn(
      "InflationLayer radius exceeds map dimensions: cell_radius={} "
      "map_dim={} (inflation_radius={})",
      cell_inflation_radius,
      std::max(size_x, size_y),
      inflation_radius);
  }

  const int base_min_i = min_i;
  const int base_min_j = min_j;
  const int base_max_i = max_i;
  const int base_max_j = max_j;
  const int start_x =
      std::max(0, min_i - static_cast<int>(cell_inflation_radius));
  const int start_y =
      std::max(0, min_j - static_cast<int>(cell_inflation_radius));
  const int end_x =
      std::min(size_x, max_i + static_cast<int>(cell_inflation_radius));
  const int end_y =
      std::min(size_y, max_j + static_cast<int>(cell_inflation_radius));

  // Epoch bump instead of a full seen-array clear; on wraparound the array
  // is refilled once so stale epochs can never alias the current cycle.
  ++epoch;
  if (epoch == 0) {
    std::fill(seen_epoch.begin(), seen_epoch.end(), 0);
    epoch = 1;
  }

  const std::size_t map_size =
      static_cast<std::size_t>(size_x) * static_cast<std::size_t>(size_y);
  if (seen_epoch.size() != map_size) {
    seen_epoch.assign(map_size, 0);
    epoch = 1;
  }

  // Bucket storage sized by the radius disc (see enqueue); capacity is kept
  // across cycles, only the per-bucket contents clear.
  const std::size_t needed_buckets =
      static_cast<std::size_t>(cell_inflation_radius) * cell_inflation_radius +
      1;
  if (buckets.size() != needed_buckets) {
    buckets.assign(needed_buckets, {});
  } else {
    for (auto& cell_bucket : buckets) {
      cell_bucket.clear();
    }
  }

  // Use lethal obstacle cells as the wavefront sources
  for (int y = start_y; y < end_y; ++y) {
    for (int x = start_x; x < end_x; ++x) {
      const auto index = master_grid.getIndex(static_cast<unsigned int>(x),
                                              static_cast<unsigned int>(y));
      if (master_grid.getCost(index) == costmap_2d::LETHAL_OBSTACLE) {
        seen_epoch[index] = epoch;
        buckets[0].push_back(QueueCell{static_cast<unsigned int>(x),
                                       static_cast<unsigned int>(y),
                                       static_cast<unsigned int>(x),
                                       static_cast<unsigned int>(y)});
      }
    }
  }

  // Wavefront: buckets are already in increasing Euclidean distance order,
  // so no priority queue is needed. Enqueue may append to the bucket being
  // iterated (a neighbor at equal squared distance); the index loop and the
  // by-value cell copy keep that safe under reallocation.
  for (std::size_t distance_sq = 0; distance_sq < buckets.size();
       ++distance_sq) {
    auto& cells = buckets[distance_sq];
    for (std::size_t i = 0; i < cells.size(); ++i) {
      const QueueCell cell = cells[i];
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

      enqueue(master_grid, buckets, cell.x - 1, cell.y, cell.src_x,
              cell.src_y);
      enqueue(master_grid, buckets, cell.x + 1, cell.y, cell.src_x,
              cell.src_y);
      enqueue(master_grid, buckets, cell.x, cell.y - 1, cell.src_x,
              cell.src_y);
      enqueue(master_grid, buckets, cell.x, cell.y + 1, cell.src_x,
              cell.src_y);
    }
  }
  for (auto& cell_bucket : buckets) {
    cell_bucket.clear();
  }
  data_current = true;
}

}  // namespace costmap_2d_layers
