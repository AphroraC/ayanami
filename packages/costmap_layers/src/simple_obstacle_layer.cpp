#include <costmap_2d_layers/simple_obstacle_layer.hpp>

#include <costmap_2d/layered_costmap.hpp>

#include <spdlog/spdlog.h>

#include <cmath>
#include <cstring>
#include <type_traits>
#include <unordered_set>

namespace costmap_2d_layers {

namespace {
constexpr int64_t k_ns_per_s = 1000000000LL;

int64_t steadyNowNs() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}
}  // namespace

void SimpleObstacleLayer::syncGeometry(const costmap_2d::Costmap2D& grid) {
  const unsigned sx = grid.getSizeInCellsX();
  const unsigned sy = grid.getSizeInCellsY();
  const double res = grid.getResolution();
  const double ox = grid.getOriginX();
  const double oy = grid.getOriginY();

  if (sx == geom_size_x && sy == geom_size_y && res == geom_res) {
    if (ox == geom_ox && oy == geom_oy) {
      return;
    }
    // Rolling recenter: shift presence by whole cells, quantized with the
    // SAME floor semantics Costmap2D::updateOrigin uses (both origins are
    // cell-aligned, so the difference is an exact multiple of the
    // resolution; floor keeps the two grids locked to one quantization).
    const int dx = static_cast<int>(std::floor((ox - geom_ox) / res));
    const int dy = static_cast<int>(std::floor((oy - geom_oy) / res));
    if (dx != 0 || dy != 0) {
      shiftPresence(dx, dy);
      // Only the newly exposed strips are dirty: shifted cells moved WITH
      // the master (same whole-cell quantization), so their content is
      // already aligned; the vacated strips (filled with free) must reach
      // the master window so inflation repaints the band from old-edge
      // obstacles into the fresh area.
      const unsigned usx = sx;
      const unsigned usy = sy;
      const unsigned shift_x = static_cast<unsigned int>(std::abs(dx));
      const unsigned shift_y = static_cast<unsigned int>(std::abs(dy));
      auto dirty_x_strip = [&](unsigned x0, unsigned x1) {
        if (x0 >= x1) return;
        touchRect(x0, 0, x1 - 1, usy - 1);
      };
      auto dirty_y_strip = [&](unsigned y0, unsigned y1) {
        if (y0 >= y1) return;
        touchRect(0, y0, usx - 1, y1 - 1);
      };
      if (shift_x >= usx || shift_y >= usy) {
        has_dirty = true;
        dirty_min_i = 0;
        dirty_min_j = 0;
        dirty_max_i = usx;
        dirty_max_j = usy;
        return;
      }
      if (dx > 0) {
        dirty_x_strip(usx - shift_x, usx);
      } else if (dx < 0) {
        dirty_x_strip(0, shift_x);
      }
      if (dy > 0) {
        dirty_y_strip(usy - shift_y, usy);
      } else if (dy < 0) {
        dirty_y_strip(0, shift_y);
      }
    }
    geom_ox = ox;
    geom_oy = oy;
    return;
  }

  // Geometry change (resize/resolution): reset occupancy.
  geom_size_x = sx;
  geom_size_y = sy;
  geom_res = res;
  geom_ox = ox;
  geom_oy = oy;
  presence_.assign(static_cast<std::size_t>(sx) * sy, 0);
  last_seen_ns_.clear();
  if (decay_time_s > 0.0) {
    last_seen_ns_.assign(static_cast<std::size_t>(sx) * sy, 0);
  }
  has_dirty = false;
}

void SimpleObstacleLayer::shiftPresence(int dx, int dy) {
  const unsigned sx = geom_size_x;
  const unsigned sy = geom_size_y;
  const unsigned shift_x = static_cast<unsigned int>(std::abs(dx));
  const unsigned shift_y = static_cast<unsigned int>(std::abs(dy));

  // In-place whole-cell shift, the same moves Costmap2D::updateOrigin makes
  // on the master grid: per-row horizontal memmove, one vertical block
  // memmove, then the vacated strips are zeroed (new edge cells are free;
  // fresh timestamps for decay). No temporary allocation, O(shift) fills.
  const auto shift_in_place = [&](auto* storage) {
    using T = std::remove_reference_t<decltype(storage[0])>;
    if (dx != 0 && shift_x < sx) {
      for (unsigned y = 0; y < sy; ++y) {
        T* row = storage + static_cast<std::size_t>(y) * sx;
        if (dx > 0) {
          std::memmove(row, row + dx, static_cast<std::size_t>(sx - dx) * sizeof(T));
        } else {
          std::memmove(row + shift_x, row,
                       static_cast<std::size_t>(sx - shift_x) * sizeof(T));
        }
      }
    }
    if (dy != 0 && shift_y < sy) {
      if (dy > 0) {
        std::memmove(storage, storage + static_cast<std::size_t>(dy) * sx,
                     static_cast<std::size_t>(sy - dy) * sx * sizeof(T));
      } else {
        std::memmove(storage + static_cast<std::size_t>(shift_y) * sx, storage,
                     static_cast<std::size_t>(sy - shift_y) * sx * sizeof(T));
      }
    }
    const auto fill_x = [&](unsigned x0, unsigned x1) {
      if (x0 >= x1) return;
      for (unsigned y = 0; y < sy; ++y) {
        std::fill(storage + static_cast<std::size_t>(y) * sx + x0,
                  storage + static_cast<std::size_t>(y) * sx + x1, T{});
      }
    };
    const auto fill_y = [&](unsigned y0, unsigned y1) {
      if (y0 >= y1) return;
      std::fill(storage + static_cast<std::size_t>(y0) * sx,
                storage + static_cast<std::size_t>(y1) * sx, T{});
    };
    if (shift_x >= sx || shift_y >= sy) {
      std::fill(storage, storage + static_cast<std::size_t>(sx) * sy, T{});
      return;
    }
    if (dx > 0) {
      fill_x(sx - shift_x, sx);
    } else if (dx < 0) {
      fill_x(0, shift_x);
    }
    if (dy > 0) {
      fill_y(sy - shift_y, sy);
    } else if (dy < 0) {
      fill_y(0, shift_y);
    }
  };
  shift_in_place(presence_.data());
  if (!last_seen_ns_.empty()) {
    shift_in_place(last_seen_ns_.data());
  }
}

void SimpleObstacleLayer::touchCell(unsigned mx, unsigned my) {
  if (!has_dirty) {
    has_dirty = true;
    dirty_min_i = mx;
    dirty_min_j = my;
    dirty_max_i = mx + 1;
    dirty_max_j = my + 1;
    return;
  }
  dirty_min_i = std::min(dirty_min_i, mx);
  dirty_min_j = std::min(dirty_min_j, my);
  dirty_max_i = std::max(dirty_max_i, mx + 1);
  dirty_max_j = std::max(dirty_max_j, my + 1);
}

void SimpleObstacleLayer::markObstacles(const costmap_2d::Costmap2D& grid,
                                        double wx, double wy,
                                        costmap_2d::CostValue cost) {
  syncGeometry(grid);
  unsigned mx = 0;
  unsigned my = 0;
  if (!grid.worldToMap(wx, wy, mx, my)) {
    return;
  }
  setPresence(mx, my, cost == 0 ? 1 : cost);
  if (!last_seen_ns_.empty()) {
    last_seen_ns_[static_cast<std::size_t>(my) * geom_size_x + mx] =
        steadyNowNs();
  }
  touchCell(mx, my);
  data_current = false;
}

void SimpleObstacleLayer::clearArea(double min_wx, double min_wy,
                                    double max_wx, double max_wy) {
  if (presence_.empty()) {
    return;
  }
  int cx0 = 0;
  int cy0 = 0;
  int cx1 = 0;
  int cy1 = 0;
  // Cell bounds from the anchored geometry (floor semantics, matching
  // Costmap2D::worldToMapNoBounds).
  cx0 = static_cast<int>(std::floor((min_wx - geom_ox) / geom_res));
  cy0 = static_cast<int>(std::floor((min_wy - geom_oy) / geom_res));
  cx1 = static_cast<int>(std::floor((max_wx - geom_ox) / geom_res)) + 1;
  cy1 = static_cast<int>(std::floor((max_wy - geom_oy) / geom_res)) + 1;
  cx0 = std::max(cx0, 0);
  cy0 = std::max(cy0, 0);
  cx1 = std::min(cx1, static_cast<int>(geom_size_x));
  cy1 = std::min(cy1, static_cast<int>(geom_size_y));
  for (int y = cy0; y < cy1; ++y) {
    for (int x = cx0; x < cx1; ++x) {
      presence_[static_cast<std::size_t>(y) * geom_size_x + x] = 0;
    }
  }
  if (cx1 > cx0 && cy1 > cy0) {
    touchRect(static_cast<unsigned>(cx0), static_cast<unsigned>(cy0),
              static_cast<unsigned>(cx1 - 1), static_cast<unsigned>(cy1 - 1));
    data_current = false;
  }
}

void SimpleObstacleLayer::raytraceFreespace(
    const costmap_2d::Costmap2D& grid, double origin_x, double origin_y,
    double end_x, double end_y) {
  syncGeometry(grid);
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
  grid.raytraceLine(
      [&](unsigned int index) {
        const unsigned int mx = index % grid.getSizeInCellsX();
        const unsigned int my = index / grid.getSizeInCellsX();
        if (mx < geom_size_x && my < geom_size_y &&
            presence_[static_cast<std::size_t>(my) * geom_size_x + mx] != 0) {
          presence_[static_cast<std::size_t>(my) * geom_size_x + mx] = 0;
        }
      },
      origin_mx, origin_my, end_mx, end_my);
  // The whole ray span is dirty unconditionally: master content along the
  // ray must be re-derived this cycle (resetMap default / static rewrite)
  // even when presence was already free -- otherwise a mutated-but-empty
  // result would leave the layer not-current with no costs pass scheduled.
  touchRect(std::min(origin_mx, end_mx), std::min(origin_my, end_my),
            std::max(origin_mx, end_mx), std::max(origin_my, end_my));
  data_current = false;
}

void SimpleObstacleLayer::decaySweep() {
  if (decay_time_s <= 0.0 || last_seen_ns_.empty() || presence_.empty()) {
    return;
  }
  const auto now = std::chrono::steady_clock::now();
  if (last_decay_sweep.time_since_epoch().count() != 0 &&
      now - last_decay_sweep <
          std::chrono::duration<double>(decay_sweep_interval_s)) {
    return;
  }
  last_decay_sweep = now;
  const int64_t now_ns = steadyNowNs();
  const int64_t max_age_ns =
      static_cast<int64_t>(decay_time_s * static_cast<double>(k_ns_per_s));
  for (unsigned my = 0; my < geom_size_y; ++my) {
    for (unsigned mx = 0; mx < geom_size_x; ++mx) {
      const std::size_t idx =
          static_cast<std::size_t>(my) * geom_size_x + mx;
      if (presence_[idx] == 0) {
        continue;
      }
      if (now_ns - last_seen_ns_[idx] > max_age_ns) {
        presence_[idx] = 0;
        touchCell(mx, my);
      }
    }
  }
  data_current = false;
}

void SimpleObstacleLayer::updateBounds(double robot_x, double robot_y,
                                       double robot_yaw, double* min_x,
                                       double* min_y, double* max_x,
                                       double* max_y) {
  (void)robot_x;
  (void)robot_y;
  (void)robot_yaw;
  if (!enabled) {
    return;
  }
  if (master == nullptr) {
    return;
  }
  // Rolling recenter (shift) + throttled decay sweep; both expand the dirty
  // bbox, which then becomes this cycle's master window.
  syncGeometry(*master->getCostmap());
  decaySweep();
  if (!has_dirty) {
    return;
  }
  // Dirty cell bbox -> world bounds (cell corners; the master window is
  // derived from these by LayeredCostmap::updateMap).
  const double wx0 = geom_ox + static_cast<double>(dirty_min_i) * geom_res;
  const double wy0 = geom_oy + static_cast<double>(dirty_min_j) * geom_res;
  const double wx1 = geom_ox + static_cast<double>(dirty_max_i) * geom_res;
  const double wy1 = geom_oy + static_cast<double>(dirty_max_j) * geom_res;
  touch(wx0, wy0, min_x, min_y, max_x, max_y);
  touch(wx1, wy1, min_x, min_y, max_x, max_y);
}

void SimpleObstacleLayer::updateCosts(costmap_2d::Costmap2D& master_grid,
                                      int min_i, int min_j, int max_i,
                                      int max_j) {
  if (!enabled) {
    return;
  }
  const int size_x = static_cast<int>(master_grid.getSizeInCellsX());
  const int size_y = static_cast<int>(master_grid.getSizeInCellsY());
  const int window_x0 = std::max(0, min_i);
  const int window_y0 = std::max(0, min_j);
  const int window_x1 = std::min(size_x, max_i);
  const int window_y1 = std::min(size_y, max_j);

  // Write live presence cells within the window. Cleared cells are restored
  // by the pipeline (resetMap default + a StaticLayer beneath), never by
  // this layer.
  for (int y = window_y0; y < window_y1; ++y) {
    for (int x = window_x0; x < window_x1; ++x) {
      if (static_cast<unsigned>(x) >= geom_size_x ||
          static_cast<unsigned>(y) >= geom_size_y) {
        continue;
      }
      const unsigned char presence =
          presence_[static_cast<std::size_t>(y) * geom_size_x + x];
      if (presence == 0) {
        continue;
      }
      applyCost(master_grid, static_cast<unsigned>(x),
                static_cast<unsigned>(y), presence);
    }
  }
  has_dirty = false;
  data_current = true;
}

void SimpleObstacleLayer::reset() {
  if (!presence_.empty()) {
    std::fill(presence_.begin(), presence_.end(), 0);
  }
  has_dirty = false;
  data_current = false;
}

void SimpleObstacleLayer::matchSize() {
  if (master == nullptr) {
    return;
  }
  const auto* grid = master->getCostmap();
  geom_size_x = grid->getSizeInCellsX();
  geom_size_y = grid->getSizeInCellsY();
  geom_res = grid->getResolution();
  geom_ox = grid->getOriginX();
  geom_oy = grid->getOriginY();
  presence_.assign(static_cast<std::size_t>(geom_size_x) * geom_size_y, 0);
  last_seen_ns_.clear();
  if (decay_time_s > 0.0) {
    last_seen_ns_.assign(
        static_cast<std::size_t>(geom_size_x) * geom_size_y, 0);
  }
  has_dirty = false;
}

void SimpleObstacleLayer::applyCost(costmap_2d::Costmap2D& grid,
                                    unsigned mx, unsigned my,
                                    costmap_2d::CostValue cost) const {
  switch (combination_method) {
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

}  // namespace costmap_2d_layers
