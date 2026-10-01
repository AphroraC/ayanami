#pragma once

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <vector>

#include <costmap_2d/costmap_2d.hpp>
#include <costmap_2d/layer.hpp>

namespace costmap_2d_layers {

/// @brief Grid-backed obstacle layer (nav2 ObstacleLayer shape): occupancy
/// state lives in a layer-owned presence grid sized/anchored to the master
/// grid -- NOT in an unbounded mark list. mark/raytrace/clear mutate the
/// presence grid in O(1)/O(ray) and record a dirty cell bbox; updateCosts
/// writes live presence cells into the master grid within the dirty window.
/// Restoring cleared cells is the LayeredCostmap's job (resetMap wipes the
/// window; a StaticLayer beneath rewrites static content), so the layer
/// itself never writes FREE into the master.
///
/// Rolling maps: updateBounds re-anchors the presence grid to the master
/// origin (whole-cell shift, the same granularity Costmap2D::updateOrigin
/// uses), so presence stays cell-aligned while the window follows the robot.
///
/// Time decay (nav2-style aging for occluded marks): cells whose last
/// observation is older than decay_time are dropped from the presence grid
/// by a throttled full sweep in updateBounds (0 = decay disabled; enable on
/// maps without a rolling window, e.g. the global costmap, where occluded
/// marks would otherwise persist until an explicit clear command).
class SimpleObstacleLayer : public costmap_2d::Layer {
 public:
  enum class CombinationMethod { OVERWRITE = 0, MAXIMUM = 1, NONE = 2 };

  /// @brief Visualization readout: the raw mark grid (0 = free, else the
  /// cost this layer writes) and its geometry anchor. Read ONLY under the
  /// host's map shared lock (the writer mutates presence during updateMap).
  const std::vector<unsigned char>& presenceGrid() const { return presence_; }
  double geomOriginX() const { return geom_ox; }
  double geomOriginY() const { return geom_oy; }
  double geomResolution() const { return geom_res; }
  unsigned geomSizeX() const { return geom_size_x; }
  unsigned geomSizeY() const { return geom_size_y; }

  void setCombinationMethod(CombinationMethod method) {
    combination_method = method;
  }

  /// @brief Mark an obstacle at world coordinates; the cell's presence and
  /// last-seen timestamp are recorded in the presence grid.
  void markObstacles(const costmap_2d::Costmap2D& grid, double wx, double wy,
                     costmap_2d::CostValue cost = costmap_2d::LETHAL_OBSTACLE);

  /// @brief Clear presence inside the world rectangle [min_wx, max_wx] x
  /// [min_wy, max_wy].
  void clearArea(double min_wx, double min_wy, double max_wx, double max_wy);

  // Nav2-style freespace clearing: drop presence along the clipped Bresenham
  // ray from the sensor origin to the (clamped) endpoint.
  void raytraceFreespace(const costmap_2d::Costmap2D& grid, double origin_x,
                         double origin_y, double end_x, double end_y);

  /// @brief Drop presence cells last observed longer than this ago (seconds;
  /// 0 disables decay). Sweep runs throttled inside updateBounds.
  void setDecayTime(double seconds) {
    decay_time_s = seconds > 0.0 ? seconds : 0.0;
    if (decay_time_s > 0.0 && !presence_.empty() && last_seen_ns_.empty()) {
      last_seen_ns_.assign(presence_.size(), 0);
    }
    if (decay_time_s == 0.0) {
      last_seen_ns_.clear();
    }
  }
  double decayTime() const { return decay_time_s; }
  /// @brief Minimum interval between full decay sweeps (seconds).
  void setDecaySweepInterval(double seconds) {
    decay_sweep_interval_s = seconds > 0.0 ? seconds : 1.0;
  }

  void updateBounds(double robot_x, double robot_y, double robot_yaw,
                    double* min_x, double* min_y, double* max_x,
                    double* max_y) override;

  void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j,
                   int max_i, int max_j) override;

  void activate() override { data_current = true; }
  void deactivate() override {}
  void reset() override;
  void matchSize() override;

 private:
  /// @brief Resize/shift the presence grid to match `grid`'s geometry
  /// (called from every grid-taking entry). A size/resolution change resets
  /// the occupancy; an origin change (rolling recenter) shifts it by whole
  /// cells, preserving data.
  void syncGeometry(const costmap_2d::Costmap2D& grid);

  /// @brief Shift presence/last_seen by whole cells (rolling recenter).
  void shiftPresence(int dx, int dy);
  /// @brief Throttled decay sweep: drop stale presence, expand dirty bbox.
  void decaySweep();
  /// @brief Record a dirty cell (bounds in cells, inclusive-exclusive).
  void touchCell(unsigned mx, unsigned my);
  /// @brief Record an inclusive cell rectangle as dirty.
  void touchRect(unsigned x0, unsigned y0, unsigned x1, unsigned y1) {
    touchCell(x0, y0);
    touchCell(x1, y1);
  }
  bool presenceAt(unsigned mx, unsigned my) const {
    return presence_[static_cast<std::size_t>(my) * geom_size_x + mx] != 0;
  }
  void setPresence(unsigned mx, unsigned my, unsigned char value) {
    presence_[static_cast<std::size_t>(my) * geom_size_x + mx] = value;
  }
  void applyCost(costmap_2d::Costmap2D& grid, unsigned mx, unsigned my,
                 costmap_2d::CostValue cost) const;

  CombinationMethod combination_method{CombinationMethod::MAXIMUM};
  /// @brief Presence grid (0 = free, else the cost to write) anchored at
  /// (geom_ox, geom_oy) with (geom_size_x, geom_size_y) cells.
  std::vector<unsigned char> presence_;
  /// @brief Last-observation timestamps (ns, steady_clock); allocated only
  /// when decay is enabled.
  std::vector<int64_t> last_seen_ns_;
  double geom_ox{0.0};
  double geom_oy{0.0};
  double geom_res{0.05};
  unsigned geom_size_x{0};
  unsigned geom_size_y{0};
  /// @brief Dirty cell bbox (inclusive min, exclusive max) since the last
  /// updateCosts.
  bool has_dirty{false};
  unsigned dirty_min_i{0};
  unsigned dirty_min_j{0};
  unsigned dirty_max_i{0};  // exclusive
  unsigned dirty_max_j{0};  // exclusive
  double decay_time_s{0.0};
  double decay_sweep_interval_s{1.0};
  std::chrono::steady_clock::time_point last_decay_sweep{};
};

}  // namespace costmap_2d_layers
