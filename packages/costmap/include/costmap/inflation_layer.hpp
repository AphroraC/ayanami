#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <vector>

#include <costmap/costmap_2d.hpp>
#include <costmap/layer.hpp>

namespace costmap_2d {

/// Inflation layer: starting from lethal obstacle cells, propagates outward with a
/// distance-ordered wavefront; cost decays exponentially with Euclidean
/// distance (nav2/legacy style). Each updateCosts rebuilds only the inflation band of the nearest
///
/// Wavefront mechanics (nav2 integer-bucket discipline, without the heap):
/// cells are bucketed by SQUARED integer cell distance (dx*dx+dy*dy), an
/// exact monotone refinement of the Euclidean order -- O(1) push, buckets
/// traversed in increasing distance, no floating point in the hot path.
/// Cost/distance lookups go through one-dimensional flat caches indexed by
/// |dx| * cache_length + |dy|. The visited set is a uint32 epoch array:
/// bumping the epoch replaces the per-cycle full clear.
class InflationLayer : public costmap_2d::Layer {
 public:
  using Ptr = std::shared_ptr<InflationLayer>;

  void setInflationRadius(double radius) {
    inflation_radius = radius;
    caches_dirty = true;
    need_reinflation = true;
    data_current = false;
  }

  void setInscribedRadius(double radius) {
    inscribed_radius = radius;
    caches_dirty = true;
    need_reinflation = true;
    data_current = false;
  }

  void setCostScalingFactor(double factor) {
    weight = factor;
    caches_dirty = true;
    need_reinflation = true;
    data_current = false;
  }

  /// Distance (cells) -> cost, numerically identical to nav2: 0 cells ->
  /// LETHAL, within the inscribed radius -> 253, otherwise the exponential
  /// decay (253-1) * exp(-cost_scaling_factor * (euclid - inscribed)),
  /// truncated like nav2's static_cast (no rounding).
  costmap_2d::CostValue computeCost(double distance_cells,
                                    double resolution) const {
    if (distance_cells == 0.0) {
      return costmap_2d::LETHAL_OBSTACLE;
    }
    const double euclideanDistance = distance_cells * resolution;
    if (euclideanDistance <= inscribed_radius) {
      return costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
    }
    const double factor = std::exp(
        -weight * (euclideanDistance - inscribed_radius));
    return static_cast<costmap_2d::CostValue>(
        (costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 1) * factor);
  }

  void updateBounds(double, double, double, double*, double*, double*,
                    double*) override;

  void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j,
                   int max_i, int max_j) override;

  void activate() override {}
  void deactivate() override {}
  void reset() override {}
  void matchSize() override {}
  void onFootprintChanged() override;

  unsigned int getCellInflationRadius() const { return cell_inflation_radius; }

 private:
  /// @brief One wavefront entry: the cell plus the lethal source cell it was
  /// reached from (the cost is a function of the cell offset from its
  /// source). No stored distance: the bucket index IS the distance.
  struct QueueCell {
    unsigned int x{0};
    unsigned int y{0};
    unsigned int src_x{0};
    unsigned int src_y{0};
  };

  void ensureCaches(double resolution);
  /// @brief Push one neighbor of a processing cell into its distance bucket.
  /// Bails on out-of-bounds, already-visited (epoch), or beyond-radius cells.
  void enqueue(const costmap_2d::Costmap2D& grid,
               std::vector<std::vector<QueueCell>>& buckets, unsigned int mx,
               unsigned int my, unsigned int src_x, unsigned int src_y);
  costmap_2d::CostValue costLookup(unsigned int mx, unsigned int my,
                                   unsigned int src_x, unsigned int src_y,
                                   double resolution) const;

  double inflation_radius{0.6};
  double inscribed_radius{0.5};
  double weight{10.0};
  unsigned int cell_inflation_radius{0};
  bool caches_dirty{true};
  bool need_reinflation{true};
  /// @brief Flat caches over cell offsets, indexed |dx| * cache_length + |dy|
  /// (cache_length = cell_inflation_radius + 2). One allocation, contiguous.
  std::vector<double> cached_distances;
  std::vector<costmap_2d::CostValue> cached_costs;
  /// @brief Distance buckets: index = squared cell distance from the source.
  /// Buckets are traversed in index order, which is exactly the Euclidean
  /// wavefront order (squared distance is a monotone function of Euclidean
  /// distance for integer offsets). Capacity is reused across cycles.
  std::vector<std::vector<QueueCell>> buckets;
  /// @brief Visited set with per-cycle epochs: a cell is visited in cycle N
  /// iff seen_epoch[index] == epoch. Bumping the epoch replaces the previous
  /// full-array clear (no O(map) memset per updateCosts).
  std::vector<std::uint32_t> seen_epoch;
  std::uint32_t epoch{0};
};

}  // namespace costmap_2d
