#pragma once

#include <algorithm>
#include <cmath>
#include <functional>
#include <queue>
#include <limits>
#include <vector>

#include <costmap_2d/costmap_2d.hpp>
#include <costmap_2d/layer.hpp>

namespace costmap_2d_layers {

/// Inflation layer: starting from lethal obstacle cells, propagates outward with a
/// priority-queue BFS (Dijkstra-style wavefront); cost decays exponentially with Euclidean
/// distance (nav2/legacy style). Each updateCosts rebuilds only the inflation band of the nearest
class InflationLayer : public costmap_2d::Layer {
 public:
  void setInflationRadius(double radius) {
    inflation_radius = radius;
    caches_dirty = true;
    need_reinflation = true;
    current = false;
  }

  void setInscribedRadius(double radius) {
    inscribed_radius = radius;
    caches_dirty = true;
    need_reinflation = true;
    current = false;
  }

  void setCostScalingFactor(double factor) {
    weight = factor;
    caches_dirty = true;
    need_reinflation = true;
    current = false;
  }

  /// Distance (cells) -> cost: 0 -> LETHAL, within inscribed radius of an obstacle -> 253,
  // otherwise (253-1) * exp(-weight * (euclid - inscribed)), rounded.
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
    return static_cast<costmap_2d::CostValue>(std::round(
        (costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 1) * factor));
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
  struct QueueCell {
    double distance{0.0};
    unsigned int x{0};
    unsigned int y{0};
    unsigned int src_x{0};
    unsigned int src_y{0};
    bool operator>(const QueueCell& other) const {
      return distance > other.distance;
    }
  };

  void ensureCaches(double resolution);
  void enqueue(const costmap_2d::Costmap2D& grid,
               std::priority_queue<QueueCell, std::vector<QueueCell>,
                                   std::greater<>>& queue,
               std::vector<bool>& seen, unsigned int mx, unsigned int my,
               unsigned int src_x, unsigned int src_y) const;
  costmap_2d::CostValue costLookup(unsigned int mx, unsigned int my,
                                   unsigned int src_x, unsigned int src_y,
                                   double resolution) const;

  double inflation_radius{0.6};
  double inscribed_radius{0.5};
  double weight{10.0};
  unsigned int cell_inflation_radius{0};
  bool caches_dirty{true};
  bool need_reinflation{true};
  bool has_last_bounds{false};
  double last_min_x{std::numeric_limits<double>::max()};
  double last_min_y{std::numeric_limits<double>::max()};
  double last_max_x{std::numeric_limits<double>::lowest()};
  double last_max_y{std::numeric_limits<double>::lowest()};
  std::vector<std::vector<double>> cached_distances;
  std::vector<std::vector<costmap_2d::CostValue>> cached_costs;
  std::vector<bool> seen;
};

}  // namespace costmap_2d_layers
