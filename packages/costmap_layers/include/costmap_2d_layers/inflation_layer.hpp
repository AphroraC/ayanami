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

// 膨胀层: 从 lethal 障碍 cell 出发, 用优先级队列 BFS (Dijkstra 式波前)
// 向四周传播, 代价按欧氏距离指数衰减 (nav2/legacy 风格)。
// 每次 updateCosts 只重建窗口内最近源的膨胀带, 复杂度 O(k log k) 非 O(N^2)。
class InflationLayer : public costmap_2d::Layer {
 public:
  void setInflationRadius(double radius) {
    inflation_radius_ = radius;
    caches_dirty_ = true;
    need_reinflation_ = true;
    current_ = false;
  }

  void setInscribedRadius(double radius) {
    inscribed_radius_ = radius;
    caches_dirty_ = true;
    need_reinflation_ = true;
    current_ = false;
  }

  void setCostScalingFactor(double factor) {
    weight_ = factor;
    caches_dirty_ = true;
    need_reinflation_ = true;
    current_ = false;
  }

  // 距离 (cells) -> cost: 0 -> LETHAL, 距障碍 <= inscribed -> 253,
  // 否则 (253-1) * exp(-weight * (euclid - inscribed)) 衰减取整。
  costmap_2d::CostValue computeCost(double distance_cells,
                                    double resolution) const {
    if (distance_cells == 0.0) {
      return costmap_2d::LETHAL_OBSTACLE;
    }
    const double euclidean_distance = distance_cells * resolution;
    if (euclidean_distance <= inscribed_radius_) {
      return costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
    }
    const double factor = std::exp(
        -weight_ * (euclidean_distance - inscribed_radius_));
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

  unsigned int getCellInflationRadius() const { return cell_inflation_radius_; }

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

  double inflation_radius_{0.6};
  double inscribed_radius_{0.5};
  double weight_{10.0};
  unsigned int cell_inflation_radius_{0};
  bool caches_dirty_{true};
  bool need_reinflation_{true};
  bool has_last_bounds_{false};
  double last_min_x_{std::numeric_limits<double>::max()};
  double last_min_y_{std::numeric_limits<double>::max()};
  double last_max_x_{std::numeric_limits<double>::lowest()};
  double last_max_y_{std::numeric_limits<double>::lowest()};
  std::vector<std::vector<double>> cached_distances_;
  std::vector<std::vector<costmap_2d::CostValue>> cached_costs_;
  std::vector<bool> seen_;
};

}  // namespace costmap_2d_layers
