#pragma once

#include <algorithm>
#include <vector>

#include <costmap_2d/costmap_2d.hpp>
#include <costmap_2d/layer.hpp>

namespace costmap_2d_layers {

/// Simple obstacle layer (compact version): accepts markObstacles / clearArea APIs to write obstacles and
/// clear regions, and writes the marks back into the master grid during updateCosts. Does not consume sensors
/// directly (observation_sources is left to upper layers, which turn observations into markObstacles calls).
class SimpleObstacleLayer : public costmap_2d::Layer {
 public:
  enum class CombinationMethod { OVERWRITE = 0, MAXIMUM = 1, NONE = 2 };

  void setCombinationMethod(CombinationMethod method) {
    combination_method = method;
  }

  // Global maps restore the underlying StaticLayer before applying dynamic
  // marks, so they must not overwrite that restored value with FREE.
  void setClearToFree(bool clear_to_free) { clear_to_free = clear_to_free; }

  /// Marks an obstacle at world coordinates (wx, wy); written into the master grid during updateCosts.
  void markObstacles(double wx, double wy,
                     costmap_2d::CostValue cost = costmap_2d::LETHAL_OBSTACLE) {
    marks.push_back(Mark{wx, wy, cost});
    current = false;
  }

  /// Clears the world rectangle [min_wx, max_wx] x [min_wy, max_wy]:
  /// removes obstacle marks inside it and writes the covered cells back to FREE during updateCosts.
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
