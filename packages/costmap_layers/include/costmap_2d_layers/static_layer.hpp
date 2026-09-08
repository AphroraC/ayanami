#pragma once

#include <string>
#include <mutex>
#include <optional>

#include <costmap_2d/costmap_2d.hpp>
#include <costmap_2d/layer.hpp>

namespace costmap_2d_layers {

/// Static map layer: holds a read-only map and writes the map costs back into the master grid each update cycle.
/// Pure C++ (no ROS publisher); the map is injected via setMap / setMapPath.
class StaticLayer : public costmap_2d::Layer {
 public:
  /// Copies a static map (size/resolution/origin/costs). For the initialization thread only;
  /// runtime producers must use queueMap, which the update cycle applies safely.
  void setMap(const costmap_2d::Costmap2D& map);
  // Nav2-style deferred update: callbacks/producers queue a map, the costmap
  // update cycle applies it at the beginning of updateBounds.
  void queueMap(const costmap_2d::Costmap2D& map);

  /// Loads a static map from a PGM (P5/P2, 8-bit) file.
  /// Pixel mapping: 0 (black) -> LETHAL_OBSTACLE, 255 (white) -> FREE_SPACE,
  /// everything else -> NO_INFORMATION. A PGM carries no resolution; the caller supplies it.
  void setMapPath(const std::string& path, double resolution = 1.0,
                  double origin_x = 0.0, double origin_y = 0.0);

  void updateBounds(double robot_x, double robot_y, double robot_yaw,
                    double* min_x, double* min_y, double* max_x,
                    double* max_y) override;

  /// Writes the in-window layer map costs back into the master (true overwrite, including NO_INFORMATION).
  void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j,
                   int max_i, int max_j) override;

  void activate() override {}
  void deactivate() override {}
  void reset() override {
    full_update = has_map;
    current = false;
  }
  void matchSize() override;

  bool hasMap() const { return has_map; }

  // Request restoration of a world-coordinate region on the next update.
  void addUpdateBounds(double min_x, double min_y, double max_x,
                       double max_y);

 private:
  void processMap(const costmap_2d::Costmap2D& map);
  costmap_2d::Costmap2D map;
  std::optional<costmap_2d::Costmap2D> pending_map;
  std::mutex map_mutex;
  bool has_map{false};
  bool map_received_in_update_bounds{false};
  bool has_updated_data{false};
  bool full_update{false};
  bool has_extra_bounds{false};
  bool have_master_origin{false};
  double last_master_origin_x{0.0};
  double last_master_origin_y{0.0};
  double extra_min_x{0.0};
  double extra_min_y{0.0};
  double extra_max_x{0.0};
  double extra_max_y{0.0};
};

}  // namespace costmap_2d_layers
