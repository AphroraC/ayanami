#pragma once

#include <string>
#include <mutex>
#include <optional>

#include <costmap/costmap_2d.hpp>
#include <costmap/layer.hpp>

namespace costmap_2d {

/// @brief PGM pixel -> cost interpretation, the map_server yaml semantics
/// (nav2 StaticLayer::interpretValue counterpart). Defaults match map_server.
struct StaticMapSemantics {
  /// @brief negate=1 inverts occupancy: white pixels become occupied
  /// (standard maps are black-occupied, so negate defaults to false).
  bool negate{false};
  /// @brief occupancy ratio above this -> LETHAL_OBSTACLE.
  double occupied_thresh{0.65};
  /// @brief occupancy ratio below this -> FREE_SPACE.
  double free_thresh{0.196};
  /// @brief trinary: between the thresholds resolves through track_unknown;
  /// scale: grayscale occupancy linearly mapped to cost.
  bool trinary{true};
  /// @brief trinary between-threshold resolution: true -> NO_INFORMATION,
  /// false -> FREE_SPACE (the master default when not tracking unknown).
  bool track_unknown{false};
};

/// Static map layer: holds a read-only map and writes the map costs back into the master grid each update cycle.
/// Pure C++ (no ROS publisher); the map is injected via setMap / setMapPath.
class StaticLayer : public costmap_2d::Layer {
 public:
  using Ptr = std::shared_ptr<StaticLayer>;

  /// Copies a static map (size/resolution/origin/costs). For the initialization thread only;
  /// runtime producers must use queueMap, which the update cycle applies safely.
  void setMap(const costmap_2d::Costmap2D& map);
  // Nav2-style deferred update: callbacks/producers queue a map, the costmap
  // update cycle applies it at the beginning of updateBounds.
  void queueMap(const costmap_2d::Costmap2D& map);

  /// Loads a static map from a PGM (P5/P2, 8-bit) file.
  /// Pixel mapping follows the map semantics (see StaticMapSemantics):
  /// occupancy ratio (negate-aware) above occupied_thresh -> LETHAL, below
  /// free_thresh -> FREE, and between them trinary-resolved or scaled. A PGM
  /// carries no resolution; the caller supplies it.
  void setMapPath(const std::string& path, double resolution = 1.0,
                  double origin_x = 0.0, double origin_y = 0.0,
                  const StaticMapSemantics& semantics = {});

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
    data_current = false;
  }
  void matchSize() override;

  bool hasMap() const { return has_map; }
  /// @brief Read-only view of the layer's own map grid (PGM geometry and
  /// costs; static content -- only mutated by setMap/queueMap at configure
  /// time, safe to read without a lock afterwards).
  const costmap_2d::Costmap2D& layerMap() const { return map; }

 private:
  void processMap(const costmap_2d::Costmap2D& map);
  StaticMapSemantics semantics;
  costmap_2d::Costmap2D map;
  std::optional<costmap_2d::Costmap2D> pending_map;
  std::mutex map_mutex;
  bool has_map{false};
  bool map_received_in_update_bounds{false};
  bool has_updated_data{false};
  bool full_update{false};
  bool have_master_origin{false};
  double last_master_origin_x{0.0};
  double last_master_origin_y{0.0};
};

}  // namespace costmap_2d
