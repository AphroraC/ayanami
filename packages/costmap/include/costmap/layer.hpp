#pragma once

#include <memory>
#include <atomic>
#include <string>
#include <utility>

#include <costmap/costmap_2d.hpp>

namespace costmap_2d {

class LayeredCostmap;

/// Pure virtual base class of costmap layers. Modules implement updateBounds/updateCosts to read and write the master grid.
///
/// Dirty-bounds protocol (nav2 CostmapLayer semantics, owned by the base):
/// - updateBounds implementations report the area whose master content must
///   be refreshed by expanding the shared world-space rectangle through
///   touch(x, y, ...); the rectangle is ONLY-GROW: LayeredCostmap validates
///   after every layer call and warns (naming the layer) when a layer
///   shrank it, then restores the previous value.
/// - addExtraBounds() lets host-side code (map clears, external producers)
///   force a region into the next updateBounds pass; the layer merges it
///   via useExtraBounds() and the area is repainted once.
class Layer {
 public:
  using Ptr = std::shared_ptr<Layer>;

  virtual ~Layer() = default;

  void initialize(LayeredCostmap* parent, std::string name) {
    master = parent;
    this->name = std::move(name);
    onInitialize();
  }

  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw,
                            double* min_x, double* min_y, double* max_x,
                            double* max_y) = 0;

  virtual void updateCosts(Costmap2D& master_grid, int min_i, int min_j,
                           int max_i, int max_j) = 0;

  virtual void activate() {}
  virtual void deactivate() {}
  virtual void reset() {}
  virtual void matchSize() {}
  virtual void onFootprintChanged() {}

  bool isCurrent() const { return data_current.load(); }
  bool isEnabled() const { return enabled; }
  void setEnabled(bool enabled) { this->enabled = enabled; }
  const std::string& getName() const { return name; }

  /// @brief The LayeredCostmap hosting this layer (null before initialize);
  /// read access for host-side helpers (footprint, master geometry).
  const LayeredCostmap* layeredCostmap() const { return master; }
  LayeredCostmap* layeredCostmap() { return master; }

  /// @brief Expand the shared dirty rectangle to include one world point
  /// (nav2 CostmapLayer::touch).
  static void touch(double x, double y, double* min_x, double* min_y,
                    double* max_x, double* max_y) {
    *min_x = std::min(*min_x, x);
    *min_y = std::min(*min_y, y);
    *max_x = std::max(*max_x, x);
    *max_y = std::max(*max_y, y);
  }

  /// @brief Force a world rectangle into the next updateBounds pass (nav2
  /// addExtraBounds): host-side producers of out-of-band changes (clears,
  /// injected maps) request a one-shot repaint of the region.
  void addExtraBounds(double min_x, double min_y, double max_x, double max_y) {
    if (!has_extra_bounds) {
      extra_min_x = min_x;
      extra_min_y = min_y;
      extra_max_x = max_x;
      extra_max_y = max_y;
      has_extra_bounds = true;
      return;
    }
    extra_min_x = std::min(extra_min_x, min_x);
    extra_min_y = std::min(extra_min_y, min_y);
    extra_max_x = std::max(extra_max_x, max_x);
    extra_max_y = std::max(extra_max_y, max_y);
  }

 protected:
  virtual void onInitialize() {}

  /// @brief Merge a pending extra-bounds request into the shared rectangle.
  /// @return true when a request was pending (the caller reported bounds).
  bool useExtraBounds(double* min_x, double* min_y, double* max_x,
                      double* max_y) {
    if (!has_extra_bounds) {
      return false;
    }
    *min_x = std::min(*min_x, extra_min_x);
    *min_y = std::min(*min_y, extra_min_y);
    *max_x = std::max(*max_x, extra_max_x);
    *max_y = std::max(*max_y, extra_max_y);
    has_extra_bounds = false;
    return true;
  }

  LayeredCostmap* master{nullptr};
  bool enabled{true};
  /// @brief Layer-level freshness: true once every input received so far has
  /// been written through updateCosts. Producers clear it when new input
  /// arrives (pending data); LayeredCostmap::isCurrent aggregates it so the
  /// execution safetyCheck gates on a fully written map.
  std::atomic<bool> data_current{false};
  std::string name;

 private:
  bool has_extra_bounds{false};
  double extra_min_x{0.0};
  double extra_min_y{0.0};
  double extra_max_x{0.0};
  double extra_max_y{0.0};
};

}  // namespace costmap_2d
