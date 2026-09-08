#pragma once

#include <memory>
#include <atomic>
#include <string>
#include <utility>

#include <costmap_2d/costmap_2d.hpp>

namespace costmap_2d {

class LayeredCostmap;

/// Pure virtual base class of costmap layers. Plugins implement updateBounds/updateCosts to read and write the master grid.
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

  bool isCurrent() const { return current.load(); }
  bool isEnabled() const { return enabled; }
  void setEnabled(bool enabled) { enabled = enabled; }
  const std::string& getName() const { return name; }

 protected:
  virtual void onInitialize() {}

  LayeredCostmap* master{nullptr};
  bool enabled{true};
  std::atomic<bool> current{false};
  std::string name;
};

}  // namespace costmap_2d
