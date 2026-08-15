#pragma once

#include <memory>
#include <string>
#include <utility>

#include <costmap_2d/costmap_2d.hpp>

namespace costmap_2d {

class LayeredCostmap;

// 成本图层的纯虚基类。插件实现 updateBounds/updateCosts 读写主 grid。
class Layer {
 public:
  using Ptr = std::shared_ptr<Layer>;

  virtual ~Layer() = default;

  void initialize(LayeredCostmap* parent, std::string name) {
    master_ = parent;
    name_ = std::move(name);
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

  bool isCurrent() const { return current_; }
  bool isEnabled() const { return enabled_; }
  void setEnabled(bool enabled) { enabled_ = enabled; }
  const std::string& getName() const { return name_; }

 protected:
  virtual void onInitialize() {}

  LayeredCostmap* master_{nullptr};
  bool enabled_{true};
  bool current_{true};
  std::string name_;
};

}  // namespace costmap_2d
