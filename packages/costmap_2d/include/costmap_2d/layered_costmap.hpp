#pragma once

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <cmath>

#include <costmap_2d/layer.hpp>

namespace costmap_2d {

// 主成本图宿主：持有主 grid + 插件层列表，每周期 updateMap 驱动各层
// updateBounds -> updateCosts 把层数据写回主 grid。
class LayeredCostmap {
 public:
  using Ptr = std::shared_ptr<LayeredCostmap>;

  LayeredCostmap(std::string global_frame, bool rolling_window,
                 bool track_unknown)
      : global_frame(std::move(global_frame)),
        rolling_window(rolling_window),
        track_unknown(track_unknown) {}

  void resizeMap(unsigned int size_x, unsigned int size_y, double resolution,
                 double origin_x, double origin_y,
                 bool size_locked = false) {
    costmap.resizeMap(size_x, size_y, resolution, origin_x, origin_y,
                       track_unknown ? NO_INFORMATION : FREE_SPACE);
    this->size_locked = size_locked;
    initialized = true;
    for (auto& plugin : plugins) {
      plugin->matchSize();
    }
  }

  void addLayer(const Layer::Ptr& plugin) {
    if (plugin) {
      plugin->initialize(this, plugin->getName());
      plugins.push_back(plugin);
    }
  }

  // 驱动所有启用层: 先 updateBounds 收集待刷新边界, 再 updateCosts 写主 grid。
  void updateMap(double robot_x, double robot_y, double robot_yaw) {
    if (rolling_window) {
      costmap.updateOrigin(robot_x - costmap.getSizeInMetersX() * 0.5,
                            robot_y - costmap.getSizeInMetersY() * 0.5);
    }

    double min_x = std::numeric_limits<double>::max();
    double min_y = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double max_y = std::numeric_limits<double>::lowest();

    updateBoundsForLayers(plugins, robot_x, robot_y, robot_yaw, min_x, min_y,
                          max_x, max_y);

    if (min_x > max_x || min_y > max_y) {
      minx = miny = maxx = maxy = 0.0;
      return;
    }

    minx = min_x;
    miny = min_y;
    maxx = max_x;
    maxy = max_y;

    int min_i = 0;
    int min_j = 0;
    int max_i = 0;
    int max_j = 0;
    costmap.worldToMapEnforceBounds(min_x, min_y, min_i, min_j);
    costmap.worldToMapEnforceBounds(max_x, max_y, max_i, max_j);
    max_i = std::min(max_i + 1, static_cast<int>(costmap.getSizeInCellsX()));
    max_j = std::min(max_j + 1, static_cast<int>(costmap.getSizeInCellsY()));

    // Nav2 invariant: rebuild the dirty master window from layer state.
    costmap.resetMap(static_cast<unsigned int>(min_i),
                      static_cast<unsigned int>(min_j),
                      static_cast<unsigned int>(max_i),
                      static_cast<unsigned int>(max_j));
    updateCostsForLayers(plugins, costmap, min_i, min_j, max_i, max_j);
  }

  bool isCurrent() const {
    for (const auto& plugin : plugins) {
      if (plugin->isEnabled() && !plugin->isCurrent()) {
        return false;
      }
    }
    return true;
  }

  void setFootprint(const Footprint& footprint) {
    this->footprint = footprint;
    inscribed_radius = std::numeric_limits<double>::max();
    circumscribed_radius = 0.0;
    if (footprint.size() > 2) {
      auto distanceToSegment = [](const Point& first, const Point& second) {
        const double dx = second.x - first.x;
        const double dy = second.y - first.y;
        const double length_sq = dx * dx + dy * dy;
        if (length_sq <= 1e-18) return std::hypot(first.x, first.y);
        const double t = std::clamp(
            -(first.x * dx + first.y * dy) / length_sq, 0.0, 1.0);
        return std::hypot(first.x + t * dx, first.y + t * dy);
      };
      for (std::size_t index = 0; index < footprint.size(); ++index) {
        const Point& point = footprint[index];
        const Point& next = footprint[(index + 1) % footprint.size()];
        inscribed_radius = std::min(
            inscribed_radius,
            std::min(std::hypot(point.x, point.y),
                     distanceToSegment(point, next)));
        circumscribed_radius =
            std::max(circumscribed_radius, std::hypot(point.x, point.y));
      }
    } else {
      inscribed_radius = 0.0;
    }
    for (auto& plugin : plugins) plugin->onFootprintChanged();
  }

  const Footprint& getFootprint() const { return footprint; }
  double getInscribedRadius() const { return inscribed_radius; }
  double getCircumscribedRadius() const { return circumscribed_radius; }

  Costmap2D* getCostmap() { return &costmap; }
  const Costmap2D* getCostmap() const { return &costmap; }

  void getUpdatedBounds(double& minx, double& miny, double& maxx,
                        double& maxy) const {
    minx = this->minx;
    miny = this->miny;
    maxx = this->maxx;
    maxy = this->maxy;
  }

  const std::string& getGlobalFrameID() const { return global_frame; }
  bool isRolling() const { return rolling_window; }
  bool isTrackingUnknown() const { return track_unknown; }
  bool isInitialized() const { return initialized; }
  bool isSizeLocked() const { return size_locked; }

  std::vector<Layer::Ptr>& getPlugins() { return plugins; }
  const std::vector<Layer::Ptr>& getPlugins() const { return plugins; }

 private:
  static void updateBoundsForLayers(const std::vector<Layer::Ptr>& layers,
                                    double robot_x, double robot_y,
                                    double robot_yaw, double& min_x,
                                    double& min_y, double& max_x,
                                    double& max_y) {
    for (const auto& layer : layers) {
      if (layer->isEnabled()) {
        layer->updateBounds(robot_x, robot_y, robot_yaw, &min_x, &min_y,
                            &max_x, &max_y);
      }
    }
  }

  static void updateCostsForLayers(const std::vector<Layer::Ptr>& layers,
                                   Costmap2D& costmap, int min_i, int min_j,
                                   int max_i, int max_j) {
    for (const auto& layer : layers) {
      if (layer->isEnabled()) {
        layer->updateCosts(costmap, min_i, min_j, max_i, max_j);
      }
    }
  }

  Costmap2D costmap;
  std::string global_frame;
  bool rolling_window{false};
  bool track_unknown{false};
  bool initialized{false};
  bool size_locked{false};
  double minx{0.0};
  double miny{0.0};
  double maxx{0.0};
  double maxy{0.0};
  std::vector<Layer::Ptr> plugins;
  Footprint footprint;
  double inscribed_radius{0.0};
  double circumscribed_radius{0.0};
};

}  // namespace costmap_2d
