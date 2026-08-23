#pragma once

#include <string>
#include <mutex>
#include <optional>

#include <costmap_2d/costmap_2d.hpp>
#include <costmap_2d/layer.hpp>

namespace costmap_2d_layers {

// 静态地图层: 持有一份只读地图, 每个更新周期把地图 cost 写回 master grid。
// 纯 C++ (无 ROS publisher), 地图通过 setMap / setMapPath 注入。
class StaticLayer : public costmap_2d::Layer {
 public:
  // 拷贝一份静态地图 (size/resolution/origin/costs)。仅用于初始化线程；
  // 运行期生产者必须用 queueMap(), 由 update cycle 安全应用。
  void setMap(const costmap_2d::Costmap2D& map);
  // Nav2-style deferred update: callbacks/producers queue a map, the costmap
  // update cycle applies it at the beginning of updateBounds().
  void queueMap(const costmap_2d::Costmap2D& map);

  // 从 PGM (P5/P2, 8 位) 文件加载静态地图。
  // 像素映射: 0(black) -> LETHAL_OBSTACLE, 255(white) -> FREE_SPACE,
  //           其余 -> NO_INFORMATION。PGM 不含分辨率, 由调用方给出。
  void setMapPath(const std::string& path, double resolution = 1.0,
                  double origin_x = 0.0, double origin_y = 0.0);

  void updateBounds(double robot_x, double robot_y, double robot_yaw,
                    double* min_x, double* min_y, double* max_x,
                    double* max_y) override;

  // 把窗口内层地图 cost 写回 master (true overwrite, 含 NO_INFORMATION)。
  void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j,
                   int max_i, int max_j) override;

  void activate() override {}
  void deactivate() override {}
  void reset() override {
    full_update_ = has_map_;
    current_ = false;
  }
  void matchSize() override;

  bool hasMap() const { return has_map_; }

  // Request restoration of a world-coordinate region on the next update.
  void addUpdateBounds(double min_x, double min_y, double max_x,
                       double max_y);

 private:
  void processMap(const costmap_2d::Costmap2D& map);
  costmap_2d::Costmap2D map_;
  std::optional<costmap_2d::Costmap2D> pending_map_;
  std::mutex map_mutex_;
  bool has_map_{false};
  bool map_received_in_update_bounds_{false};
  bool has_updated_data_{false};
  bool full_update_{false};
  bool has_extra_bounds_{false};
  bool have_master_origin_{false};
  double last_master_origin_x_{0.0};
  double last_master_origin_y_{0.0};
  double extra_min_x_{0.0};
  double extra_min_y_{0.0};
  double extra_max_x_{0.0};
  double extra_max_y_{0.0};
};

}  // namespace costmap_2d_layers
