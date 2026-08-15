#pragma once

#include <string>

#include <costmap_2d/costmap_2d.hpp>
#include <costmap_2d/layer.hpp>

namespace costmap_2d_layers {

// 静态地图层: 持有一份只读地图, 每个更新周期把地图 cost 写回 master grid。
// 纯 C++ (无 ROS publisher), 地图通过 setMap / setMapPath 注入。
class StaticLayer : public costmap_2d::Layer {
 public:
  // 拷贝一份静态地图 (size/resolution/origin/costs)。
  void setMap(const costmap_2d::Costmap2D& map);

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
  void reset() override {}
  void matchSize() override {}

  bool hasMap() const { return has_map_; }

 private:
  costmap_2d::Costmap2D map_;
  bool has_map_{false};
};

}  // namespace costmap_2d_layers
