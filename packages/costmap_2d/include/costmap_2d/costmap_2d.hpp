#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <climits>
#include <vector>

#include <costmap_2d/cost_values.hpp>

namespace costmap_2d {

struct MapLocation {
  unsigned int x{0};
  unsigned int y{0};
};

struct Point {
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

using Footprint = std::vector<Point>;

class Costmap2D {
 public:
  Costmap2D() = default;

  Costmap2D(unsigned int size_x, unsigned int size_y, double resolution,
            double origin_x, double origin_y,
            CostValue default_value = FREE_SPACE) {
    resizeMap(size_x, size_y, resolution, origin_x, origin_y, default_value);
  }

  virtual ~Costmap2D() = default;

  // Ruling #6: 显式 this-> 成员赋值, 避免参数遮蔽成员（历史 size_x = size_x 自赋值 bug）
  void resizeMap(unsigned int size_x, unsigned int size_y, double resolution,
                 double origin_x, double origin_y,
                 CostValue default_value = FREE_SPACE) {
    this->size_x = size_x;
    this->size_y = size_y;
    this->resolution = resolution;
    this->origin_x = origin_x;
    this->origin_y = origin_y;
    this->default_value = default_value;
    data.assign(static_cast<std::size_t>(size_x) * size_y, default_value);
  }

  void resetMap() { std::fill(data.begin(), data.end(), default_value); }

  void resetMap(unsigned int x0, unsigned int y0, unsigned int xn,
                unsigned int yn) {
    resetMapToValue(x0, y0, xn, yn, default_value);
  }

  void resetMapToValue(unsigned int x0, unsigned int y0, unsigned int xn,
                       unsigned int yn, CostValue value) {
    x0 = std::min(x0, size_x);
    y0 = std::min(y0, size_y);
    xn = std::min(xn, size_x);
    yn = std::min(yn, size_y);
    if (x0 >= xn || y0 >= yn) {
      return;
    }
    for (unsigned int y = y0; y < yn; ++y) {
      std::fill(data.begin() + getIndex(x0, y),
                data.begin() + getIndex(xn, y), value);
    }
  }

  CostValue getCost(unsigned int mx, unsigned int my) const {
    return data[getIndex(mx, my)];
  }

  CostValue getCost(unsigned int index) const { return data[index]; }

  void setCost(unsigned int mx, unsigned int my, CostValue cost) {
    data[getIndex(mx, my)] = cost;
  }

  bool worldToMap(double wx, double wy, unsigned int& mx,
                  unsigned int& my) const {
    if (wx < origin_x || wy < origin_y || resolution <= 0.0) {
      return false;
    }
    mx = static_cast<unsigned int>((wx - origin_x) / resolution);
    my = static_cast<unsigned int>((wy - origin_y) / resolution);
    return mx < size_x && my < size_y;
  }

  void worldToMapNoBounds(double wx, double wy, int& mx, int& my) const {
    mx = static_cast<int>(std::floor((wx - origin_x) / resolution));
    my = static_cast<int>(std::floor((wy - origin_y) / resolution));
  }

  void worldToMapEnforceBounds(double wx, double wy, int& mx, int& my) const {
    if (size_x == 0 || size_y == 0 || resolution <= 0.0) {
      mx = 0;
      my = 0;
      return;
    }
    // Compare in world space before converting to int. Layer bounds may use
    // numeric_limits<double>::max()/lowest() for full reinflation.
    if (wx < origin_x) {
      mx = 0;
    } else if (wx > origin_x + resolution * size_x) {
      mx = static_cast<int>(size_x) - 1;
    } else {
      mx = static_cast<int>((wx - origin_x) / resolution);
    }
    if (wy < origin_y) {
      my = 0;
    } else if (wy > origin_y + resolution * size_y) {
      my = static_cast<int>(size_y) - 1;
    } else {
      my = static_cast<int>((wy - origin_y) / resolution);
    }
  }

  void mapToWorld(unsigned int mx, unsigned int my, double& wx,
                  double& wy) const {
    wx = origin_x + (static_cast<double>(mx) + 0.5) * resolution;
    wy = origin_y + (static_cast<double>(my) + 0.5) * resolution;
  }

  unsigned int getIndex(unsigned int mx, unsigned int my) const {
    return my * size_x + mx;
  }

  void indexToCells(unsigned int index, unsigned int& mx,
                    unsigned int& my) const {
    my = index / size_x;
    mx = index - my * size_x;
  }

  unsigned char* getCharMap() {
    return reinterpret_cast<unsigned char*>(data.data());
  }

  const unsigned char* getCharMap() const {
    return reinterpret_cast<const unsigned char*>(data.data());
  }

  unsigned int getSizeInCellsX() const { return size_x; }
  unsigned int getSizeInCellsY() const { return size_y; }
  double getSizeInMetersX() const { return size_x * resolution; }
  double getSizeInMetersY() const { return size_y * resolution; }
  double getResolution() const { return resolution; }
  double getOriginX() const { return origin_x; }
  double getOriginY() const { return origin_y; }
  CostValue getDefaultValue() const { return default_value; }
  void setDefaultValue(CostValue value) { default_value = value; }

  unsigned int cellDistance(double world_dist) const {
    if (resolution <= 0.0 || world_dist <= 0.0) {
      return 0;
    }
    return static_cast<unsigned int>(std::ceil(world_dist / resolution));
  }

  bool inBounds(unsigned int mx, unsigned int my) const {
    return mx < size_x && my < size_y;
  }

  template <class Action>
  void raytraceLine(Action action, unsigned int x0, unsigned int y0,
                    unsigned int x1, unsigned int y1,
                    unsigned int max_length = UINT_MAX,
                    unsigned int min_length = 0) const {
    if (!inBounds(x0, y0) || !inBounds(x1, y1)) return;
    const int full_dx = static_cast<int>(x1) - static_cast<int>(x0);
    const int full_dy = static_cast<int>(y1) - static_cast<int>(y0);
    const double distance = std::hypot(full_dx, full_dy);
    if (distance < min_length) return;
    const unsigned int start_x = distance > 0.0
                                     ? static_cast<unsigned int>(
                                           x0 + full_dx / distance * min_length)
                                     : x0;
    const unsigned int start_y = distance > 0.0
                                     ? static_cast<unsigned int>(
                                           y0 + full_dy / distance * min_length)
                                     : y0;
    int dx = static_cast<int>(x1) - static_cast<int>(start_x);
    int dy = static_cast<int>(y1) - static_cast<int>(start_y);
    const unsigned int abs_dx = static_cast<unsigned int>(std::abs(dx));
    const unsigned int abs_dy = static_cast<unsigned int>(std::abs(dy));
    const int offset_dx = dx >= 0 ? 1 : -1;
    const int offset_dy = (dy >= 0 ? 1 : -1) * static_cast<int>(size_x);
    unsigned int offset = getIndex(start_x, start_y);
    const double scale = distance == 0.0
                             ? 1.0
                             : std::min(1.0, max_length / distance);
    auto trace = [&](unsigned int abs_da, unsigned int abs_db, int offset_a,
                     int offset_b) {
      int error = static_cast<int>(abs_da / 2);
      const unsigned int end =
          std::min(static_cast<unsigned int>(scale * abs_da), abs_da);
      for (unsigned int index = 0; index < end; ++index) {
        action(offset);
        offset = static_cast<unsigned int>(static_cast<int>(offset) + offset_a);
        error += static_cast<int>(abs_db);
        if (static_cast<unsigned int>(error) >= abs_da) {
          offset =
              static_cast<unsigned int>(static_cast<int>(offset) + offset_b);
          error -= static_cast<int>(abs_da);
        }
      }
      action(offset);
    };
    if (abs_dx >= abs_dy) {
      trace(abs_dx, abs_dy, offset_dx, offset_dy);
    } else {
      trace(abs_dy, abs_dx, offset_dy, offset_dx);
    }
  }

  // 裁剪窗口复制：把 source 的 [sx0,sxn) x [sy0,syn) 区域拷到自身 [dx0,dx1)。
  // 返回是否整窗口在界内。Ruling #6 同款命名纪律：this->size_x 显式引用成员。
  bool copyWindow(const Costmap2D& source, unsigned int sx0, unsigned int sy0,
                  unsigned int sxn, unsigned int syn, unsigned int dx0,
                  unsigned int dy0) {
    if (sx0 > sxn || sy0 > syn || sxn > source.size_x || syn > source.size_y) {
      return false;
    }
    const unsigned int region_size_x = sxn - sx0;
    const unsigned int region_size_y = syn - sy0;
    if (dx0 + region_size_x > this->size_x || dy0 + region_size_y > this->size_y) {
      return false;
    }
    for (unsigned int y = 0; y < region_size_y; ++y) {
      const auto src =
          source.data.begin() + static_cast<std::ptrdiff_t>(
                                     (sy0 + y) * source.size_x + sx0);
      auto dst = data.begin() + static_cast<std::ptrdiff_t>((dy0 + y) * size_x + dx0);
      std::copy(src, src + region_size_x, dst);
    }
    return true;
  }

  // 滚动窗口: 平移地图数据, 使新原点对齐 (Ruling #6 纪律: 成员显式 this->)
  void updateOrigin(double new_origin_x, double new_origin_y) {
    if (size_x == 0 || size_y == 0 || resolution <= 0.0) {
      origin_x = new_origin_x;
      origin_y = new_origin_y;
      return;
    }

    const int cell_ox = static_cast<int>(
        std::floor((new_origin_x - origin_x) / resolution));
    const int cell_oy = static_cast<int>(
        std::floor((new_origin_y - origin_y) / resolution));
    const double aligned_origin_x = origin_x + cell_ox * resolution;
    const double aligned_origin_y = origin_y + cell_oy * resolution;
    if (cell_ox == 0 && cell_oy == 0) {
      return;
    }

    const int old_start_x = std::max(0, cell_ox);
    const int old_start_y = std::max(0, cell_oy);
    const int new_start_x = std::max(0, -cell_ox);
    const int new_start_y = std::max(0, -cell_oy);
    const int cell_size_x = static_cast<int>(size_x) - std::abs(cell_ox);
    const int cell_size_y = static_cast<int>(size_y) - std::abs(cell_oy);

    std::vector<CostValue> new_data(static_cast<std::size_t>(size_x) * size_y,
                                    default_value);
    if (cell_size_x > 0 && cell_size_y > 0) {
      const unsigned int reg_x = static_cast<unsigned int>(cell_size_x);
      const unsigned int reg_y = static_cast<unsigned int>(cell_size_y);
      for (unsigned int y = 0; y < reg_y; ++y) {
        const auto src = data.begin() +
                         static_cast<std::ptrdiff_t>(
                             (static_cast<unsigned int>(old_start_y) + y) * size_x +
                             static_cast<unsigned int>(old_start_x));
        auto dst = new_data.begin() +
                   static_cast<std::ptrdiff_t>(
                       (static_cast<unsigned int>(new_start_y) + y) * size_x +
                       static_cast<unsigned int>(new_start_x));
        std::copy(src, src + reg_x, dst);
      }
    }
    data.swap(new_data);
    origin_x = aligned_origin_x;
    origin_y = aligned_origin_y;
  }

 private:
  unsigned int size_x{0};
  unsigned int size_y{0};
  double resolution{0.0};
  double origin_x{0.0};
  double origin_y{0.0};
  CostValue default_value{FREE_SPACE};
  std::vector<CostValue> data;
};

}  // namespace costmap_2d
