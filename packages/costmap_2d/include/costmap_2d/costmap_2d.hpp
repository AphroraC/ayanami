#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
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
    this->size_x_ = size_x;
    this->size_y_ = size_y;
    this->resolution_ = resolution;
    this->origin_x_ = origin_x;
    this->origin_y_ = origin_y;
    this->default_value_ = default_value;
    data_.assign(static_cast<std::size_t>(size_x_) * size_y_, default_value_);
  }

  void resetMap() { std::fill(data_.begin(), data_.end(), default_value_); }

  void resetMap(unsigned int x0, unsigned int y0, unsigned int xn,
                unsigned int yn) {
    resetMapToValue(x0, y0, xn, yn, default_value_);
  }

  void resetMapToValue(unsigned int x0, unsigned int y0, unsigned int xn,
                       unsigned int yn, CostValue value) {
    x0 = std::min(x0, size_x_);
    y0 = std::min(y0, size_y_);
    xn = std::min(xn, size_x_);
    yn = std::min(yn, size_y_);
    if (x0 >= xn || y0 >= yn) {
      return;
    }
    for (unsigned int y = y0; y < yn; ++y) {
      std::fill(data_.begin() + getIndex(x0, y),
                data_.begin() + getIndex(xn, y), value);
    }
  }

  CostValue getCost(unsigned int mx, unsigned int my) const {
    return data_[getIndex(mx, my)];
  }

  CostValue getCost(unsigned int index) const { return data_[index]; }

  void setCost(unsigned int mx, unsigned int my, CostValue cost) {
    data_[getIndex(mx, my)] = cost;
  }

  bool worldToMap(double wx, double wy, unsigned int& mx,
                  unsigned int& my) const {
    if (wx < origin_x_ || wy < origin_y_ || resolution_ <= 0.0) {
      return false;
    }
    mx = static_cast<unsigned int>((wx - origin_x_) / resolution_);
    my = static_cast<unsigned int>((wy - origin_y_) / resolution_);
    return mx < size_x_ && my < size_y_;
  }

  void worldToMapNoBounds(double wx, double wy, int& mx, int& my) const {
    mx = static_cast<int>(std::floor((wx - origin_x_) / resolution_));
    my = static_cast<int>(std::floor((wy - origin_y_) / resolution_));
  }

  void worldToMapEnforceBounds(double wx, double wy, int& mx, int& my) const {
    if (size_x_ == 0 || size_y_ == 0 || resolution_ <= 0.0) {
      mx = 0;
      my = 0;
      return;
    }
    worldToMapNoBounds(wx, wy, mx, my);
    mx = std::clamp(mx, 0, static_cast<int>(size_x_) - 1);
    my = std::clamp(my, 0, static_cast<int>(size_y_) - 1);
  }

  void mapToWorld(unsigned int mx, unsigned int my, double& wx,
                  double& wy) const {
    wx = origin_x_ + (static_cast<double>(mx) + 0.5) * resolution_;
    wy = origin_y_ + (static_cast<double>(my) + 0.5) * resolution_;
  }

  unsigned int getIndex(unsigned int mx, unsigned int my) const {
    return my * size_x_ + mx;
  }

  void indexToCells(unsigned int index, unsigned int& mx,
                    unsigned int& my) const {
    my = index / size_x_;
    mx = index - my * size_x_;
  }

  unsigned char* getCharMap() {
    return reinterpret_cast<unsigned char*>(data_.data());
  }

  const unsigned char* getCharMap() const {
    return reinterpret_cast<const unsigned char*>(data_.data());
  }

  unsigned int getSizeInCellsX() const { return size_x_; }
  unsigned int getSizeInCellsY() const { return size_y_; }
  double getSizeInMetersX() const { return size_x_ * resolution_; }
  double getSizeInMetersY() const { return size_y_ * resolution_; }
  double getResolution() const { return resolution_; }
  double getOriginX() const { return origin_x_; }
  double getOriginY() const { return origin_y_; }
  CostValue getDefaultValue() const { return default_value_; }
  void setDefaultValue(CostValue value) { default_value_ = value; }

  unsigned int cellDistance(double world_dist) const {
    if (resolution_ <= 0.0 || world_dist <= 0.0) {
      return 0;
    }
    return static_cast<unsigned int>(std::ceil(world_dist / resolution_));
  }

  bool inBounds(unsigned int mx, unsigned int my) const {
    return mx < size_x_ && my < size_y_;
  }

  // 裁剪窗口复制：把 source 的 [sx0,sxn) x [sy0,syn) 区域拷到自身 [dx0,dx1)。
  // 返回是否整窗口在界内。Ruling #6 同款命名纪律：this->size_x_ 显式引用成员。
  bool copyWindow(const Costmap2D& source, unsigned int sx0, unsigned int sy0,
                  unsigned int sxn, unsigned int syn, unsigned int dx0,
                  unsigned int dy0) {
    if (sx0 > sxn || sy0 > syn || sxn > source.size_x_ || syn > source.size_y_) {
      return false;
    }
    const unsigned int region_size_x = sxn - sx0;
    const unsigned int region_size_y = syn - sy0;
    if (dx0 + region_size_x > this->size_x_ || dy0 + region_size_y > this->size_y_) {
      return false;
    }
    for (unsigned int y = 0; y < region_size_y; ++y) {
      const auto src =
          source.data_.begin() + static_cast<std::ptrdiff_t>(
                                     (sy0 + y) * source.size_x_ + sx0);
      auto dst = data_.begin() + static_cast<std::ptrdiff_t>((dy0 + y) * size_x_ + dx0);
      std::copy(src, src + region_size_x, dst);
    }
    return true;
  }

  // 滚动窗口: 平移地图数据, 使新原点对齐 (Ruling #6 纪律: 成员显式 this->)
  void updateOrigin(double new_origin_x, double new_origin_y) {
    if (size_x_ == 0 || size_y_ == 0 || resolution_ <= 0.0) {
      origin_x_ = new_origin_x;
      origin_y_ = new_origin_y;
      return;
    }

    const int cell_ox = static_cast<int>(
        std::floor((new_origin_x - origin_x_) / resolution_));
    const int cell_oy = static_cast<int>(
        std::floor((new_origin_y - origin_y_) / resolution_));
    if (cell_ox == 0 && cell_oy == 0) {
      origin_x_ = new_origin_x;
      origin_y_ = new_origin_y;
      return;
    }

    const int old_start_x = std::max(0, cell_ox);
    const int old_start_y = std::max(0, cell_oy);
    const int new_start_x = std::max(0, -cell_ox);
    const int new_start_y = std::max(0, -cell_oy);
    const int cell_size_x = static_cast<int>(size_x_) - std::abs(cell_ox);
    const int cell_size_y = static_cast<int>(size_y_) - std::abs(cell_oy);

    std::vector<CostValue> new_data(static_cast<std::size_t>(size_x_) * size_y_,
                                    default_value_);
    if (cell_size_x > 0 && cell_size_y > 0) {
      const unsigned int reg_x = static_cast<unsigned int>(cell_size_x);
      const unsigned int reg_y = static_cast<unsigned int>(cell_size_y);
      for (unsigned int y = 0; y < reg_y; ++y) {
        const auto src = data_.begin() +
                         static_cast<std::ptrdiff_t>(
                             (static_cast<unsigned int>(old_start_y) + y) * size_x_ +
                             static_cast<unsigned int>(old_start_x));
        auto dst = new_data.begin() +
                   static_cast<std::ptrdiff_t>(
                       (static_cast<unsigned int>(new_start_y) + y) * size_x_ +
                       static_cast<unsigned int>(new_start_x));
        std::copy(src, src + reg_x, dst);
      }
    }
    data_.swap(new_data);
    origin_x_ = new_origin_x;
    origin_y_ = new_origin_y;
  }

 private:
  unsigned int size_x_{0};
  unsigned int size_y_{0};
  double resolution_{0.0};
  double origin_x_{0.0};
  double origin_y_{0.0};
  CostValue default_value_{FREE_SPACE};
  std::vector<CostValue> data_;
};

}  // namespace costmap_2d
