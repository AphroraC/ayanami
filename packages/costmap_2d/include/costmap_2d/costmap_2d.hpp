#pragma once

#include <algorithm>
#include <cstring>
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

  // Explicit this-> member assignment to avoid parameters shadowing members (historical size_x = size_x self-assignment bug)
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
    // numeric_limits<double>::max/lowest for full reinflation.
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
  /// @brief Re-anchor the grid without touching the cell data (the snapshot
  /// restore path: same geometry, new rolling origin).
  void setOrigin(double ox, double oy) {
    this->origin_x = ox;
    this->origin_y = oy;
  }
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

  /// Clipped window copy: copies the [sx0,sxn) x [sy0,syn) region of source into own [dx0,dx1). Returns whether the
  /// whole window is in bounds. Same naming discipline: members referenced explicitly via this->size_x.
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

  /// Rolling window: shifts the map data so the new origin is aligned
  /// (discipline: members referenced via explicit this->). In-place: the
  /// surviving overlap is moved with memmove (per-row horizontal, one
  /// contiguous block vertical) and only the vacated strips are filled with
  /// the default -- no temporary whole-map allocation, O(shift) bytes
  /// touched instead of O(map) allocated + copied.
  /// @return true when a whole-cell shift actually happened (the origin moved
  /// and data was re-laid); a sub-cell request changes nothing and returns
  /// false (the aligned origin only advances with an actual shift).
  bool updateOrigin(double new_origin_x, double new_origin_y) {
    if (this->size_x == 0 || this->size_y == 0 || this->resolution <= 0.0) {
      this->origin_x = new_origin_x;
      this->origin_y = new_origin_y;
      return false;
    }

    const int cell_ox = static_cast<int>(
        std::floor((new_origin_x - this->origin_x) / this->resolution));
    const int cell_oy = static_cast<int>(
        std::floor((new_origin_y - this->origin_y) / this->resolution));
    const double aligned_origin_x = this->origin_x + cell_ox * this->resolution;
    const double aligned_origin_y = this->origin_y + cell_oy * this->resolution;
    if (cell_ox == 0 && cell_oy == 0) {
      return false;
    }

    const unsigned int sx = this->size_x;
    const unsigned int sy = this->size_y;
    const unsigned int shift_x = static_cast<unsigned int>(std::abs(cell_ox));
    const unsigned int shift_y = static_cast<unsigned int>(std::abs(cell_oy));

    // Horizontal: new(x, y) = old(x + cell_ox, y).
    if (cell_ox != 0 && shift_x < sx) {
      for (unsigned int y = 0; y < sy; ++y) {
        CostValue* row = this->data.data() + static_cast<std::size_t>(y) * sx;
        if (cell_ox > 0) {
          std::memmove(row, row + shift_x,
                       static_cast<std::size_t>(sx - shift_x) * sizeof(CostValue));
        } else {
          std::memmove(row + shift_x, row,
                       static_cast<std::size_t>(sx - shift_x) * sizeof(CostValue));
        }
      }
    }

    // Vertical: new(x, y) = shifted(x, y + cell_oy); row-major rows are one
    // contiguous block.
    if (cell_oy != 0 && shift_y < sy) {
      if (cell_oy > 0) {
        std::memmove(this->data.data(),
                     this->data.data() + static_cast<std::size_t>(shift_y) * sx,
                     static_cast<std::size_t>(sy - shift_y) * sx * sizeof(CostValue));
      } else {
        std::memmove(this->data.data() + static_cast<std::size_t>(shift_y) * sx,
                     this->data.data(),
                     static_cast<std::size_t>(sy - shift_y) * sx * sizeof(CostValue));
      }
    }

    // Fill the vacated strips with the default. The y-strip fill runs last
    // and covers full rows, so any x-strip work inside vacated rows is
    // subsumed; a shift at least the map dimension vacates everything.
    const auto fill_strip_x = [&](unsigned int x0, unsigned int x1) {
      if (x0 >= x1) {
        return;
      }
      for (unsigned int y = 0; y < sy; ++y) {
        std::fill(this->data.begin() + static_cast<std::ptrdiff_t>(y * sx + x0),
                  this->data.begin() + static_cast<std::ptrdiff_t>(y * sx + x1),
                  this->default_value);
      }
    };
    const auto fill_strip_y = [&](unsigned int y0, unsigned int y1) {
      if (y0 >= y1) {
        return;
      }
      std::fill(this->data.begin() + static_cast<std::ptrdiff_t>(y0) * sx,
                this->data.begin() + static_cast<std::ptrdiff_t>(y1) * sx,
                this->default_value);
    };
    if (shift_x >= sx) {
      this->resetMap();
    } else {
      if (cell_ox > 0) {
        fill_strip_x(sx - shift_x, sx);
      } else if (cell_ox < 0) {
        fill_strip_x(0, shift_x);
      }
      if (cell_oy > 0) {
        fill_strip_y(sy - shift_y, sy);
      } else if (cell_oy < 0) {
        fill_strip_y(0, shift_y);
      }
    }

    this->origin_x = aligned_origin_x;
    this->origin_y = aligned_origin_y;
    return true;
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
