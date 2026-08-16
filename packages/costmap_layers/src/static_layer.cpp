#include <costmap_2d_layers/static_layer.hpp>

#include <algorithm>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>

namespace costmap_2d_layers {

namespace {

// PGM 头 token 读取: 跳过空行与 '#' 注释行。
std::string readToken(std::istream& in) {
  std::string token;
  while (in >> token) {
    if (token.empty() || token[0] != '#') {
      return token;
    }
    in.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  }
  return token;
}

costmap_2d::CostValue pixelToCost(unsigned char pixel) {
  if (pixel == 0) {
    return costmap_2d::LETHAL_OBSTACLE;
  }
  if (pixel >= 254) {
    return costmap_2d::FREE_SPACE;
  }
  return costmap_2d::NO_INFORMATION;
}

}  // namespace

void StaticLayer::setMap(const costmap_2d::Costmap2D& map) {
  map_.resizeMap(map.getSizeInCellsX(), map.getSizeInCellsY(),
                 map.getResolution(), map.getOriginX(), map.getOriginY(),
                 map.getDefaultValue());
  for (unsigned int y = 0; y < map.getSizeInCellsY(); ++y) {
    for (unsigned int x = 0; x < map.getSizeInCellsX(); ++x) {
      map_.setCost(x, y, map.getCost(x, y));
    }
  }
  has_map_ = true;
  current_ = true;
}

void StaticLayer::setMapPath(const std::string& path, double resolution,
                             double origin_x, double origin_y) {
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    throw std::runtime_error("failed to open PGM map: " + path);
  }

  const std::string magic = readToken(input);
  if (magic != "P5" && magic != "P2") {
    throw std::runtime_error("only PGM P5/P2 images are supported: " + path);
  }

  const unsigned int width =
      static_cast<unsigned int>(std::stoul(readToken(input)));
  const unsigned int height =
      static_cast<unsigned int>(std::stoul(readToken(input)));
  const unsigned int max_value =
      static_cast<unsigned int>(std::stoul(readToken(input)));
  if (width == 0 || height == 0 || max_value == 0 || max_value > 65535) {
    throw std::runtime_error("invalid PGM header: " + path);
  }

  map_.resizeMap(width, height, resolution, origin_x, origin_y,
                 costmap_2d::NO_INFORMATION);

  const std::size_t pixel_count =
      static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
  if (magic == "P5") {
    input.get();  // 吃掉 maxval 后的单个空白符
    for (std::size_t i = 0; i < pixel_count; ++i) {
      unsigned int value = 0;
      if (max_value < 256) {
        const int v = input.get();
        if (v == std::char_traits<char>::eof()) {
          throw std::runtime_error("truncated PGM image: " + path);
        }
        value = static_cast<unsigned int>(v);
      } else {
        const int high = input.get();
        const int low = input.get();
        if (high == std::char_traits<char>::eof() ||
            low == std::char_traits<char>::eof()) {
          throw std::runtime_error("truncated PGM image: " + path);
        }
        value = static_cast<unsigned int>((high << 8) | low);
      }
      const unsigned char pixel = static_cast<unsigned char>(
          std::min(value, max_value) * 255.0 / max_value);
      const unsigned int mx = static_cast<unsigned int>(i % width);
      const unsigned int my = static_cast<unsigned int>(i / width);
      map_.setCost(mx, my, pixelToCost(pixel));
    }
  } else {  // P2 ASCII
    for (std::size_t i = 0; i < pixel_count; ++i) {
      const unsigned int value =
          static_cast<unsigned int>(std::stoul(readToken(input)));
      const unsigned char pixel = static_cast<unsigned char>(
          std::min(value, max_value) * 255.0 / max_value);
      const unsigned int mx = static_cast<unsigned int>(i % width);
      const unsigned int my = static_cast<unsigned int>(i / width);
      map_.setCost(mx, my, pixelToCost(pixel));
    }
  }

  has_map_ = true;
  current_ = true;
}

void StaticLayer::updateBounds(double, double, double, double* min_x,
                               double* min_y, double* max_x, double* max_y) {
  if (!enabled_ || !has_map_) {
    return;
  }
  *min_x = std::min(*min_x, map_.getOriginX());
  *min_y = std::min(*min_y, map_.getOriginY());
  *max_x = std::max(*max_x, map_.getOriginX() + map_.getSizeInMetersX());
  *max_y = std::max(*max_y, map_.getOriginY() + map_.getSizeInMetersY());
}

void StaticLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i,
                              int min_j, int max_i, int max_j) {
  if (!enabled_ || !has_map_) {
    return;
  }
  const int end_x = std::min(max_i, static_cast<int>(master_grid.getSizeInCellsX()));
  const int end_y = std::min(max_j, static_cast<int>(master_grid.getSizeInCellsY()));
  for (int y = std::max(0, min_j); y < end_y; ++y) {
    for (int x = std::max(0, min_i); x < end_x; ++x) {
      // Task 12 stage C: rolling global costmap (M2 Ruling #M2-4) 下 master
      // grid 原点随机器人滚动, 不能按相同 cell 索引直拷; 改用 cell 中心世界
      // 坐标在静态图上采样, 让窗口内正确渲染地图数据。
      double wx = 0.0;
      double wy = 0.0;
      master_grid.mapToWorld(static_cast<unsigned int>(x),
                             static_cast<unsigned int>(y), wx, wy);
      unsigned int mx = 0;
      unsigned int my = 0;
      if (map_.worldToMap(wx, wy, mx, my)) {
        master_grid.setCost(static_cast<unsigned int>(x),
                            static_cast<unsigned int>(y),
                            map_.getCost(mx, my));
      } else {
        master_grid.setCost(static_cast<unsigned int>(x),
                            static_cast<unsigned int>(y),
                            costmap_2d::FREE_SPACE);
      }
    }
  }
}

}  // namespace costmap_2d_layers
