#include <costmap_2d_layers/static_layer.hpp>
#include <costmap_2d/layered_costmap.hpp>

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
  std::lock_guard<std::mutex> lock(map_mutex);
  processMap(map);
}

void StaticLayer::queueMap(const costmap_2d::Costmap2D& map) {
  std::lock_guard<std::mutex> lock(map_mutex);
  pending_map = map;
  current = false;
}

void StaticLayer::processMap(const costmap_2d::Costmap2D& map) {
  if (master && !master->isRolling()) {
    auto* master_grid = master->getCostmap();
    if (master_grid->getSizeInCellsX() != map.getSizeInCellsX() ||
        master_grid->getSizeInCellsY() != map.getSizeInCellsY() ||
        master_grid->getResolution() != map.getResolution() ||
        master_grid->getOriginX() != map.getOriginX() ||
        master_grid->getOriginY() != map.getOriginY() ||
        !master->isSizeLocked()) {
      master->resizeMap(map.getSizeInCellsX(), map.getSizeInCellsY(),
                         map.getResolution(), map.getOriginX(),
                         map.getOriginY(), true);
    }
  }
  this->map.resizeMap(map.getSizeInCellsX(), map.getSizeInCellsY(),
                      map.getResolution(), map.getOriginX(), map.getOriginY(),
                      map.getDefaultValue());
  for (unsigned int y = 0; y < map.getSizeInCellsY(); ++y) {
    for (unsigned int x = 0; x < map.getSizeInCellsX(); ++x) {
      this->map.setCost(x, y, map.getCost(x, y));
    }
  }
  has_map = true;
  has_updated_data = true;
  current = true;
  full_update = true;
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

  costmap_2d::Costmap2D loaded_map(
      width, height, resolution, origin_x, origin_y,
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
      const unsigned int image_y = static_cast<unsigned int>(i / width);
      const unsigned int my = height - image_y - 1;
      loaded_map.setCost(mx, my, pixelToCost(pixel));
    }
  } else {  // P2 ASCII
    for (std::size_t i = 0; i < pixel_count; ++i) {
      const unsigned int value =
          static_cast<unsigned int>(std::stoul(readToken(input)));
      const unsigned char pixel = static_cast<unsigned char>(
          std::min(value, max_value) * 255.0 / max_value);
      const unsigned int mx = static_cast<unsigned int>(i % width);
      const unsigned int image_y = static_cast<unsigned int>(i / width);
      const unsigned int my = height - image_y - 1;
      loaded_map.setCost(mx, my, pixelToCost(pixel));
    }
  }

  processMap(loaded_map);
}

void StaticLayer::matchSize() {
  if (!master || master->isRolling()) return;
  const auto* master_grid = master->getCostmap();
  if (map.getSizeInCellsX() != master_grid->getSizeInCellsX() ||
      map.getSizeInCellsY() != master_grid->getSizeInCellsY() ||
      map.getResolution() != master_grid->getResolution() ||
      map.getOriginX() != master_grid->getOriginX() ||
      map.getOriginY() != master_grid->getOriginY()) {
    map.resizeMap(master_grid->getSizeInCellsX(),
                   master_grid->getSizeInCellsY(),
                   master_grid->getResolution(), master_grid->getOriginX(),
                   master_grid->getOriginY(), costmap_2d::NO_INFORMATION);
  }
}

void StaticLayer::addUpdateBounds(double min_x, double min_y, double max_x,
                                  double max_y) {
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

void StaticLayer::updateBounds(double, double, double, double* min_x,
                               double* min_y, double* max_x, double* max_y) {
  {
    std::lock_guard<std::mutex> lock(map_mutex);
    if (pending_map) {
      processMap(*pending_map);
      pending_map.reset();
    }
  }
  if (!enabled || !has_map) {
    map_received_in_update_bounds = false;
    return;
  }
  map_received_in_update_bounds = true;
  if (master && !master->isRolling() && !has_updated_data &&
      !has_extra_bounds && !full_update) {
    return;
  }
  if (full_update) {
    *min_x = std::min(*min_x, map.getOriginX());
    *min_y = std::min(*min_y, map.getOriginY());
    *max_x = std::max(*max_x, map.getOriginX() + map.getSizeInMetersX());
    *max_y = std::max(*max_y, map.getOriginY() + map.getSizeInMetersY());
    full_update = false;
  }
  if (master && master->isRolling()) {
    const auto* master_grid = master->getCostmap();
    const double origin_x = master_grid->getOriginX();
    const double origin_y = master_grid->getOriginY();
    if (!have_master_origin || origin_x != last_master_origin_x ||
        origin_y != last_master_origin_y) {
      *min_x = std::min(*min_x, origin_x);
      *min_y = std::min(*min_y, origin_y);
      *max_x = std::max(*max_x, origin_x + master_grid->getSizeInMetersX());
      *max_y = std::max(*max_y, origin_y + master_grid->getSizeInMetersY());
      last_master_origin_x = origin_x;
      last_master_origin_y = origin_y;
      have_master_origin = true;
    }
  }
  if (has_extra_bounds) {
    *min_x = std::min(*min_x, extra_min_x);
    *min_y = std::min(*min_y, extra_min_y);
    *max_x = std::max(*max_x, extra_max_x);
    *max_y = std::max(*max_y, extra_max_y);
    has_extra_bounds = false;
  }
  has_updated_data = false;
}

void StaticLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i,
                              int min_j, int max_i, int max_j) {
  if (!enabled || !has_map ||
      (master && !map_received_in_update_bounds)) {
    return;
  }
  const int end_x = std::min(max_i, static_cast<int>(master_grid.getSizeInCellsX()));
  const int end_y = std::min(max_j, static_cast<int>(master_grid.getSizeInCellsY()));
  const bool same_geometry =
      master_grid.getSizeInCellsX() == map.getSizeInCellsX() &&
      master_grid.getSizeInCellsY() == map.getSizeInCellsY() &&
      master_grid.getResolution() == map.getResolution() &&
      master_grid.getOriginX() == map.getOriginX() &&
      master_grid.getOriginY() == map.getOriginY();
  for (int y = std::max(0, min_j); y < end_y; ++y) {
    for (int x = std::max(0, min_i); x < end_x; ++x) {
      if (same_geometry) {
        master_grid.setCost(static_cast<unsigned int>(x),
                            static_cast<unsigned int>(y),
                            map.getCost(static_cast<unsigned int>(x),
                                         static_cast<unsigned int>(y)));
        continue;
      }
      // rolling global costmap (M2) 下 master
      // grid 原点随机器人滚动, 不能按相同 cell 索引直拷; 改用 cell 中心世界
      // 坐标在静态图上采样, 让窗口内正确渲染地图数据。
      double wx = 0.0;
      double wy = 0.0;
      master_grid.mapToWorld(static_cast<unsigned int>(x),
                             static_cast<unsigned int>(y), wx, wy);
      unsigned int mx = 0;
      unsigned int my = 0;
      if (map.worldToMap(wx, wy, mx, my)) {
        master_grid.setCost(static_cast<unsigned int>(x),
                            static_cast<unsigned int>(y),
                            map.getCost(mx, my));
      }
    }
  }
  current = true;
}

}  // namespace costmap_2d_layers
