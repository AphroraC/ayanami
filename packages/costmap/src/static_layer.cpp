#include <costmap/static_layer.hpp>
#include <costmap/layered_costmap.hpp>

#include <algorithm>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>

namespace costmap_2d {

namespace {

// PGM header token reading: skips blank lines and '#' comment lines.
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

/// @brief Map-server pixel interpretation: occupancy ratio is negate-aware
/// (standard maps are black-occupied), thresholds and the between-band
/// resolution follow the configured semantics. The previous hardcoding
/// (0 -> lethal, >=254 -> free, everything else -> unknown) silently
/// declared every antialiased wall edge traversable-unknown regardless of
/// the map json's declared mode/thresholds.
costmap_2d::CostValue pixelToCost(unsigned char pixel,
                                  const StaticMapSemantics& semantics) {
  const double ratio_raw = static_cast<double>(pixel) / 255.0;
  const double occupancy =
      semantics.negate ? ratio_raw : (1.0 - ratio_raw);
  if (occupancy > semantics.occupied_thresh) {
    return costmap_2d::LETHAL_OBSTACLE;
  }
  if (occupancy < semantics.free_thresh) {
    return costmap_2d::FREE_SPACE;
  }
  if (semantics.trinary) {
    return semantics.track_unknown ? costmap_2d::NO_INFORMATION
                                   : costmap_2d::FREE_SPACE;
  }
  // Scale mode: grayscale occupancy linearly to cost (saturating below
  // LETHAL: the cell is below the occupied threshold by construction).
  const double scaled =
      std::round(occupancy * static_cast<double>(costmap_2d::LETHAL_OBSTACLE));
  return static_cast<costmap_2d::CostValue>(
      std::min(scaled, static_cast<double>(costmap_2d::LETHAL_OBSTACLE) - 1.0));
}

}  // namespace

void StaticLayer::setMap(const costmap_2d::Costmap2D& map) {
  std::lock_guard<std::mutex> lock(map_mutex);
  processMap(map);
}

void StaticLayer::queueMap(const costmap_2d::Costmap2D& map) {
  std::lock_guard<std::mutex> lock(map_mutex);
  pending_map = map;
  data_current = false;
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
  data_current = true;
  full_update = true;
}

void StaticLayer::setMapPath(const std::string& path, double resolution,
                             double origin_x, double origin_y,
                             const StaticMapSemantics& semantics) {
  this->semantics = semantics;
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
    input.get();  // Consume the single whitespace after maxval
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
      loaded_map.setCost(mx, my, pixelToCost(pixel, semantics));
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
      loaded_map.setCost(mx, my, pixelToCost(pixel, semantics));
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
  if (master && !master->isRolling() && !has_updated_data && !full_update &&
      !useExtraBounds(min_x, min_y, max_x, max_y)) {
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
  useExtraBounds(min_x, min_y, max_x, max_y);
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
      // Under a rolling global costmap (M2) the master grid origin
      // rolls with the robot, so a direct copy by identical cell index is invalid;
      // instead sample the static map at the cell center world coordinates so the window renders map data correctly.
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
  data_current = true;
}

}  // namespace costmap_2d
