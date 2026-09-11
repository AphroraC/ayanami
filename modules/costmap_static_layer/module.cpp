#include "costmap_static_layer_plugin.hpp"

#include <ayanami/utility/extension_module.hpp>

#include <Eigen/Geometry>

namespace ayanami {
namespace modules {

// Out-of-line key function (see the header note): pins the class vtable.
CostmapStaticLayerPlugin::~CostmapStaticLayerPlugin() = default;

void CostmapStaticLayerPlugin::configureParameters(
    const ayanami::ModuleContext& context) {
  const Config& file = context.config;
  const std::string& section = context.name;
  param_.static_map =
      file.param<std::string>(section, "static_map", param_.static_map);
  param_.fallback_size_x =
      file.param<int>(section, "size_x", param_.fallback_size_x);
  param_.fallback_size_y =
      file.param<int>(section, "size_y", param_.fallback_size_y);
  param_.fallback_resolution =
      file.param<double>(section, "resolution", param_.fallback_resolution);
}

void CostmapStaticLayerPlugin::loadMap() {
  if (param_.static_map.empty()) {
    return;
  }
  try {
    // A relative static_map (config/assets/nav.pgm) resolves against the
    // PARENT of the config directory (the deployment root), the same
    // semantics configureCoreMaps always used; no dependency on the node
    // cwd (~/.ros under roslaunch).
    std::string map_path = param_.static_map;
    if (map_path.front() != '/' && map_path.find("://") == std::string::npos) {
      const std::string directory =
          ayanami::GlobalConfig::instance()
              ? ayanami::GlobalConfig::instance()->param<std::string>(
                    "global", "config_path", ".")
              : ".";
      map_path = (std::filesystem::path(directory).parent_path() / map_path)
                     .string();
    }
    // Ayanami map metadata is JSON-only and loaded through the common
    // Config API (<map>.json next to the pgm).
    std::filesystem::path metadata_path(map_path);
    metadata_path.replace_extension(".json");
    Config map_cfg(metadata_path.string());
    const auto map_resolution = map_cfg.param<double>("map", "resolution");
    const auto map_origin = map_cfg.param<Eigen::Vector3d>("map", "origin");
    if (!map_resolution || !map_origin) {
      throw std::runtime_error("invalid JSON map metadata: " +
                               metadata_path.string());
    }
    // Runs from onAddedToMap: the master is set, so processMap sizes it to
    // the map geometry.
    impl_->setMapPath(map_path, *map_resolution, map_origin->x(),
                      map_origin->y());
  } catch (const std::exception& e) {
    spdlog::error("CostmapStaticLayerPlugin: {}", e.what());
  }
}

void CostmapStaticLayerPlugin::onAddedToMap(
    costmap_2d::LayeredCostmap& layered_costmap) {
  loadMap();
  if (impl_->hasMap()) {
    return;
  }
  if (param_.fallback_size_x > 0 && param_.fallback_size_y > 0) {
    layered_costmap.resizeMap(
        static_cast<unsigned int>(param_.fallback_size_x),
        static_cast<unsigned int>(param_.fallback_size_y),
        param_.fallback_resolution, 0.0, 0.0);
  } else {
    throw std::runtime_error(
        "static global map failed to load and no fallback size is "
        "configured");
  }
}

}  // namespace modules
}  // namespace ayanami

extern "C" ayanami::ExtensionModule* create_extension_module() {
  return new ayanami::modules::CostmapStaticLayerPlugin();
}
