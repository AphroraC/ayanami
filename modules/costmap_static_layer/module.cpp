#include "costmap_static_layer_plugin.hpp"

#include <ayanami/utility/extension_module.hpp>
#include <ayanami/utility/logging.hpp>

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
    // Map-server semantics from the map json (all keys optional; defaults
    // match map_server: negate=false, 0.65/0.196, trinary).
    costmap_2d_layers::StaticMapSemantics semantics;
    semantics.negate =
        map_cfg.param<int>("map", "negate", 0) != 0;
    semantics.occupied_thresh = map_cfg.param<double>(
        "map", "occupied_thresh", semantics.occupied_thresh);
    semantics.free_thresh =
        map_cfg.param<double>("map", "free_thresh", semantics.free_thresh);
    semantics.trinary = map_cfg.param<std::string>("map", "mode", "trinary") !=
                        "scale";
    // The between-threshold band resolves per the layered costmap's
    // track_unknown policy (the layer does not own that decision).
    const auto* layered = impl_->layeredCostmap();
    semantics.track_unknown = layered ? layered->isTrackingUnknown() : false;
    // Runs from onAddedToMap: the master is set, so processMap sizes it to
    // the map geometry.
    impl_->setMapPath(map_path, *map_resolution, map_origin->x(), map_origin->y(), semantics);
  } catch (const std::exception& e) {
    SPDLOG_LOGGER_ERROR(ayanami::costmapLogger(), "CostmapStaticLayerPlugin: {}", e.what());
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
