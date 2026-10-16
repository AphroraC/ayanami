#pragma once

#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <costmap/costmap_wrapper.hpp>
#include <costmap/inflation_layer.hpp>
#include <costmap/simple_obstacle_layer.hpp>
#include <costmap/static_layer.hpp>

#include "ayanami/costmap/components/costmap_obstacle_applier.hpp"
#include "ayanami/costmap/core/costmap_observation.hpp"
#include "ayanami/utility/config.hpp"
#include "ayanami/utility/logging.hpp"

namespace ayanami {
namespace costmap {

/**
 * @brief Static layer composition (the successor of the former dlopen
 * layer-plugin family): builds the two FIXED map compositions from the
 * navigation route json and returns the layer handles the writer needs.
 *
 * - global map: StaticLayer + InflationLayer (static content only);
 * - local map: SimpleObstacleLayer + InflationLayer (rolling, dynamic
 *   marks) + the parsed observation-source list + the observation applier.
 *
 * Layer parameters live in the route json's embedded blocks
 * (costmap_global.static_layer / .inflation_layer,
 * costmap_local.obstacle_layer / .inflation_layer -- the INLINE spelling the
 * plugin era's production config already used), so existing deployments
 * keep working after deleting the per-layer "so_name" keys. The "layers"
 * list is no longer read: the composition is compile-time fixed.
 */
struct CostmapLayerAssembly {
  costmap_2d::StaticLayer::Ptr static_layer;            // global only
  costmap_2d::SimpleObstacleLayer::Ptr obstacle_layer;  // local only
  costmap_2d::InflationLayer::Ptr inflation_layer;
  CostmapObstacleApplier::Ptr obstacle_applier;  // local only
  /// Observation sources consumed by the obstacle layer (name/kind +
  /// filtering windows + liveness rate), one source per sensor kind.
  std::vector<ObservationSourceConfig> observation_sources;
};

/// @brief One map's inflation block -> setters (shared by both maps).
inline void configureInflation(const Config& route_cfg,
                               const std::string& map_section,
                               costmap_2d::InflationLayer& inflation) {
  const Config cfg = route_cfg.child(map_section, "inflation_layer");
  inflation.setInflationRadius(
      cfg.param<double>("inflation_layer", "inflation_radius", 0.5));
  inflation.setInscribedRadius(
      cfg.param<double>("inflation_layer", "inscribed_radius", 0.12));
  inflation.setCostScalingFactor(
      cfg.param<double>("inflation_layer", "cost_scaling_factor", 3.0));
}

/// @brief Build the global map's layers: StaticLayer + InflationLayer.
///
/// Also performs the static plugin's former onAddedToMap work: resolving the
/// static_map path (relative paths resolve against the deployment root =
/// the parent of the config directory), loading the pgm + <pgm>.json
/// metadata (map-server semantics), and sizing the master from the map -- or
/// from the fallback size_x/size_y/resolution when no map file is deployed
/// (a missing fallback size is a startup error). MUST be called after the
/// two layers were added to the LayeredCostmap (the master geometry is
/// sized here).
inline CostmapLayerAssembly buildGlobalLayers(
    const Config& route_cfg, costmap_2d::LayeredCostmap& lm) {
  CostmapLayerAssembly assembly;

  assembly.static_layer = std::make_shared<costmap_2d::StaticLayer>();
  lm.addLayer(assembly.static_layer);

  assembly.inflation_layer = std::make_shared<costmap_2d::InflationLayer>();
  configureInflation(route_cfg, "costmap_global", *assembly.inflation_layer);
  lm.addLayer(assembly.inflation_layer);

  // ---- static map load + master sizing (former plugin onAddedToMap) ----
  const Config static_cfg = route_cfg.child("costmap_global", "static_layer");
  const std::string static_map =
      static_cfg.param<std::string>("static_layer", "static_map", "");
  bool loaded = false;
  if (!static_map.empty()) {
    try {
      // A relative static_map (config/assets/nav.pgm) resolves against the
      // PARENT of the config directory (the deployment root), the same
      // semantics the plugin era used; no dependency on the node cwd.
      std::string map_path = static_map;
      if (map_path.front() != '/' &&
          map_path.find("://") == std::string::npos) {
        const std::string directory =
            ayanami::GlobalConfig::instance()
                ? ayanami::GlobalConfig::instance()->param<std::string>(
                      "global", "config_path", ".")
                : ".";
        map_path =
            (std::filesystem::path(directory).parent_path() / map_path)
                .string();
      }
      // Ayanami map metadata is JSON-only (<map>.json next to the pgm).
      std::filesystem::path metadata_path(map_path);
      metadata_path.replace_extension(".json");
      Config map_json(metadata_path.string());
      const auto map_resolution = map_json.param<double>("map", "resolution");
      const auto map_origin = map_json.param<Eigen::Vector3d>("map", "origin");
      if (!map_resolution || !map_origin) {
        throw std::runtime_error("invalid JSON map metadata: " +
                                 metadata_path.string());
      }
      costmap_2d::StaticMapSemantics semantics;
      semantics.negate = map_json.param<int>("map", "negate", 0) != 0;
      semantics.occupied_thresh = map_json.param<double>(
          "map", "occupied_thresh", semantics.occupied_thresh);
      semantics.free_thresh = map_json.param<double>("map", "free_thresh",
                                                     semantics.free_thresh);
      semantics.trinary =
          map_json.param<std::string>("map", "mode", "trinary") != "scale";
      // The between-threshold band resolves per the layered costmap's
      // track_unknown policy (the layer does not own that decision).
      semantics.track_unknown = lm.isTrackingUnknown();
      // The master is set (addLayer ran), so processMap sizes it.
      assembly.static_layer->setMapPath(map_path, *map_resolution,
                                        map_origin->x(), map_origin->y(),
                                        semantics);
      loaded = assembly.static_layer->hasMap();
    } catch (const std::exception& e) {
      SPDLOG_LOGGER_ERROR(ayanami::costmapLogger(),
                          "buildGlobalLayers: {}", e.what());
    }
  }
  if (!loaded) {
    const int size_x = static_cfg.param<int>("static_layer", "size_x", 0);
    const int size_y = static_cfg.param<int>("static_layer", "size_y", 0);
    const double resolution =
        static_cfg.param<double>("static_layer", "resolution", 0.05);
    if (size_x > 0 && size_y > 0) {
      lm.resizeMap(static_cast<unsigned int>(size_x),
                   static_cast<unsigned int>(size_y), resolution, 0.0, 0.0);
    } else {
      throw std::runtime_error(
          "static global map failed to load and no fallback size is "
          "configured");
    }
  }
  return assembly;
}

/// @brief Build the local map's layers: SimpleObstacleLayer + InflationLayer.
///
/// Parses the obstacle layer's own parameter block (combination policy,
/// decay, footprint clearing) and its observation_sources list + per-source
/// child blocks (one source per sensor kind -- a duplicate kind is a config
/// error, matching the plugin-era rule). The caller adds the returned
/// obstacle/inflation layers to its LayeredCostmap in that order.
inline CostmapLayerAssembly buildLocalLayers(const Config& route_cfg) {
  CostmapLayerAssembly assembly;
  const Config obstacle_cfg = route_cfg.child("costmap_local", "obstacle_layer");

  assembly.obstacle_layer =
      std::make_shared<costmap_2d::SimpleObstacleLayer>();
  const std::string combination = obstacle_cfg.param<std::string>(
      "obstacle_layer", "combination", "maximum");
  if (combination == "overwrite") {
    assembly.obstacle_layer->setCombinationMethod(
        costmap_2d::SimpleObstacleLayer::CombinationMethod::OVERWRITE);
  } else if (combination == "none") {
    assembly.obstacle_layer->setCombinationMethod(
        costmap_2d::SimpleObstacleLayer::CombinationMethod::NONE);
  } else {
    assembly.obstacle_layer->setCombinationMethod(
        costmap_2d::SimpleObstacleLayer::CombinationMethod::MAXIMUM);
  }
  assembly.obstacle_layer->setDecayTime(
      obstacle_cfg.param<double>("obstacle_layer", "decay_time", 0.0));
  assembly.obstacle_layer->setDecaySweepInterval(obstacle_cfg.param<double>(
      "obstacle_layer", "decay_sweep_interval", 1.0));
  const bool footprint_clearing =
      obstacle_cfg.param<bool>("obstacle_layer", "footprint_clearing", true);

  assembly.inflation_layer = std::make_shared<costmap_2d::InflationLayer>();
  configureInflation(route_cfg, "costmap_local", *assembly.inflation_layer);

  // Observation sources: names + per-source child blocks under the obstacle
  // layer's own section (nav2 semantics: the filtering parameters belong to
  // the consuming obstacle layer, not to the server).
  for (const auto& name :
       obstacle_cfg.paramStringOrList("obstacle_layer", "observation_sources")) {
    Config source_cfg = obstacle_cfg.child("obstacle_layer", name);
    if (!source_cfg.hasSection(name)) {
      SPDLOG_LOGGER_WARN(
        ayanami::costmapLogger(),
        "obstacle layer: observation_sources declares '{}' but there is no "
        "'{}' parameter block; source ignored",
        name,
        name);
      continue;
    }
    ObservationSourceConfig source;
    source.name = name;
    const std::string type =
        source_cfg.param<std::string>(name, "type", "laserscan");
    source.kind =
        type == "pointcloud"
            ? ObservationSourceConfig::Kind::PointCloud
            : ObservationSourceConfig::Kind::LaserScan;
    for (const auto& existing : assembly.observation_sources) {
      if (existing.kind == source.kind) {
        throw std::runtime_error(
            "obstacle layer: more than one '" + type + "' source ('" +
            existing.name + "', '" + name +
            "') is not supported; one source per sensor kind");
      }
    }
    source.marking = source_cfg.param<bool>(name, "marking", true);
    source.clearing = source_cfg.param<bool>(name, "clearing", true);
    source.min_obstacle_height =
        source_cfg.param<double>(name, "min_obstacle_height", 0.0);
    source.height_relative_to_robot =
        source_cfg.param<bool>(name, "height_relative_to_robot", false);
    source.max_obstacle_height =
        source_cfg.param<double>(name, "max_obstacle_height", 1.0);
    source.obstacle_min_range =
        source_cfg.param<double>(name, "obstacle_min_range", 0.0);
    source.obstacle_max_range =
        source_cfg.param<double>(name, "obstacle_max_range", 3.0);
    source.raytrace_min_range =
        source_cfg.param<double>(name, "raytrace_min_range", 0.0);
    source.raytrace_max_range =
        source_cfg.param<double>(name, "raytrace_max_range", 3.5);
    source.expected_update_rate =
        source_cfg.param<double>(name, "expected_update_rate", 0.0);
    auditUnusedParameters(source_cfg, name,
                          "obstacle layer source '" + name + "'");
    assembly.observation_sources.push_back(source);
  }

  assembly.obstacle_applier = std::make_shared<CostmapObstacleApplier>(
      assembly.obstacle_layer, footprint_clearing);
  return assembly;
}

}  // namespace costmap
}  // namespace ayanami
