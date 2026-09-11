#pragma once

#include <string>
#include <vector>

#include <costmap_2d/layer.hpp>

#include "ayanami/costmap/core/costmap_observation.hpp"
#include "ayanami/utility/extension_module.hpp"

namespace ayanami {
namespace costmap {

/**
 * @brief Pluggable costmap layer family (the modules/ so+json pattern,
 * mirrors CostmapPlanner/CostmapController layering).
 *
 * A layer plugin owns one concrete costmap_2d::Layer implementation and
 * hands it to the host via layer(); the host adds it to its LayeredCostmap
 * (list order = composition order) and never touches the plugin's
 * parameters. Configuration is family-shaped like every other module
 * family: the loader calls configureParameters(ModuleContext) once after a
 * successful load/cast/version check, and the plugin fills its own
 * XxxParam member from its own json section.
 *
 * Layers that consume sensor observations (obstacle layers) additionally
 * declare observationSources() and implement observe(): the sole map
 * writer resolves the per-map stamp-time TF and robot pose, takes the
 * map's write lock, and hands the observation over; the layer does its own
 * filtering and marking (nav2: the filtering parameters belong to the
 * ObstacleLayer, not to the costmap node).
 */
class CostmapLayerPlugin : public virtual ExtensionModule {
 public:
  using Ptr = std::shared_ptr<CostmapLayerPlugin>;

  /// @brief Interface version of this family, as compiled into the module.
  /// Bump on ANY change to this header's virtual surface (see
  /// ExtensionModule::apiVersion).
  static constexpr std::uint32_t kApiVersion = 1;
  std::uint32_t apiVersion() const override { return kApiVersion; }

  virtual ~CostmapLayerPlugin() = default;

  /// @brief The layer implementation for LayeredCostmap::addLayer. The
  /// same object every call (the host adds it once; the pointer must stay
  /// valid for the plugin's lifetime).
  virtual costmap_2d::Layer::Ptr layer() = 0;

  /// @brief Host hook after addLayer (initialize has run, master is set):
  /// layers that could not determine their data (e.g. no static map
  /// deployed) may size the master here. Default: no-op.
  virtual void onAddedToMap(costmap_2d::LayeredCostmap& layered_costmap) {
    (void)layered_costmap;
  }

  /// @brief Sources this layer consumes (non-empty only for obstacle
  /// layers); the writer routes observations by Kind.
  virtual const std::vector<ObservationSourceConfig>& observationSources()
      const {
    static const std::vector<ObservationSourceConfig> k_none;
    return k_none;
  }

  /// @brief Apply one observation (the caller holds the target map's write
  /// lock; TF and robot pose are already resolved into the map frame).
  /// Default: no-op (layers that do not consume observations).
  virtual void observe(const MapObservation& observation) {
    (void)observation;
  }
};

}  // namespace costmap
}  // namespace ayanami
