#pragma once

#include <memory>
#include <string>
#include <vector>

#include <costmap_2d_layers/simple_obstacle_layer.hpp>

#include <ayanami/costmap/core/costmap_layer_plugin.hpp>
#include <ayanami/utility/config.hpp>

namespace ayanami {
namespace modules {

/// @brief Obstacle-layer plugin (costmap layer family, modules/ so+json).
///
/// Owns a costmap_2d_layers::SimpleObstacleLayer and owns the observation
/// filtering policy for the sources it consumes (nav2 semantics: the
/// filtering parameters belong to the ObstacleLayer, not to the costmap
/// node):
/// - combination: master-grid write policy;
/// - decay_time: drop marks not re-observed within this window (0 = off);
///   decay_sweep_interval throttles the full decay sweeps;
/// - observation_sources + per-source child blocks (type, marking,
///   clearing, height/range windows, expected_update_rate);
/// - footprint_clearing.
///
/// observe() runs under the target map's write lock (the sole map writer
/// resolved the stamp-time TF and robot pose already) and performs: hit
/// filtering in the map frame -> per-endpoint raytrace clearing ->
/// marking -> footprint clearing. A frame with no ray endpoints clears
/// nothing; retired marks age out via decay_time.
class CostmapObstacleLayerPlugin final
    : public ayanami::costmap::CostmapLayerPlugin {
 public:
  CostmapObstacleLayerPlugin() = default;
  /// @brief Out-of-line key function: pins the vtable emission to
  /// module.cpp (all-inline virtuals make it vague-linkage, which dlopen
  /// cannot resolve across the module boundary).
  ~CostmapObstacleLayerPlugin() override;

  costmap_2d::Layer::Ptr layer() override { return impl_; }

  void configureParameters(const ayanami::ModuleContext& context) override;

  const std::vector<ayanami::costmap::ObservationSourceConfig>&
  observationSources() const override {
    return sources_;
  }

  void observe(const ayanami::costmap::MapObservation& observation) override;

 private:
  struct CostmapObstacleLayerParam {
    std::string combination = "maximum";  // "overwrite" | "maximum" | "none"
    double decay_time = 0.0;              // seconds; 0 = disabled
    /// @brief Minimum interval between full decay sweeps (seconds).
    double decay_sweep_interval = 1.0;
    bool footprint_clearing = true;
  };

  CostmapObstacleLayerParam param_;
  std::shared_ptr<costmap_2d_layers::SimpleObstacleLayer> impl_{
      std::make_shared<costmap_2d_layers::SimpleObstacleLayer>()};
  std::vector<ayanami::costmap::ObservationSourceConfig> sources_;
  std::chrono::steady_clock::time_point last_outside_warn_at_{};
};

}  // namespace modules
}  // namespace ayanami
