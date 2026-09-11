#pragma once

#include <memory>
#include <string>

#include <costmap_2d/layered_costmap.hpp>
#include <costmap_2d_layers/static_layer.hpp>

#include <ayanami/costmap/core/costmap_layer_plugin.hpp>
#include <ayanami/utility/config.hpp>

#include <stdexcept>
#include <filesystem>

#include <Eigen/Geometry>
#include <spdlog/spdlog.h>

namespace ayanami {
namespace modules {

/// @brief Static-layer plugin (costmap layer family, modules/ so+json).
///
/// Owns a costmap_2d_layers::StaticLayer and reads its own json section:
/// static_map (pgm path, resolved relative to the config directory like
/// every other config asset) plus the optional size_x/size_y fallback the
/// host consults when no map file is deployed.
class CostmapStaticLayerPlugin final : public ayanami::costmap::CostmapLayerPlugin {
 public:
  CostmapStaticLayerPlugin() = default;
  /// @brief Out-of-line key function: pins the vtable emission to
  /// module.cpp (all-inline virtuals make it vague-linkage, which dlopen
  /// cannot resolve across the module boundary).
  ~CostmapStaticLayerPlugin() override;

  costmap_2d::Layer::Ptr layer() override { return impl_; }

  void configureParameters(const ayanami::ModuleContext& context) override;

  /// @brief The host's fallback geometry question (no map file deployed):
  /// read from the plugin's own params instead of map-level keys.
  int fallbackSizeX() const { return param_.fallback_size_x; }
  int fallbackSizeY() const { return param_.fallback_size_y; }

  /// @brief Runs after addLayer (master set): load the configured map
  /// file, then size the master from it -- or from the fallback geometry
  /// when no map is deployed (a missing fallback size is a startup error,
  /// matching the legacy server behavior).
  void onAddedToMap(costmap_2d::LayeredCostmap& layered_costmap) override;

  /// @brief Read the pgm + <pgm>.json metadata into the layer (the master
  /// must already be set; StaticLayer::processMap sizes the master).
  void loadMap();

 private:
  struct CostmapStaticLayerParam {
    /// @brief static_map pgm path; relative paths resolve against the
    /// config directory (global.config_path). Empty = no map file.
    std::string static_map;
    int fallback_size_x{0};
    int fallback_size_y{0};
    double fallback_resolution{0.05};
  };

  CostmapStaticLayerParam param_;
  std::shared_ptr<costmap_2d_layers::StaticLayer> impl_{
      std::make_shared<costmap_2d_layers::StaticLayer>()};
};


}  // namespace modules
}  // namespace ayanami
