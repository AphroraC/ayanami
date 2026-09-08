#pragma once

#include <memory>
#include <string>

#include "ayanami/costmap/components/costmap_map_context.hpp"
#include "ayanami/map/map_family_factory.hpp"

namespace ayanami {
namespace costmap {

/**
 * @brief The costmap map-family factory: the sole creation point of
 * CostmapMapContext (spec map-family abstraction: "the core server does not
 * know CostmapContext, only the map-family factory").
 *
 * - The host (CostmapNavigation) constructs this factory and passes it to
 *   CostmapNavigationServer::configureCoreMaps; the family server assembles
 *   wrapper/frames/footprint/commandPort after obtaining an empty context
 *   via createContext().
 * - Future map families (mesh/voxel, registered in
 *   map/map_family_factory.hpp) plug in as parallel
 *   {Family}MapFamilyFactory implementations, unaware of each other.
 * - The factory returns AbstractMapContext::Ptr (the abstract surface); the
 *   family server holds CostmapMapContext::Ptr (the family surface); the
 *   two converge via dynamic_pointer_cast.
 */
class CostmapMapFamilyFactory final : public map::MapFamilyFactory {
 public:
  using Ptr = std::shared_ptr<CostmapMapFamilyFactory>;

  std::string familyName() const override { return "costmap"; }

  /// @brief Create an empty costmap family context (content assembled by configureCoreMaps).
  map::AbstractMapContext::Ptr createContext() override {
    return std::make_shared<CostmapMapContext>();
  }
};

}  // namespace costmap
}  // namespace ayanami
