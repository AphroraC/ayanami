#pragma once

#include <memory>
#include <string>

#include "ayanami/map/map_family_factory.hpp"
#include "ayanami/rosemap/core/rosemap_map_context.hpp"

namespace ayanami {
namespace rosemap {

/// @brief The rosemap map-family factory: the sole creation point of
/// RosemapMapContext (the core server only knows the MapFamilyFactory
/// abstract surface).
///
/// The host (RosemapNavigation) constructs this factory and passes it to
/// RosemapNavigationServer::configureCoreMaps; the family server assembles
/// frames/commandPort/field after obtaining an empty context via
/// createContext(). Parallel map families plug in as their own
/// {Family}MapFamilyFactory implementations, unaware of this one.
class RosemapMapFamilyFactory final : public map::MapFamilyFactory {
 public:
  using Ptr = std::shared_ptr<RosemapMapFamilyFactory>;

  std::string familyName() const override { return "rosemap"; }

  /// @brief Create an empty rosemap family context (content assembled by
  /// configureCoreMaps).
  map::AbstractMapContext::Ptr createContext() override {
    return std::make_shared<RosemapMapContext>();
  }
};

}  // namespace rosemap
}  // namespace ayanami
