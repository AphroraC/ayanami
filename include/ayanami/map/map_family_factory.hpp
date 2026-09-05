#pragma once

#include "ayanami/map/abstract_map_context.hpp"

#include <memory>
#include <string>

namespace ayanami::map {

/** Factory seam owned by a concrete map family, not NavigationServerBase. */
class MapFamilyFactory {
 public:
  using Ptr = std::shared_ptr<MapFamilyFactory>;

  virtual ~MapFamilyFactory() = default;

  virtual std::string familyName() const = 0;
  virtual AbstractMapContext::Ptr createContext() = 0;
};

}  // namespace ayanami::map
