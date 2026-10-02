#pragma once

#include <memory>
#include <string>

#include <esdfmap/esdf_map.hpp>

#include "ayanami/abstract/core/abstract_smoother.hpp"
#include "ayanami/types.hpp"

namespace ayanami {
namespace esdfmap {

/// @brief Path smoothing module base class (esdfmap family, MBF-style
/// layering).
///
/// On top of abstract::AbstractSmoother (zero-map pure virtual interface),
/// only the map injection point is added:
/// - configure(context, map): injects the module logical name + its own
///   config + the ESDF map handle (the smoother is the field's only reader).
/// - smoothPath(...): the abstract pure virtual in-place signature; the
///   implementation holds `currentSnapshot()` for the whole optimization
///   (immutable snapshot, lock-free) and evaluates collision costs through
///   the package interpolation contract (distance + analytic gradient).
class EsdfmapSmoother : public ayanami::abstract::AbstractSmoother {
 public:
  using Ptr = std::shared_ptr<EsdfmapSmoother>;

  virtual void configure(const ModuleContext& context,
                         ::esdfmap::EsdfMap::Ptr map) = 0;
};

}  // namespace esdfmap
}  // namespace ayanami
