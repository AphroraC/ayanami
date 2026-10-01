#pragma once

#include <memory>
#include <string>

#include "ayanami/abstract/core/abstract_smoother.hpp"
#include "ayanami/rosemap/core/rosemap_map_context.hpp"
#include "ayanami/types.hpp"

namespace ayanami {
namespace rosemap {

/// @brief Path smoothing module base class (rosemap family, MBF-style
/// layering).
///
/// On top of abstract::AbstractSmoother (zero-map pure virtual interface),
/// only the map-family injection point is added:
/// - configure(context, map): injects the module logical name + its own
///   config + a const pointer to the field module.
/// - smoothPath(...): the abstract pure virtual in-place signature; the
///   implementation holds `currentSnapshot()` for the whole optimization
///   (immutable snapshot, no locks) and evaluates collision costs through
///   the family interpolation contract (distance + analytic gradient).
class RosemapSmoother : public ayanami::abstract::AbstractSmoother {
 public:
  using Ptr = std::shared_ptr<RosemapSmoother>;

  virtual void configure(const ModuleContext& context,
                         RosemapMapContext::ConstPtr map) = 0;
};

}  // namespace rosemap
}  // namespace ayanami
