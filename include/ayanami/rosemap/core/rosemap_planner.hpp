#pragma once

#include <memory>
#include <string>

#include "ayanami/abstract/core/abstract_planner.hpp"
#include "ayanami/rosemap/core/rosemap_map_context.hpp"
#include "ayanami/types.hpp"

namespace ayanami {
namespace rosemap {

/// @brief Global planning module base class (rosemap family, MBF-style
/// layering).
///
/// On top of abstract::AbstractPlanner (zero-map pure virtual interface),
/// only the map-family injection point is added:
/// - configure(context, map): module logical name + its own config + the
///   rosemap field module (owned by the host, injected as a const smart
///   pointer; the module never modifies the field).
/// - makePlan(...): the abstract pure virtual signature; the implementation
///   holds `currentSnapshot()` for the whole search (immutable snapshot, no
///   locks) and plans on the distance field.
class RosemapPlanner : public ayanami::abstract::AbstractPlanner {
 public:
  using Ptr = std::shared_ptr<RosemapPlanner>;

  virtual void configure(const ModuleContext& context,
                         RosemapMapContext::ConstPtr map) = 0;
};

}  // namespace rosemap
}  // namespace ayanami
