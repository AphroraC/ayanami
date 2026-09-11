#pragma once

#include <memory>
#include <string>

#include "ayanami/abstract/core/abstract_smoother.hpp"
#include "ayanami/costmap/components/costmap_map_context.hpp"
#include "ayanami/types.hpp"

namespace ayanami {
namespace costmap {

/**
 * @brief Path smoothing module base class (costmap family, MBF-style
 * layering).
 *
 * On top of abstract::AbstractSmoother (zero-map pure virtual interface),
 * only the map-family injection point is added:
 * - configure(context, map): injects the module logical name + its own
 *   config + a const pointer to the map module.
 * - smoothPath(...): the abstract pure virtual in-place signature; when
 *   collision checking is needed, the implementation reads the GLOBAL and
 *   the LOCAL map under a lock internally via map->readGlobal()/readLocal()
 *   (zero copy; the two locks are taken sequentially, never nested). A path
 *   expressed in another frame is transformed into each grid's frame once
 *   per call.
 */
class CostmapSmoother : public ayanami::abstract::AbstractSmoother {
 public:
  using Ptr = std::shared_ptr<CostmapSmoother>;

  virtual void configure(const ModuleContext& context,
                         CostmapMapContext::ConstPtr map) = 0;
};

}  // namespace costmap
}  // namespace ayanami
