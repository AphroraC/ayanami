#pragma once

#include <memory>
#include <string>

#include "ayanami/abstract/core/abstract_planner.hpp"
#include "ayanami/costmap/components/costmap_map_context.hpp"
#include "ayanami/types.hpp"

namespace ayanami {
namespace costmap {

/**
 * @brief Global planning module base class (costmap family, MBF-style
 * layering).
 *
 * On top of abstract::AbstractPlanner (zero-map pure virtual interface),
 * only the map-family injection point is added:
 * - configure(context, map): module logical name + its own config + the
 *   costmap map module (owned by the host, injected as a const smart
 *   pointer; the module never modifies the map).
 * - makePlan(...): the abstract pure virtual signature; the implementation
 *   reads the map through its held map's readGlobal() callback (shared lock +
 *   zero copy), holding the lock for a single planning call (<10ms); the
 *   map writer waits for the lock to write.
 */
class CostmapPlanner : public ayanami::abstract::AbstractPlanner {
 public:
  using Ptr = std::shared_ptr<CostmapPlanner>;

  virtual void configure(const ModuleContext& context,
                         CostmapMapContext::ConstPtr map) = 0;
};

}  // namespace costmap
}  // namespace ayanami
