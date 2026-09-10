#pragma once

#include <memory>
#include <string>

#include "ayanami/abstract/core/abstract_controller.hpp"
#include "ayanami/costmap/components/costmap_map_context.hpp"
#include "ayanami/types.hpp"

namespace ayanami {
namespace costmap {

/**
 * @brief Local control module base class (costmap family, MBF-style
 * layering).
 *
 * On top of abstract::AbstractController (zero-map pure virtual interface),
 * only the map-family injection point is added:
 * - configure(context, map): injects the module logical name + its own
 *   config + a const pointer to the map module.
 * - computeVelocityCommand(...): the abstract pure virtual signature; the
 *   implementation reads the local master grid under a lock per cycle via
 *   map->readLocal() (zero copy, released when the callback returns).
 */
class CostmapController : public ayanami::abstract::AbstractController {
 public:
  using Ptr = std::shared_ptr<CostmapController>;

  virtual void configure(const ModuleContext& context,
                         CostmapMapContext::ConstPtr map) = 0;
};

}  // namespace costmap
}  // namespace ayanami
