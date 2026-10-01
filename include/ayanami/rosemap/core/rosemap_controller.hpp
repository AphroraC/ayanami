#pragma once

#include <memory>
#include <string>

#include "ayanami/abstract/core/abstract_controller.hpp"
#include "ayanami/rosemap/core/rosemap_map_context.hpp"
#include "ayanami/types.hpp"

namespace ayanami {
namespace rosemap {

/// @brief Local control module base class (rosemap family, MBF-style
/// layering).
///
/// On top of abstract::AbstractController (zero-map pure virtual interface),
/// only the map-family injection point is added:
/// - configure(context, map): injects the module logical name + its own
///   config + a const pointer to the field module. An MPC-style controller
///   may ignore the field entirely (obstacle safety lives upstream in the
///   planner/smoother/BT layers).
/// - computeVelocityCommand(...): the abstract pure virtual signature.
class RosemapController : public ayanami::abstract::AbstractController {
 public:
  using Ptr = std::shared_ptr<RosemapController>;

  virtual void configure(const ModuleContext& context,
                         RosemapMapContext::ConstPtr map) = 0;
};

}  // namespace rosemap
}  // namespace ayanami
