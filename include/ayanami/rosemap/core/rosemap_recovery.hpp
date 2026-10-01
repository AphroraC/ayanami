#pragma once

#include <memory>
#include <string>

#include "ayanami/abstract/core/abstract_recovery.hpp"
#include "ayanami/rosemap/core/rosemap_command.hpp"
#include "ayanami/rosemap/core/rosemap_map_context.hpp"
#include "ayanami/types.hpp"

namespace ayanami {
namespace rosemap {

using ayanami::ErrorCode;
using ayanami::ResultT;

/// @brief Recovery behavior module base class (rosemap family, MBF-style
/// layering).
///
/// On top of abstract::AbstractRecovery (the start/tick/cancel/finish
/// unified protocol), only the following is added:
/// - configure(context, map): injects the module logical name + its own
///   config + a const pointer to the field module (escape-type recoveries
///   read the distance field / gradient per tick via queryRosemap).
/// Recoveries that need to modify the field submit commands through
/// map->commandPort() (RosemapCommandPort), executed by the sole writer
/// (RosemapNavigationServer); recovery modules never touch the writer's
/// working buffers.
class RosemapRecovery : public ayanami::abstract::AbstractRecovery {
 public:
  using Ptr = std::shared_ptr<RosemapRecovery>;

  virtual void configure(const ModuleContext& context,
                         RosemapMapContext::ConstPtr map) = 0;
};

}  // namespace rosemap
}  // namespace ayanami
