#pragma once

#include <string>

#include "ayanami/map/abstract_map_context.hpp"

namespace ayanami {
namespace rosemap {

/// @brief Reset the field's obstacle marks (rosemap family's write command):
/// every occupancy mark is dropped and the field rebuilds to obstacle-free
/// on the next writer cycle (or immediately for the inline writer path).
struct ResetFieldCommand {
  bool enabled{true};
};

/// @brief Typed map write channel for the rosemap family (exposed
/// family-neutrally via AbstractMapContext::commandPort). Recovery modules
/// submit commands; the sole writer (RosemapNavigationServer) executes them.
class RosemapCommandPort : public map::MapCommandPort {
 public:
  using Ptr = std::shared_ptr<RosemapCommandPort>;

  virtual bool submit(const ResetFieldCommand& command) = 0;
};

}  // namespace rosemap
}  // namespace ayanami
