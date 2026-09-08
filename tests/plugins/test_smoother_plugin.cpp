// Test plugin: success smoother (SuccessSmoother), exported via the common create_extension_module factory.

#include <ayanami/abstract/core/abstract_smoother.hpp>
#include <ayanami/messages/failure.hpp>
#include <ayanami/types.hpp>
#include <ayanami/utility/extension_module.hpp>

namespace {

using ayanami::nav_msgs::Path;
using ayanami::ResultT;
class SuccessSmoother : public ayanami::abstract::AbstractSmoother {
public:
  ResultT<void> smoothPath(Path& path, double /*max_duration*/) override {
    path.header.frame_id = "map";
    return ResultT<void>::success();
  }
  bool cancel() override { return true; }
};

}  // namespace

extern "C" ayanami::ExtensionModule* create_extension_module() {
  return new SuccessSmoother;
}
