// Test module: costmap-family smoother (SoCostmapSmoother), common factory export.

#include <ayanami/costmap/core/costmap_smoother.hpp>
#include <ayanami/messages/failure.hpp>
#include <ayanami/types.hpp>
#include <ayanami/utility/extension_module.hpp>

namespace {

using ayanami::nav_msgs::Path;
using ayanami::ResultT;
class SoCostmapSmoother : public ayanami::costmap::CostmapSmoother {
 public:
  void configure(const ayanami::ModuleContext& /*context*/,
                 ayanami::costmap::CostmapMapContext::ConstPtr /*map*/) override {}
  ResultT<void> smoothPath(Path& path, double /*max_duration*/) override {
    path.header.frame_id = "map";
    return ResultT<void>::success();
  }
  bool cancel() override { return true; }
};

}  // namespace

extern "C" ayanami::ExtensionModule* create_extension_module() {
  return new SoCostmapSmoother;
}
