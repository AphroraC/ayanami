// Test plugin: costmap-family controller (SoCostmapController), common factory export.

#include <ayanami/costmap/core/costmap_controller.hpp>
#include <ayanami/messages/failure.hpp>
#include <ayanami/types.hpp>
#include <ayanami/utility/extension_module.hpp>

namespace {

using ayanami::geometry_msgs::PoseStamped;
using ayanami::geometry_msgs::TwistStamped;
using ayanami::nav_msgs::Path;
using ayanami::ResultT;
class SoCostmapController : public ayanami::costmap::CostmapController {
 public:
  void configure(const ayanami::PluginContext& /*context*/,
                 ayanami::costmap::CostmapMapContext::ConstPtr /*map*/) override {}
  ResultT<void> computeVelocityCommand(const PoseStamped& /*pose*/,
                                           const TwistStamped& /*velocity*/, ayanami::geometry_msgs::TwistStamped& cmd) override {
    cmd = TwistStamped{};
    cmd.twist.linear.x() = 0.3;
    return ResultT<void>::success();
  }
  bool setPlan(const Path& /*plan*/) override { return true; }
  bool cancel() override { return true; }
};

}  // namespace

extern "C" ayanami::ExtensionModule* create_extension_module() {
  return new SoCostmapController;
}
