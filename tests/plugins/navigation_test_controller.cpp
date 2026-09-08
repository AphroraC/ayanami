// Test controller plugin: CostmapController family, never declares arrival itself (arrival is decided
// by the goal_checker), keeps FollowPath RUNNING and emits cmd_vel every cycle
// (verifies the host BT cmd_vel bypass). Exported via the common create_extension_module factory.

#include <ayanami/costmap/core/costmap_controller.hpp>
#include <ayanami/messages/failure.hpp>
#include <ayanami/messages/geometry_msgs.hpp>
#include <ayanami/messages/nav_msgs.hpp>
#include <ayanami/types.hpp>
#include <ayanami/utility/extension_module.hpp>

#include <string>

namespace {

using ayanami::geometry_msgs::PoseStamped;
using ayanami::geometry_msgs::Twist;
using ayanami::geometry_msgs::TwistStamped;
using ayanami::nav_msgs::Path;
using ayanami::ResultT;
class NavigationTestController : public ayanami::costmap::CostmapController {
 public:
  void configure(const ayanami::PluginContext& /*context*/,
                 ayanami::costmap::CostmapMapContext::ConstPtr /*map*/) override {}

  ResultT<void> computeVelocityCommand(const PoseStamped& /*pose*/,
                                           const TwistStamped& /*velocity*/, ayanami::geometry_msgs::TwistStamped& cmd) override {
    cmd = TwistStamped{};  // Zeroing discipline
    cmd.twist = Twist{};
    cmd.twist.linear.x() = 0.5;
    return ResultT<void>::success();
  }

  bool setPlan(const Path& /*plan*/) override { return true; }
  bool cancel() override { return true; }
};

}  // namespace

extern "C" ayanami::ExtensionModule* create_extension_module() {
  return new NavigationTestController;
}
