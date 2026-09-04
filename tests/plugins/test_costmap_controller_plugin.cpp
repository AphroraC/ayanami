// 测试插件: costmap 族 controller (SoCostmapController), 公共工厂导出。

#include <ayanami/costmap/core/costmap_controller.hpp>
#include <ayanami/messages/failure.hpp>
#include <ayanami/types.hpp>
#include <ayanami/utility/extension_module.hpp>

namespace {

using ayanami::geometry_msgs::PoseStamped;
using ayanami::geometry_msgs::TwistStamped;
using ayanami::nav_msgs::Path;
using ayanami::ControllerCommand;
using ayanami::ControllerResultT;

class SoCostmapController : public ayanami::costmap::CostmapController {
 public:
  void configure(const ayanami::PluginContext& /*context*/,
                 ayanami::costmap::CostmapContext::ConstPtr /*map*/) override {}
  ControllerResultT computeVelocityCommand(const PoseStamped& /*pose*/,
                                           const TwistStamped& /*velocity*/) override {
    ControllerCommand cmd;
    cmd.velocity = TwistStamped{};
    cmd.velocity.twist.linear.x() = 0.3;
    return ControllerResultT::success(std::move(cmd));
  }
  bool setPlan(const Path& /*plan*/) override { return true; }
  bool cancel() override { return true; }
};

}  // namespace

extern "C" ayanami::ExtensionModule* create_extension_module() {
  return new SoCostmapController;
}
