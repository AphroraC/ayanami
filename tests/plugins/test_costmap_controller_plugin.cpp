// 测试插件: costmap 族 controller (SoCostmapController), 公共工厂导出。

#include <ayanami/costmap/core/costmap_controller.hpp>
#include <ayanami/messages/failure.hpp>
#include <ayanami/utility/extension_module.hpp>

namespace {

using ayanami::geometry_msgs::PoseStamped;
using ayanami::geometry_msgs::TwistStamped;
using ayanami::FailureCode;
using ayanami::nav_msgs::Path;

class SoCostmapController : public ayanami::costmap::CostmapController {
 public:
  void initialize(const std::string& /*name*/,
                  ayanami::costmap::CostmapContext::ConstPtr /*map*/) override {}
  FailureCode computeVelocityCommands(
      const PoseStamped& /*pose*/, const TwistStamped& /*velocity*/,
      TwistStamped& cmd_vel, std::string& /*message*/) override {
    cmd_vel.twist.linear.x() = 0.3;
    return FailureCode::None;
  }
  bool setPlan(const Path& /*plan*/) override { return true; }
  bool cancel() override { return true; }
};

}  // namespace

extern "C" ayanami::ExtensionModule* create_extension_module() {
  return new SoCostmapController;
}