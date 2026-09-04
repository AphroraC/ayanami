// 测试插件: 成功 controller (SuccessController), 经公共工厂 create_extension_module 导出。

#include <ayanami/abstract/core/abstract_controller.hpp>
#include <ayanami/messages/failure.hpp>
#include <ayanami/types.hpp>
#include <ayanami/utility/extension_module.hpp>

namespace {

using ayanami::geometry_msgs::PoseStamped;
using ayanami::geometry_msgs::TwistStamped;
using ayanami::nav_msgs::Path;
using ayanami::ControllerCommand;
using ayanami::ControllerResultT;

class SuccessController : public ayanami::abstract::AbstractController {
public:
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
  return new SuccessController;
}
