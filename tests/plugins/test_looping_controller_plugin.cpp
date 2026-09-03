// 测试插件: 永不到达目标的 controller (LoopingController), 经公共工厂
// create_extension_module 导出。

#include <ayanami/abstract/core/abstract_controller.hpp>
#include <ayanami/messages/failure.hpp>
#include <ayanami/utility/extension_module.hpp>

namespace {

using ayanami::geometry_msgs::PoseStamped;
using ayanami::geometry_msgs::TwistStamped;
using ayanami::FailureCode;
using ayanami::nav_msgs::Path;

class LoopingController : public ayanami::abstract::AbstractController {
public:
  FailureCode computeVelocityCommands(const PoseStamped& /*pose*/,
                                      const TwistStamped& /*velocity*/,
                                      TwistStamped& cmd_vel,
                                      std::string& /*message*/) override {
    cmd_vel.twist.linear.x() = 0.1;
    return FailureCode::None;
  }
  bool setPlan(const Path& /*plan*/) override { return true; }
  bool cancel() override { return true; }
};

}  // namespace

extern "C" ayanami::ExtensionModule* create_extension_module() {
  return new LoopingController;
}