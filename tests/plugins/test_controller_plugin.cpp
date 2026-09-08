// Test plugin: success controller (SuccessController), exported via the common create_extension_module factory.

#include <ayanami/abstract/core/abstract_controller.hpp>
#include <ayanami/messages/failure.hpp>
#include <ayanami/types.hpp>
#include <ayanami/utility/extension_module.hpp>

namespace {

using ayanami::geometry_msgs::PoseStamped;
using ayanami::geometry_msgs::TwistStamped;
using ayanami::nav_msgs::Path;
using ayanami::ResultT;
class SuccessController : public ayanami::abstract::AbstractController {
public:
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
  return new SuccessController;
}
