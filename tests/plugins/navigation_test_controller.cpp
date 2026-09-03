// Task 25 测试控制器插件: CostmapController 族, 永不主动判到达 (到达判定由
// goal_checker 负责) 让 FollowPath 持续 RUNNING 并每周期 emit cmd_vel
// (验证宿主 BT 旁路 cmd_vel)。经公共工厂 create_extension_module 导出。

#include <ayanami/costmap/core/costmap_controller.hpp>
#include <ayanami/messages/failure.hpp>
#include <ayanami/messages/geometry_msgs.hpp>
#include <ayanami/messages/nav_msgs.hpp>
#include <ayanami/utility/extension_module.hpp>

#include <string>

namespace {

using ayanami::geometry_msgs::PoseStamped;
using ayanami::FailureCode;

using ayanami::geometry_msgs::Twist;
using ayanami::geometry_msgs::TwistStamped;
using ayanami::nav_msgs::Path;

class NavigationTestController : public ayanami::costmap::CostmapController {
 public:
  void initialize(const std::string& /*name*/,
                  ayanami::costmap::CostmapContext::ConstPtr /*map*/) override {}

  FailureCode computeVelocityCommands(
      const PoseStamped& /*pose*/, const TwistStamped& /*velocity*/,
      TwistStamped& cmd_vel, std::string& /*message*/) override {
    cmd_vel = TwistStamped{};  // 清零纪律
    cmd_vel.twist = Twist{};
    cmd_vel.twist.linear.x() = 0.5;
    return FailureCode::None;
  }

  bool setPlan(const Path& /*plan*/) override { return true; }
  bool cancel() override { return true; }
};

}  // namespace

extern "C" ayanami::ExtensionModule* create_extension_module() {
  return new NavigationTestController;
}