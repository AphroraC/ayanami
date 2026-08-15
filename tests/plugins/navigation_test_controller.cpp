// Task 25 测试控制器插件: CostmapController 族, isGoalReached=false 让
// FollowPath 持续 RUNNING 并每周期 emit cmd_vel (验证宿主 BT 旁路 cmd_vel)。

#include <ayanami/costmap/components/costmap_context.hpp>
#include <ayanami/costmap/core/costmap_controller.hpp>
#include <ayanami/messages/actions.hpp>
#include <ayanami/messages/geometry_msgs.hpp>
#include <ayanami/messages/nav_msgs.hpp>

#include <cstdint>
#include <string>

namespace {

using ayanami::geometry_msgs::PoseStamped;
using ayanami::geometry_msgs::Twist;
using ayanami::geometry_msgs::TwistStamped;
using ayanami::nav_msgs::Path;

class NavigationTestController : public ayanami::costmap::CostmapController {
 public:
  void initialize(const std::string& /*name*/,
                  ayanami::costmap::CostmapContext::Ptr /*ctx*/) override {}

  uint32_t computeVelocityCommands(const PoseStamped& /*pose*/,
                                   const TwistStamped& /*velocity*/,
                                   TwistStamped& cmd_vel,
                                   std::string& /*message*/) override {
    cmd_vel = TwistStamped{};  // 清零纪律
    cmd_vel.twist = Twist{};
    cmd_vel.twist.linear.x() = 0.5;
    return ayanami::actions::ExePathResult::SUCCESS;
  }

  bool isGoalReached(double /*dist*/, double /*angle*/) override {
    return false;
  }

  bool setPlan(const Path& /*plan*/) override { return true; }
  bool cancel() override { return true; }
};

}  // namespace

extern "C" ayanami::abstract::AbstractController*
create_navigation_test_controller_plugin() {
  return new NavigationTestController;
}
