// 测试插件: 非 costmap 族 planner (PlainPlanner), 验证 costmap 宿主拒绝路径。

#include <ayanami/abstract/core/abstract_planner.hpp>
#include <ayanami/messages/failure.hpp>
#include <ayanami/utility/extension_module.hpp>

namespace {

using ayanami::geometry_msgs::PoseStamped;
using ayanami::FailureCode;
using ayanami::nav_msgs::Path;

class PlainPlanner : public ayanami::abstract::AbstractPlanner {
 public:
  FailureCode makePlan(const PoseStamped& /*start*/, const PoseStamped& /*goal*/,
                       double /*tolerance*/, Path& /*plan*/, double& /*cost*/,
                       std::string& /*message*/) override {
    return FailureCode::PathNotFound;
  }
  bool cancel() override { return true; }
};

}  // namespace

extern "C" ayanami::ExtensionModule* create_extension_module() {
  return new PlainPlanner;
}