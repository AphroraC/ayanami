// 测试插件: 非 costmap 族 planner (PlainPlanner), 验证 costmap 宿主拒绝路径。

#include <ayanami/abstract/core/abstract_planner.hpp>
#include <ayanami/messages/failure.hpp>
#include <ayanami/types.hpp>
#include <ayanami/utility/extension_module.hpp>

namespace {

using ayanami::geometry_msgs::PoseStamped;
using ayanami::PlannerResultT;

class PlainPlanner : public ayanami::abstract::AbstractPlanner {
 public:
  PlannerResultT makePlan(const PoseStamped& /*start*/,
                          const PoseStamped& /*goal*/,
                          double /*tolerance*/) override {
    return PlannerResultT::failure(ayanami::ErrorCode::PATH_NOT_FOUND,
                                   "no path found");
  }
  bool cancel() override { return true; }
};

}  // namespace

extern "C" ayanami::ExtensionModule* create_extension_module() {
  return new PlainPlanner;
}
