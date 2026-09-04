// 测试插件: 失败 planner (FailingPlanner), 经公共工厂 create_extension_module 导出。

#include <ayanami/abstract/core/abstract_planner.hpp>
#include <ayanami/messages/failure.hpp>
#include <ayanami/types.hpp>
#include <ayanami/utility/extension_module.hpp>

namespace {

using ayanami::geometry_msgs::PoseStamped;
using ayanami::PlannerResultT;

class FailingPlanner : public ayanami::abstract::AbstractPlanner {
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
  return new FailingPlanner;
}
