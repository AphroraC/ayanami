// 测试插件: 成功 planner (SuccessPlanner), 经公共工厂 create_extension_module 导出。

#include <ayanami/abstract/core/abstract_planner.hpp>
#include <ayanami/messages/failure.hpp>
#include <ayanami/types.hpp>
#include <ayanami/utility/extension_module.hpp>

namespace {

using ayanami::geometry_msgs::PoseStamped;
using ayanami::nav_msgs::Path;
using ayanami::PlannerResultT;

class SuccessPlanner : public ayanami::abstract::AbstractPlanner {
public:
  PlannerResultT makePlan(const PoseStamped& start,
                          const PoseStamped& /*goal*/,
                          double /*tolerance*/) override {
    Path plan;
    plan.header.frame_id = "map";
    plan.poses.push_back(start);
    plan.poses.push_back(start);
    return PlannerResultT::success(std::move(plan));
  }
  bool cancel() override { return true; }
};

}  // namespace

extern "C" ayanami::ExtensionModule* create_extension_module() {
  return new SuccessPlanner;
}
