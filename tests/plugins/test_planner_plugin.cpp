// 测试插件: 成功 planner (SuccessPlanner), 经公共工厂 create_extension_module 导出。

#include <ayanami/abstract/core/abstract_planner.hpp>
#include <ayanami/messages/failure.hpp>
#include <ayanami/utility/extension_module.hpp>

namespace {

using ayanami::geometry_msgs::PoseStamped;
using ayanami::FailureCode;
using ayanami::nav_msgs::Path;

class SuccessPlanner : public ayanami::abstract::AbstractPlanner {
public:
  FailureCode makePlan(const PoseStamped& start, const PoseStamped& /*goal*/,
                       double /*tolerance*/, Path& plan, double& cost,
                       std::string& /*message*/) override {
    plan.header.frame_id = "map";
    plan.poses.push_back(start);
    plan.poses.push_back(start);
    cost = 2.0;
    return FailureCode::None;
  }
  bool cancel() override { return true; }
};

}  // namespace

extern "C" ayanami::ExtensionModule* create_extension_module() {
  return new SuccessPlanner;
}