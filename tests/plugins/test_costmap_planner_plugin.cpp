// 测试插件: costmap 族 planner (SoCostmapPlanner), 公共工厂导出。

#include <ayanami/costmap/core/costmap_planner.hpp>
#include <ayanami/messages/failure.hpp>
#include <ayanami/types.hpp>
#include <ayanami/utility/extension_module.hpp>

namespace {

using ayanami::geometry_msgs::PoseStamped;
using ayanami::nav_msgs::Path;
using ayanami::PlannerResultT;

class SoCostmapPlanner : public ayanami::costmap::CostmapPlanner {
 public:
  void configure(const ayanami::PluginContext& /*context*/,
                 ayanami::costmap::CostmapMapContext::ConstPtr /*map*/) override {}
  PlannerResultT makePlan(const PoseStamped& /*start*/, const PoseStamped& /*goal*/,
                          double /*tolerance*/) override {
    Path plan;
    plan.header.frame_id = "map";
    plan.poses.resize(2);
    return PlannerResultT::success(std::move(plan));
  }
  bool cancel() override { return true; }
};

}  // namespace

extern "C" ayanami::ExtensionModule* create_extension_module() {
  return new SoCostmapPlanner;
}
