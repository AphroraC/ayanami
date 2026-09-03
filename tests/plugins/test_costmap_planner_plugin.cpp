// 测试插件: costmap 族 planner (SoCostmapPlanner), 公共工厂导出。

#include <ayanami/costmap/core/costmap_planner.hpp>
#include <ayanami/messages/failure.hpp>
#include <ayanami/utility/extension_module.hpp>

namespace {

using ayanami::geometry_msgs::PoseStamped;
using ayanami::FailureCode;
using ayanami::nav_msgs::Path;

class SoCostmapPlanner : public ayanami::costmap::CostmapPlanner {
 public:
  void initialize(const std::string& /*name*/,
                  ayanami::costmap::CostmapContext::ConstPtr /*map*/) override {}
  FailureCode makePlan(const PoseStamped& /*start*/, const PoseStamped& /*goal*/,
      double /*tolerance*/, Path& plan, double& cost,
      std::string& /*message*/) override {
    plan.header.frame_id = "map";
    plan.poses.resize(2);
    cost = 1.0;
    return FailureCode::None;
  }
  bool cancel() override { return true; }
};

}  // namespace

extern "C" ayanami::ExtensionModule* create_extension_module() {
  return new SoCostmapPlanner;
}