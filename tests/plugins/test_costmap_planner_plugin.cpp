// Test plugin: costmap-family planner (SoCostmapPlanner), common factory export.

#include <ayanami/costmap/core/costmap_planner.hpp>
#include <ayanami/messages/failure.hpp>
#include <ayanami/types.hpp>
#include <ayanami/utility/extension_module.hpp>

namespace {

using ayanami::geometry_msgs::PoseStamped;
using ayanami::nav_msgs::Path;
using ayanami::ResultT;
class SoCostmapPlanner : public ayanami::costmap::CostmapPlanner {
 public:
  void configure(const ayanami::PluginContext& /*context*/,
                 ayanami::costmap::CostmapMapContext::ConstPtr /*map*/) override {}
  ResultT<void> makePlan(const PoseStamped& /*start*/, const PoseStamped& /*goal*/,
                          double /*tolerance*/, ayanami::nav_msgs::Path& plan_out) override {
    Path plan;
    plan.header.frame_id = "map";
    plan.poses.resize(2);
    plan_out = std::move(plan);
    return ResultT<void>::success();
  }
  bool cancel() override { return true; }
};

}  // namespace

extern "C" ayanami::ExtensionModule* create_extension_module() {
  return new SoCostmapPlanner;
}
