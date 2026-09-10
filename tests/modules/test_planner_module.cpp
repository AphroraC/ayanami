// Test module: success planner (SuccessPlanner), exported via the common create_extension_module factory.

#include <ayanami/abstract/core/abstract_planner.hpp>
#include <ayanami/messages/failure.hpp>
#include <ayanami/types.hpp>
#include <ayanami/utility/extension_module.hpp>

namespace {

using ayanami::geometry_msgs::PoseStamped;
using ayanami::nav_msgs::Path;
using ayanami::ResultT;
class SuccessPlanner : public ayanami::abstract::AbstractPlanner {
public:
  ResultT<void> makePlan(const PoseStamped& start,
                          const PoseStamped& /*goal*/,
                          double /*tolerance*/, ayanami::nav_msgs::Path& plan_out) override {
    Path plan;
    plan.header.frame_id = "map";
    plan.poses.push_back(start);
    plan.poses.push_back(start);
    plan_out = std::move(plan);
    return ResultT<void>::success();
  }
  bool cancel() override { return true; }
};

}  // namespace

extern "C" ayanami::ExtensionModule* create_extension_module() {
  return new SuccessPlanner;
}
