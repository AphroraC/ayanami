// Test module: non-costmap-family planner (PlainPlanner), verifies the costmap host reject path.

#include <ayanami/abstract/core/abstract_planner.hpp>
#include <ayanami/messages/failure.hpp>
#include <ayanami/types.hpp>
#include <ayanami/utility/extension_module.hpp>

namespace {

using ayanami::geometry_msgs::PoseStamped;
using ayanami::ResultT;
class PlainPlanner : public ayanami::abstract::AbstractPlanner {
 public:
  ResultT<void> makePlan(const PoseStamped& /*start*/,
                          const PoseStamped& /*goal*/,
                          double /*tolerance*/, ayanami::nav_msgs::Path& plan_out) override {
    return ResultT<void>::failure(ayanami::ErrorCode::PATH_NOT_FOUND,
                                   "no path found");
  }
  bool cancel() override { return true; }
};

}  // namespace

extern "C" ayanami::ExtensionModule* create_extension_module() {
  return new PlainPlanner;
}
