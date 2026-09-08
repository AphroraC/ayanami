#include <ayanami/abstract/core/abstract_planner.hpp>
#include <ayanami/messages/nav_msgs.hpp>
#include <ayanami/types.hpp>
#include <ayanami/utility/extension_module.hpp>

namespace {

/// Supplement: a real plugin returning AbstractPlanner* (for PluginManager loading).
/// createDummyModule remains a pure dlopen smoke symbol (returns int, only for
/// low-level load_module tests, no algorithm/navigation loading).
class DummyPlanner : public ayanami::abstract::AbstractPlanner {
public:
  ayanami::ResultT<void> makePlan(
      const ayanami::geometry_msgs::PoseStamped& /*start*/,
      const ayanami::geometry_msgs::PoseStamped& /*goal*/,
      double /*tolerance*/, ayanami::nav_msgs::Path& plan_out) override {
    ayanami::nav_msgs::Path plan;
    plan.header.frame_id = "map";
    plan_out = std::move(plan);
    return ayanami::ResultT<void>::success();
  }
};

} // namespace

extern "C" int createDummyModule() { return 42; }

/// Common factory: returns ExtensionModule* (typed by PluginManager via dynamic_pointer_cast).
extern "C" ayanami::ExtensionModule* create_extension_module() {
  return new DummyPlanner;
}