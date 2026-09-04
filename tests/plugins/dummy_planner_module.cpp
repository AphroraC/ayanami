#include <ayanami/abstract/core/abstract_planner.hpp>
#include <ayanami/messages/nav_msgs.hpp>
#include <ayanami/types.hpp>
#include <ayanami/utility/extension_module.hpp>

namespace {

// Task 10 补充: 真实插件, 返回 AbstractPlanner* (PluginManager 加载用)。
// createDummyModule 保留为 Task 3 的纯 dlopen 冒烟符号 (返回 int, 仅供
// 低层 load_module 测试, 不做算法/导航装载)。
class DummyPlanner : public ayanami::abstract::AbstractPlanner {
public:
  ayanami::PlannerResultT makePlan(
      const ayanami::geometry_msgs::PoseStamped& /*start*/,
      const ayanami::geometry_msgs::PoseStamped& /*goal*/,
      double /*tolerance*/) override {
    ayanami::nav_msgs::Path plan;
    plan.header.frame_id = "map";
    return ayanami::PlannerResultT::success(std::move(plan));
  }
};

} // namespace

extern "C" int createDummyModule() { return 42; }

// 公共工厂: 返回 ExtensionModule* (PluginManager 经 dynamic_pointer_cast 类型化)。
extern "C" ayanami::ExtensionModule* create_extension_module() {
  return new DummyPlanner;
}