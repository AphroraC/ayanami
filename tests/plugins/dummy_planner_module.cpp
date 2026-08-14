#include <ayanami/core/abstract_planner.hpp>

namespace {

// Task 10 补充: 真实插件工厂, 返回 AbstractPlanner* (PluginManager 加载用)。
// create_dummy_module 保留为 Task 3 的纯 dlopen 冒烟符号 (返回 int)。
class DummyPlanner : public ayanami::abstract::AbstractPlanner {
public:
  uint32_t makePlan(const ayanami::geometry_msgs::PoseStamped& /*start*/,
                    const ayanami::geometry_msgs::PoseStamped& /*goal*/,
                    double /*tolerance*/, ayanami::nav_msgs::Path& plan,
                    double& cost, std::string& /*message*/) override {
    plan.header.frame_id = "map";
    cost = 0.0;
    return 0;
  }
};

} // namespace

extern "C" int create_dummy_module() { return 42; }

extern "C" ayanami::abstract::AbstractPlanner* create_dummy_planner_plugin() {
  return new DummyPlanner;
}
