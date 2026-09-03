// 测试插件: 失败 planner (FailingPlanner), 经公共工厂 create_extension_module 导出。

#include <ayanami/abstract/core/abstract_planner.hpp>
#include <ayanami/messages/failure.hpp>
#include <ayanami/utility/extension_module.hpp>

namespace {

using ayanami::geometry_msgs::PoseStamped;
using ayanami::FailureCode;
using ayanami::nav_msgs::Path;

class FailingPlanner : public ayanami::abstract::AbstractPlanner {
public:
  FailureCode makePlan(const PoseStamped& /*start*/,
                       const PoseStamped& /*goal*/, double /*tolerance*/,
                       Path& /*plan*/, double& /*cost*/,
                       std::string& /*message*/) override {
    return FailureCode::PathNotFound;
  }
  bool cancel() override { return true; }
};

}  // namespace

extern "C" ayanami::ExtensionModule* create_extension_module() {
  return new FailingPlanner;
}