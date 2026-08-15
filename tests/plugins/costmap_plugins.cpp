// Task 20 测试插件: CostmapNavigationServer 的 4 族 costmap 插件工厂。
//
// 工厂符号按 spec §4.3 约定: extern "C" 无参构造返回 PluginType*。
// create_plain_abstract_planner_plugin 返回非 costmap 族 planner,
// 用于验证 initializePlannerPlugin 的 dynamic_pointer_cast 失败路径。

#include <ayanami/core/abstract_controller.hpp>
#include <ayanami/core/abstract_planner.hpp>
#include <ayanami/core/abstract_recovery.hpp>
#include <ayanami/core/abstract_smoother.hpp>
#include <ayanami/costmap/components/costmap_context.hpp>
#include <ayanami/costmap/core/costmap_controller.hpp>
#include <ayanami/costmap/core/costmap_planner.hpp>
#include <ayanami/costmap/core/costmap_recovery.hpp>
#include <ayanami/costmap/core/costmap_smoother.hpp>
#include <ayanami/messages/geometry_msgs.hpp>
#include <ayanami/messages/nav_msgs.hpp>

#include <cstdint>
#include <string>

namespace {

using ayanami::geometry_msgs::PoseStamped;
using ayanami::geometry_msgs::TwistStamped;
using ayanami::nav_msgs::Path;

class SoCostmapPlanner : public ayanami::costmap::CostmapPlanner {
 public:
  void initialize(const std::string& /*name*/,
                  ayanami::costmap::CostmapContext::Ptr /*ctx*/) override {}
  uint32_t makePlan(const PoseStamped& /*start*/, const PoseStamped& /*goal*/,
                    double /*tolerance*/, Path& plan, double& cost,
                    std::string& /*message*/) override {
    plan.header.frame_id = "map";
    plan.poses.resize(2);
    cost = 1.0;
    return 0;
  }
  bool cancel() override { return true; }
};

class SoCostmapSmoother : public ayanami::costmap::CostmapSmoother {
 public:
  void initialize(const std::string& /*name*/,
                  ayanami::costmap::CostmapContext::Ptr /*ctx*/) override {}
  uint32_t smoothPath(const Path& in, Path& out, double /*max_duration*/,
                      bool /*check_for_collision*/,
                      std::string& /*message*/) override {
    out.header.frame_id = "map";
    out.poses = in.poses;
    return 0;
  }
  bool cancel() override { return true; }
};

class SoCostmapController : public ayanami::costmap::CostmapController {
 public:
  void initialize(const std::string& /*name*/,
                  ayanami::costmap::CostmapContext::Ptr /*ctx*/) override {}
  uint32_t computeVelocityCommands(const PoseStamped& /*pose*/,
                                   const TwistStamped& /*velocity*/,
                                   TwistStamped& cmd_vel,
                                   std::string& /*message*/) override {
    cmd_vel.twist.linear.x() = 0.3;
    return 0;
  }
  bool isGoalReached(double /*dist*/, double /*angle*/) override {
    return true;
  }
  bool setPlan(const Path& /*plan*/) override { return true; }
  bool cancel() override { return true; }
};

class SoCostmapRecovery : public ayanami::costmap::CostmapRecovery {
 public:
  void initialize(const std::string& /*name*/,
                  ayanami::costmap::CostmapContext::Ptr /*ctx*/) override {}
  uint32_t runRecovery(std::string& /*message*/) override { return 0; }
  bool cancel() override { return true; }
};

// 非 costmap 族 planner: 验证 initializePlannerPlugin dynamic_pointer_cast 失败。
class PlainPlanner : public ayanami::abstract::AbstractPlanner {
 public:
  uint32_t makePlan(const PoseStamped& /*start*/, const PoseStamped& /*goal*/,
                    double /*tolerance*/, Path& /*plan*/, double& /*cost*/,
                    std::string& /*message*/) override {
    return 58;
  }
  bool cancel() override { return true; }
};

}  // namespace

extern "C" ayanami::abstract::AbstractPlanner* create_test_costmap_planner_plugin() {
  return new SoCostmapPlanner;
}

extern "C" ayanami::abstract::AbstractSmoother* create_test_costmap_smoother_plugin() {
  return new SoCostmapSmoother;
}

extern "C" ayanami::abstract::AbstractController* create_test_costmap_controller_plugin() {
  return new SoCostmapController;
}

extern "C" ayanami::abstract::AbstractRecovery* create_test_costmap_recovery_plugin() {
  return new SoCostmapRecovery;
}

extern "C" ayanami::abstract::AbstractPlanner* create_plain_abstract_planner_plugin() {
  return new PlainPlanner;
}
