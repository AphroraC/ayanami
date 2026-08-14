// Task 11 测试插件: NavigationServerBase 的 4 族插件工厂。
//
// 工厂符号按 spec §4.3 约定: extern "C" 无参构造返回 PluginType*。
// 通过 create_symbol 选择成功/失败行为 (fallback 旁路测试用失败 planner)。

#include <ayanami/core/abstract_controller.hpp>
#include <ayanami/core/abstract_planner.hpp>
#include <ayanami/core/abstract_recovery.hpp>
#include <ayanami/core/abstract_smoother.hpp>
#include <ayanami/messages/actions.hpp>

#include <cstdint>
#include <string>

namespace {

using ayanami::geometry_msgs::PoseStamped;
using ayanami::geometry_msgs::TwistStamped;
using ayanami::nav_msgs::Path;

// 成功 planner: 填充 2 个 pose 的路径并返回 0。
class SuccessPlanner : public ayanami::abstract::AbstractPlanner {
public:
  uint32_t makePlan(const PoseStamped& start, const PoseStamped& /*goal*/,
                    double /*tolerance*/, Path& plan, double& cost,
                    std::string& /*message*/) override {
    plan.header.frame_id = "map";
    plan.poses.push_back(start);
    plan.poses.push_back(start);
    cost = 2.0;
    return 0;
  }
  bool cancel() override { return true; }
};

// 失败 planner: 返回 NO_PATH_FOUND (spec §5.1 planner 段 58)。
class FailingPlanner : public ayanami::abstract::AbstractPlanner {
public:
  uint32_t makePlan(const PoseStamped& /*start*/, const PoseStamped& /*goal*/,
                    double /*tolerance*/, Path& /*plan*/, double& /*cost*/,
                    std::string& /*message*/) override {
    return ayanami::actions::GetPathResult::NO_PATH_FOUND;
  }
  bool cancel() override { return true; }
};

class SuccessSmoother : public ayanami::abstract::AbstractSmoother {
public:
  uint32_t smoothPath(const Path& in, Path& out, double /*max_duration*/,
                      bool /*check_for_collision*/,
                      std::string& /*message*/) override {
    out.header.frame_id = "map";
    out.poses = in.poses;
    return 0;
  }
  bool cancel() override { return true; }
};

class SuccessController : public ayanami::abstract::AbstractController {
public:
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

class SuccessRecovery : public ayanami::abstract::AbstractRecovery {
public:
  uint32_t runRecovery(std::string& /*message*/) override { return 0; }
  bool cancel() override { return true; }
};

}  // namespace

extern "C" ayanami::abstract::AbstractPlanner* create_test_planner_plugin() {
  return new SuccessPlanner;
}

extern "C" ayanami::abstract::AbstractPlanner* create_fail_planner_plugin() {
  return new FailingPlanner;
}

extern "C" ayanami::abstract::AbstractSmoother* create_test_smoother_plugin() {
  return new SuccessSmoother;
}

extern "C" ayanami::abstract::AbstractController*
create_test_controller_plugin() {
  return new SuccessController;
}

extern "C" ayanami::abstract::AbstractRecovery* create_test_recovery_plugin() {
  return new SuccessRecovery;
}
