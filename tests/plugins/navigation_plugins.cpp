// Task 11 测试插件: NavigationServerBase 的 4 族插件工厂。
//
// 工厂符号按 spec §4.3 约定: extern "C" 无参构造返回 PluginType*。
// 通过 create_symbol 选择成功/失败行为 (fallback 旁路测试用失败 planner)。

#include <ayanami/abstract/core/abstract_controller.hpp>
#include <ayanami/abstract/core/abstract_goal_checker.hpp>
#include <ayanami/abstract/core/abstract_planner.hpp>
#include <ayanami/abstract/core/abstract_recovery.hpp>
#include <ayanami/abstract/core/abstract_smoother.hpp>
#include <ayanami/messages/failure.hpp>

#include <string>

namespace {

using ayanami::geometry_msgs::PoseStamped;
using ayanami::FailureCode;

using ayanami::geometry_msgs::TwistStamped;
using ayanami::nav_msgs::Path;

// 成功 planner: 填充 2 个 pose 的路径并返回 None。
class SuccessPlanner : public ayanami::abstract::AbstractPlanner {
public:
  FailureCode makePlan(const PoseStamped& start, const PoseStamped& /*goal*/,
                       double /*tolerance*/, Path& plan, double& cost,
                       std::string& /*message*/) override {
    plan.header.frame_id = "map";
    plan.poses.push_back(start);
    plan.poses.push_back(start);
    cost = 2.0;
    return FailureCode::None;
  }
  bool cancel() override { return true; }
};

// 失败 planner: 返回统一失败码 PathNotFound。
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

class SuccessSmoother : public ayanami::abstract::AbstractSmoother {
public:
  FailureCode smoothPath(const Path& in, Path& out, double /*max_duration*/,
                         bool /*check_for_collision*/,
                         std::string& /*message*/) override {
    out.header.frame_id = "map";
    out.poses = in.poses;
    return FailureCode::None;
  }
  bool cancel() override { return true; }
};

class SuccessController : public ayanami::abstract::AbstractController {
public:
  FailureCode computeVelocityCommands(const PoseStamped& /*pose*/,
                                      const TwistStamped& /*velocity*/,
                                      TwistStamped& cmd_vel,
                                      std::string& /*message*/,
                                      const ayanami::abstract::AbstractGoalChecker::Ptr& /*goal_checker*/) override {
    cmd_vel.twist.linear.x() = 0.3;
    return FailureCode::None;
  }
  bool setPlan(const Path& /*plan*/) override { return true; }
  bool cancel() override { return true; }
};

// 永不到达目标的控制器: 让 ControllerExecution 循环运行 (checker 注入测试用)。
class LoopingController : public ayanami::abstract::AbstractController {
public:
  FailureCode computeVelocityCommands(const PoseStamped& /*pose*/,
                                      const TwistStamped& /*velocity*/,
                                      TwistStamped& cmd_vel,
                                      std::string& /*message*/,
                                      const ayanami::abstract::AbstractGoalChecker::Ptr& /*goal_checker*/) override {
    cmd_vel.twist.linear.x() = 0.1;
    return FailureCode::None;
  }
  bool setPlan(const Path& /*plan*/) override { return true; }
  bool cancel() override { return true; }
};

class SuccessRecovery : public ayanami::abstract::AbstractRecovery {
public:
  FailureCode runRecovery(std::string& /*message*/) override {
    return FailureCode::None;
  }
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

extern "C" ayanami::abstract::AbstractController*
create_looping_controller_plugin() {
  return new LoopingController;
}

extern "C" ayanami::abstract::AbstractRecovery* create_test_recovery_plugin() {
  return new SuccessRecovery;
}
