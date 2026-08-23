// Task 20 测试插件: CostmapNavigationServer 的 4 族 costmap 插件工厂。
//
// 工厂符号按 spec §4.3 约定: extern "C" 无参构造返回 PluginType*。
// create_plain_abstract_planner_plugin 返回非 costmap 族 planner,
// 用于验证 initializePlannerPlugin 的 dynamic_pointer_cast 失败路径。

#include <ayanami/abstract/core/abstract_controller.hpp>
#include <ayanami/abstract/core/abstract_goal_checker.hpp>
#include <ayanami/abstract/core/abstract_planner.hpp>
#include <ayanami/abstract/core/abstract_recovery.hpp>
#include <ayanami/abstract/core/abstract_smoother.hpp>
#include <ayanami/costmap/components/costmap_context.hpp>
#include <ayanami/costmap/core/costmap_controller.hpp>
#include <ayanami/costmap/core/costmap_planner.hpp>
#include <ayanami/costmap/core/costmap_recovery.hpp>
#include <ayanami/costmap/core/costmap_smoother.hpp>
#include <ayanami/messages/failure.hpp>
#include <ayanami/messages/geometry_msgs.hpp>
#include <ayanami/messages/nav_msgs.hpp>

#include <string>

namespace {

using ayanami::geometry_msgs::PoseStamped;
using ayanami::FailureCode;

using ayanami::geometry_msgs::TwistStamped;
using ayanami::nav_msgs::Path;

class SoCostmapPlanner : public ayanami::costmap::CostmapPlanner {
 public:
  void initialize(const std::string& /*name*/,
                  ayanami::costmap::CostmapContext::Ptr /*ctx*/) override {}
  FailureCode makePlan(const PoseStamped& /*start*/, const PoseStamped& /*goal*/,
                       double /*tolerance*/, Path& plan, double& cost,
                       std::string& /*message*/) override {
    plan.header.frame_id = "map";
    plan.poses.resize(2);
    cost = 1.0;
    return FailureCode::None;
  }
  bool cancel() override { return true; }
};

class SoCostmapSmoother : public ayanami::costmap::CostmapSmoother {
 public:
  void initialize(const std::string& /*name*/,
                  ayanami::costmap::CostmapContext::Ptr /*ctx*/) override {}
  FailureCode smoothPath(const Path& in, Path& out, double /*max_duration*/,
                         bool /*check_for_collision*/,
                         std::string& /*message*/) override {
    out.header.frame_id = "map";
    out.poses = in.poses;
    return FailureCode::None;
  }
  bool cancel() override { return true; }
};

class SoCostmapController : public ayanami::costmap::CostmapController {
 public:
  void initialize(const std::string& /*name*/,
                  ayanami::costmap::CostmapContext::Ptr /*ctx*/) override {}
  FailureCode computeVelocityCommands(
      const PoseStamped& /*pose*/, const TwistStamped& /*velocity*/,
      TwistStamped& cmd_vel, std::string& /*message*/,
      const ayanami::abstract::AbstractGoalChecker::Ptr& /*goal_checker*/)
      override {
    cmd_vel.twist.linear.x() = 0.3;
    return FailureCode::None;
  }
  bool setPlan(const Path& /*plan*/) override { return true; }
  bool cancel() override { return true; }
};

class SoCostmapRecovery : public ayanami::costmap::CostmapRecovery {
 public:
  void initialize(const std::string& /*name*/,
                  ayanami::costmap::CostmapContext::Ptr /*ctx*/) override {}
  FailureCode runRecovery(std::string& /*message*/) override {
    return FailureCode::None;
  }
  bool cancel() override { return true; }
};

// 非 costmap 族 planner: 验证 initializePlannerPlugin dynamic_pointer_cast 失败。
class PlainPlanner : public ayanami::abstract::AbstractPlanner {
 public:
  FailureCode makePlan(const PoseStamped& /*start*/, const PoseStamped& /*goal*/,
                       double /*tolerance*/, Path& /*plan*/, double& /*cost*/,
                       std::string& /*message*/) override {
    return FailureCode::PathNotFound;
  }
  bool cancel() override { return true; }
};

// 非 costmap 族 smoother/controller/recovery: 验证对应 initialize*Plugin 拒绝。
class PlainSmoother : public ayanami::abstract::AbstractSmoother {
 public:
  FailureCode smoothPath(const Path& /*in*/, Path& /*out*/, double /*max_duration*/,
                         bool /*check_for_collision*/,
                         std::string& /*message*/) override {
    return FailureCode::InvalidInput;
  }
  bool cancel() override { return true; }
};

class PlainController : public ayanami::abstract::AbstractController {
 public:
  FailureCode computeVelocityCommands(
      const PoseStamped& /*pose*/, const TwistStamped& /*velocity*/,
      TwistStamped& cmd_vel, std::string& /*message*/,
      const ayanami::abstract::AbstractGoalChecker::Ptr& /*goal_checker*/)
      override {
    cmd_vel.twist.linear.x() = 0.1;
    return FailureCode::None;
  }
  bool setPlan(const Path& /*plan*/) override { return true; }
  bool cancel() override { return true; }
};

class PlainRecovery : public ayanami::abstract::AbstractRecovery {
 public:
  FailureCode runRecovery(std::string& /*message*/) override {
    return FailureCode::Canceled;
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

extern "C" ayanami::abstract::AbstractSmoother*
create_plain_abstract_smoother_plugin() {
  return new PlainSmoother;
}

extern "C" ayanami::abstract::AbstractController*
create_plain_abstract_controller_plugin() {
  return new PlainController;
}

extern "C" ayanami::abstract::AbstractRecovery*
create_plain_abstract_recovery_plugin() {
  return new PlainRecovery;
}
