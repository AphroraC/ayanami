// goal_update_follow_test 专用测试插件族。
//
// 一个源文件按 AYANAMI_GOAL_UPDATE_ROLE_{PLANNER,SMOOTHER,CONTROLLER} 展开
// 三个角色, 各生成一个 .so (每 .so 单一公共工厂 create_extension_module)。
//
// 跨 DSO 观测: 三个 .so 各自内嵌一个 Probe 单例, 经 extern "C"
// ayanami_goal_update_probe() 导出; 测试可执行文件 dlopen 同一路径后 dlsym
// 取指针, 直接断言插件内部行为 (makePlan goal / smoothed 末点 / setPlan 末点)。

#include <ayanami/abstract/core/abstract_controller.hpp>
#include <ayanami/abstract/core/abstract_planner.hpp>
#include <ayanami/abstract/core/abstract_smoother.hpp>
#include <ayanami/messages/failure.hpp>
#include <ayanami/types.hpp>
#include <ayanami/utility/extension_module.hpp>

#include <cmath>

namespace {

// 跨 DSO 探针 (POD 布局, 锁内仅原子写方; 测试侧只在 wait 轮询中读取)。
struct GoalUpdateProbe {
  int planner_calls{0};
  double planned_goal_x{0.0};
  double planned_goal_y{0.0};
  int smooth_calls{0};
  double smoothed_goal_x{0.0};
  double smoothed_goal_y{0.0};
  int set_plan_calls{0};
  double followed_goal_x{0.0};
  double followed_goal_y{0.0};
};

GoalUpdateProbe* probe() {
  static GoalUpdateProbe instance;
  return &instance;
}

}  // namespace

extern "C" GoalUpdateProbe* ayanami_goal_update_probe() { return probe(); }

#if defined(AYANAMI_GOAL_UPDATE_ROLE_PLANNER)

// 直线 planner: [start, goal] 两点路径 (真实使用 goal 入参)。
class StraightPlanner : public ayanami::abstract::AbstractPlanner {
 public:
  ayanami::PlannerResultT makePlan(
      const ayanami::geometry_msgs::PoseStamped& start,
      const ayanami::geometry_msgs::PoseStamped& goal,
      double /*tolerance*/) override {
    auto* p = probe();
    p->planner_calls++;
    p->planned_goal_x = goal.pose.position.x();
    p->planned_goal_y = goal.pose.position.y();
    ayanami::nav_msgs::Path plan;
    plan.header.frame_id = "map";
    plan.poses.push_back(start);
    plan.poses.push_back(goal);
    return ayanami::PlannerResultT::success(std::move(plan));
  }
  bool cancel() override { return true; }
};

extern "C" ayanami::ExtensionModule* create_extension_module() {
  return new StraightPlanner;
}

#elif defined(AYANAMI_GOAL_UPDATE_ROLE_SMOOTHER)

// 透传 smoother: in-place 语义 (直接修改传入路径), 记录平滑后的末点。
class ProbeSmoother : public ayanami::abstract::AbstractSmoother {
 public:
  ayanami::SmootherResultT smoothPath(
      ayanami::nav_msgs::Path& path, double /*max_duration*/,
      bool /*check_for_collision*/) override {
    auto* p = probe();
    p->smooth_calls++;
    if (!path.poses.empty()) {
      p->smoothed_goal_x = path.poses.back().pose.position.x();
      p->smoothed_goal_y = path.poses.back().pose.position.y();
    }
    path.header.frame_id = "map";
    return ayanami::SmootherResultT::success(std::move(path));
  }
  bool cancel() override { return true; }
};

extern "C" ayanami::ExtensionModule* create_extension_module() {
  return new ProbeSmoother;
}

#elif defined(AYANAMI_GOAL_UPDATE_ROLE_CONTROLLER)

// 记录型 controller: setPlan 记录跟踪路径末点 (到达判定由 execution 的
// goal_checker 负责, 插件自身永不判到达)。
class ProbeController : public ayanami::abstract::AbstractController {
 public:
  bool setPlan(const ayanami::nav_msgs::Path& plan) override {
    auto* p = probe();
    p->set_plan_calls++;
    if (!plan.poses.empty()) {
      p->followed_goal_x = plan.poses.back().pose.position.x();
      p->followed_goal_y = plan.poses.back().pose.position.y();
    }
    return true;
  }

  ayanami::ControllerResultT computeVelocityCommand(
      const ayanami::geometry_msgs::PoseStamped& /*pose*/,
      const ayanami::geometry_msgs::TwistStamped& /*velocity*/) override {
    ayanami::ControllerCommand cmd;
    cmd.velocity = ayanami::geometry_msgs::TwistStamped{};
    cmd.velocity.twist.linear.x() = 0.2;
    return ayanami::ControllerResultT::success(std::move(cmd));
  }
  bool cancel() override { return true; }
};

extern "C" ayanami::ExtensionModule* create_extension_module() {
  return new ProbeController;
}

#else
#error "define AYANAMI_GOAL_UPDATE_ROLE_{PLANNER,SMOOTHER,CONTROLLER}"
#endif
