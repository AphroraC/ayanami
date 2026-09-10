// Test module family dedicated to goal_update_follow_test.
//
// One source file expanded by AYANAMI_GOAL_UPDATE_ROLE_{PLANNER,SMOOTHER,CONTROLLER} into
// three roles, each producing one .so (a single common factory create_extension_module per .so).
//
// Cross-DSO observation: each of the three .so files embeds its own Probe singleton, exported via extern "C"
// ayanami_goal_update_probe; the test executable dlopens the same path, dlsyms
// the pointer, and directly asserts module-internal behavior (makePlan goal / smoothed endpoint / setPlan endpoint).

#include <ayanami/abstract/core/abstract_controller.hpp>
#include <ayanami/abstract/core/abstract_planner.hpp>
#include <ayanami/abstract/core/abstract_smoother.hpp>
#include <ayanami/messages/failure.hpp>
#include <ayanami/types.hpp>
#include <ayanami/utility/extension_module.hpp>

#include <cmath>

namespace {

/// Cross-DSO probe (POD layout, only atomic writers under the lock; the test side reads only inside wait polling).
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

/// Straight-line planner: a [start, goal] two-point path (really uses the goal input).
class StraightPlanner : public ayanami::abstract::AbstractPlanner {
 public:
  ayanami::ResultT<void> makePlan(
      const ayanami::geometry_msgs::PoseStamped& start,
      const ayanami::geometry_msgs::PoseStamped& goal,
      double /*tolerance*/, ayanami::nav_msgs::Path& plan_out) override {
    auto* p = probe();
    p->planner_calls++;
    p->planned_goal_x = goal.pose.position.x();
    p->planned_goal_y = goal.pose.position.y();
    ayanami::nav_msgs::Path plan;
    plan.header.frame_id = "map";
    plan.poses.push_back(start);
    plan.poses.push_back(goal);
    plan_out = std::move(plan);
    return ayanami::ResultT<void>::success();
  }
  bool cancel() override { return true; }
};

extern "C" ayanami::ExtensionModule* create_extension_module() {
  return new StraightPlanner;
}

#elif defined(AYANAMI_GOAL_UPDATE_ROLE_SMOOTHER)

/// Pass-through smoother: in-place semantics (modifies the given path directly), records the smoothed endpoint.
class ProbeSmoother : public ayanami::abstract::AbstractSmoother {
 public:
  ayanami::ResultT<void> smoothPath(ayanami::nav_msgs::Path& path,
                             double /*max_duration*/) override {
    auto* p = probe();
    p->smooth_calls++;
    if (!path.poses.empty()) {
      p->smoothed_goal_x = path.poses.back().pose.position.x();
      p->smoothed_goal_y = path.poses.back().pose.position.y();
    }
    path.header.frame_id = "map";
    return ayanami::ResultT<void>::success();
  }
  bool cancel() override { return true; }
};

extern "C" ayanami::ExtensionModule* create_extension_module() {
  return new ProbeSmoother;
}

#elif defined(AYANAMI_GOAL_UPDATE_ROLE_CONTROLLER)

/// Recording controller: setPlan records the tracked path endpoint (arrival is decided by the execution
/// goal_checker; the module itself never declares arrival).
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

  ayanami::ResultT<void> computeVelocityCommand(
      const ayanami::geometry_msgs::PoseStamped& /*pose*/,
      const ayanami::geometry_msgs::TwistStamped& /*velocity*/, ayanami::geometry_msgs::TwistStamped& cmd) override {
    cmd = ayanami::geometry_msgs::TwistStamped{};
    cmd.twist.linear.x() = 0.2;
    return ayanami::ResultT<void>::success();
  }
  bool cancel() override { return true; }
};

extern "C" ayanami::ExtensionModule* create_extension_module() {
  return new ProbeController;
}

#else
#error "define AYANAMI_GOAL_UPDATE_ROLE_{PLANNER,SMOOTHER,CONTROLLER}"
#endif
