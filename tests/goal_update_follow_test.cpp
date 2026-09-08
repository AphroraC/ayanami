// goal_update_follow_test: end-to-end regression for delivering a new goal while navigation is in progress.
//
// Reproduces and locks the rviz-scenario bug ("the new goal was replanned, but the robot does not follow the new path"):
// a resident default tree + a real NavigationServerBase + real plugin execution threads; goal2 is delivered while the
// controller is tracking the goal1 path, asserting:
// 1. the planner immediately replans for goal2 (planned_goal updated);
// 2. the smoother re-smooths the new path (smoothed_goal updated) -- the fix: the smoother branch uses
// {path_id} as reset_token, so a newly landed path is re-smoothed immediately;
// 3. the controller catches up via in-place updatePlan (followed_goal updated, and the first
// controller task is not cancelled/rebuilt -- the set_plan count stays continuous).

#include <gtest/gtest.h>

#include <ayanami/abstract/components/navigation_server_base.hpp>
#include <ayanami/abstract/components/robot_state_controller.hpp>
#include <ayanami/abstract/core/abstract_goal_checker.hpp>
#include <ayanami/abstract/core/abstract_progress_checker.hpp>
#include <ayanami/behavior_tree/pilot.hpp>
#include <ayanami/utility/load_module.hpp>

#include <Eigen/Geometry>

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <thread>

#ifndef AYANAMI_TEST_BT_EXT_SO
#error "AYANAMI_TEST_BT_EXT_SO required"
#endif
#ifndef AYANAMI_TEST_GOAL_UPDATE_PLANNER_SO
#error "AYANAMI_TEST_GOAL_UPDATE_PLANNER_SO required"
#endif
#ifndef AYANAMI_TEST_GOAL_UPDATE_SMOOTHER_SO
#error "AYANAMI_TEST_GOAL_UPDATE_SMOOTHER_SO required"
#endif
#ifndef AYANAMI_TEST_GOAL_UPDATE_CONTROLLER_SO
#error "AYANAMI_TEST_GOAL_UPDATE_CONTROLLER_SO required"
#endif

namespace {

using ayanami::behavior_tree::BehaviorTreePilot;
using ayanami::components::ControllerExecution;
using ayanami::components::NavigationServerBase;
using ayanami::geometry_msgs::PoseStamped;

/// Field-by-field identical POD layout to the one in tests/plugins/goal_update_follow_plugins.cpp.
/// Each .so embeds its own Probe singleton (static instances are isolated per DSO) and writes only its own role fields;
/// the test takes a pointer from each of the three .so files and reads the respective fields.
struct GoalUpdateProbe {
  int planner_calls;
  double planned_goal_x;
  double planned_goal_y;
  int smooth_calls;
  double smoothed_goal_x;
  double smoothed_goal_y;
  int set_plan_calls;
  double followed_goal_x;
  double followed_goal_y;
};

GoalUpdateProbe* loadProbe(const std::string& so_path) {
  void* handle = ayanami::openSo(so_path);
  EXPECT_NE(handle, nullptr);
  auto* fn = reinterpret_cast<GoalUpdateProbe* (*)()>(
      ayanami::loadSymbol(handle, "ayanami_goal_update_probe"));
  EXPECT_NE(fn, nullptr);
  return fn ? fn() : nullptr;
}

// ---- Test-only checkers (implemented directly in the test binary, not loaded as plugins) ----

class NeverArrivedGoalChecker : public ayanami::abstract::AbstractGoalChecker {
 public:
  void reset() override {}
  bool isGoalReached(const PoseStamped&,
                     const PoseStamped&,
                     const ayanami::geometry_msgs::TwistStamped&) override {
    return false;
  }
  ayanami::abstract::GoalCheckerTolerances getTolerances() const override {
    return {};
  }
};

class AlwaysProgressChecker
    : public ayanami::abstract::AbstractProgressChecker {
 public:
  void reset() override {}
  bool check(const PoseStamped&) override { return true; }
};

// ---- Test server: loads the goal_update plugin family via the protected PluginManager ----

class GoalUpdateTestServer : public NavigationServerBase {
 public:
  void loadTestPlugins(const std::string& planner_so,
                       const std::string& smoother_so,
                       const std::string& controller_so) {
    default_planner = "straight_planner";
    default_smoother = "probe_smoother";
    default_controller = "probe_controller";
    planner_plugins.loadPlugin({default_planner, planner_so}, {});
    smoother_plugins.loadPlugin({default_smoother, smoother_so}, {});
    controller_plugins.loadPlugin({default_controller, controller_so}, {});
    createActions();
  }

 protected:
  void configureControllerExecution(
      std::shared_ptr<ControllerExecution> exec) const override {
    // Never arrives: FollowPath stays RUNNING to support mid-navigation goal update assertions.
    exec->setGoalChecker(goal_checker);
    exec->setProgressChecker(progress_checker);
    exec->setMaximumVelocity(0.5, 0.0, 1.5);
  }

 public:
  std::shared_ptr<NeverArrivedGoalChecker> goal_checker{
      std::make_shared<NeverArrivedGoalChecker>()};
  std::shared_ptr<AlwaysProgressChecker> progress_checker{
      std::make_shared<AlwaysProgressChecker>()};
};

PoseStamped makeGoal(double x, double y) {
  PoseStamped p;
  p.header.frame_id = "map";
  p.pose = ayanami::geometry_msgs::Pose::fromIsometry(
      Eigen::Isometry3d::Identity());
  p.pose.position = Eigen::Vector3d(x, y, 0.0);
  return p;
}

/// Temp config dir: only drives the pilot (BT extension + frequency). planner_frequency=-1 means one replan per goal
/// revision (production config semantics, precisely exposing the smoother stale-cache regression).
std::filesystem::path makeConfigDir() {
  const auto dir =
      std::filesystem::temp_directory_path() / "ayanami_goal_update_test";
  std::filesystem::remove_all(dir);
  std::filesystem::create_directories(dir);
  {
    std::ofstream ofs(dir / "config.json");
    ofs << R"({ "global": { "config_navigation": "costmap_navigation.json" } })";
  }
  {
    std::ofstream ofs(dir / "costmap_navigation.json");
    ofs << R"({
      "navigation": {
        "behavior_tree_extension": ")" AYANAMI_TEST_BT_EXT_SO R"(",
        "planner_frequency": -1.0,
        "smoother_frequency": -1.0,
        "controller_frequency": 20.0,
        "tick_frequency": 100.0
      }
    })";
  }
  return dir;
}

class PilotRunner {
 public:
  explicit PilotRunner(BehaviorTreePilot* pilot)
      : pilot(pilot), thread([this] { this->pilot->run(); }) {}
  ~PilotRunner() {
    pilot->stop();
    if (thread.joinable()) {
      thread.join();
    }
  }
  PilotRunner(const PilotRunner&) = delete;
  PilotRunner& operator=(const PilotRunner&) = delete;

 private:
  BehaviorTreePilot* pilot;
  std::thread thread;
};

bool waitForControllerFollow(const GoalUpdateProbe* controller_probe, double x,
                             double y, int min_set_plan_delta, int timeout_ms,
                             int set_plan_baseline) {
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    if (controller_probe->set_plan_calls - set_plan_baseline >=
            min_set_plan_delta &&
        std::abs(controller_probe->followed_goal_x - x) < 1e-6 &&
        std::abs(controller_probe->followed_goal_y - y) < 1e-6) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  return false;
}

}  // namespace

TEST(GoalUpdateFollowTest, controllerFollowsReplannedPathAfterGoalUpdate) {
  GoalUpdateProbe* planner_probe = loadProbe(AYANAMI_TEST_GOAL_UPDATE_PLANNER_SO);
  GoalUpdateProbe* smoother_probe = loadProbe(AYANAMI_TEST_GOAL_UPDATE_SMOOTHER_SO);
  GoalUpdateProbe* controller_probe =
      loadProbe(AYANAMI_TEST_GOAL_UPDATE_CONTROLLER_SO);
  ASSERT_NE(planner_probe, nullptr);
  ASSERT_NE(smoother_probe, nullptr);
  ASSERT_NE(controller_probe, nullptr);
  // Probe static counters live per DSO and accumulate across tests in this process: assertions always use relative baseline deltas.
  const int planner_base = planner_probe->planner_calls;
  const int smooth_base = smoother_probe->smooth_calls;
  const int set_plan_base = controller_probe->set_plan_calls;

  // Robot state: pose = origin (map<-odom<-base_link all identity); velocity = zero.
  auto& robot_state = ayanami::components::RobotStateController::instance();
  robot_state.initialize("map", "odom", "base_link", 0.25);
  robot_state.setTransform("map", "odom", ayanami::Timestamp::latest(),
                           Eigen::Isometry3d::Identity(), true);
  robot_state.setTransform("odom", "base_link", ayanami::Timestamp::latest(),
                           Eigen::Isometry3d::Identity(), true);

  auto server = std::make_shared<GoalUpdateTestServer>();
  server->loadTestPlugins(AYANAMI_TEST_GOAL_UPDATE_PLANNER_SO,
                          AYANAMI_TEST_GOAL_UPDATE_SMOOTHER_SO,
                          AYANAMI_TEST_GOAL_UPDATE_CONTROLLER_SO);
  server->start();

  BehaviorTreePilot pilot;
  const auto dir = makeConfigDir();
  pilot.configure(dir.string());
  pilot.loadFromFile("", "");  // Empty path -> the embedded production default tree
  pilot.setNavigationRuntime(server);

  {
    PilotRunner runner(&pilot);
    pilot.beginNavigation(makeGoal(5.0, 0.0));
    // 1) First goal: the controller catches up with the goal1 path.
    ASSERT_TRUE(waitForControllerFollow(controller_probe, 5.0, 0.0, 1, 5000,
                                        set_plan_base))
        << "controller never followed the first plan";

    // 2) Deliver a new goal mid-navigation (rviz scenario): the controller is not cancelled and the robot keeps
    // following the old path until the new path is available.
    pilot.beginNavigation(makeGoal(0.0, 5.0));

    // 3) The planner immediately replans for goal2.
    const auto replan_deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(2000);
    while (std::chrono::steady_clock::now() < replan_deadline &&
           !(std::abs(planner_probe->planned_goal_x - 0.0) < 1e-6 &&
             std::abs(planner_probe->planned_goal_y - 5.0) < 1e-6)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    ASSERT_NEAR(planner_probe->planned_goal_x, 0.0, 1e-6);
    ASSERT_NEAR(planner_probe->planned_goal_y, 5.0, 1e-6);
    EXPECT_EQ(planner_probe->planner_calls - planner_base, 2);

    // 4) The smoother re-smooths the new path (fix-point regression assertion).
    const auto smooth_deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(2000);
    while (std::chrono::steady_clock::now() < smooth_deadline &&
           !(std::abs(smoother_probe->smoothed_goal_x - 0.0) < 1e-6 &&
             std::abs(smoother_probe->smoothed_goal_y - 5.0) < 1e-6)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    EXPECT_NEAR(smoother_probe->smoothed_goal_x, 0.0, 1e-6);
    EXPECT_NEAR(smoother_probe->smoothed_goal_y, 5.0, 1e-6);
    EXPECT_GE(smoother_probe->smooth_calls - smooth_base, 2);

    // 5) Core regression assertion: the controller catches up with the goal2 path via the in-place update (instead of
    // tracking the old goal1 path forever).
    EXPECT_TRUE(waitForControllerFollow(controller_probe, 0.0, 5.0, 1, 5000,
                                        set_plan_base + 1))
        << "controller did not follow the replanned path after goal update";
  }

  server->stop();
}

TEST(GoalUpdateFollowTest, plannerAndSmootherFireOncePerFreshPlan) {
  GoalUpdateProbe* planner_probe = loadProbe(AYANAMI_TEST_GOAL_UPDATE_PLANNER_SO);
  GoalUpdateProbe* smoother_probe = loadProbe(AYANAMI_TEST_GOAL_UPDATE_SMOOTHER_SO);
  GoalUpdateProbe* controller_probe =
      loadProbe(AYANAMI_TEST_GOAL_UPDATE_CONTROLLER_SO);
  ASSERT_NE(planner_probe, nullptr);
  ASSERT_NE(smoother_probe, nullptr);
  ASSERT_NE(controller_probe, nullptr);
  const int planner_base = planner_probe->planner_calls;
  const int smooth_base = smoother_probe->smooth_calls;
  const int set_plan_base = controller_probe->set_plan_calls;

  auto& robot_state = ayanami::components::RobotStateController::instance();
  robot_state.initialize("map", "odom", "base_link", 0.25);
  robot_state.setTransform("map", "odom", ayanami::Timestamp::latest(),
                           Eigen::Isometry3d::Identity(), true);
  robot_state.setTransform("odom", "base_link", ayanami::Timestamp::latest(),
                           Eigen::Isometry3d::Identity(), true);

  auto server = std::make_shared<GoalUpdateTestServer>();
  server->loadTestPlugins(AYANAMI_TEST_GOAL_UPDATE_PLANNER_SO,
                          AYANAMI_TEST_GOAL_UPDATE_SMOOTHER_SO,
                          AYANAMI_TEST_GOAL_UPDATE_CONTROLLER_SO);
  server->start();

  BehaviorTreePilot pilot;
  const auto dir = makeConfigDir();
  pilot.configure(dir.string());
  pilot.loadFromFile("", "");
  pilot.setNavigationRuntime(server);

  {
    PilotRunner runner(&pilot);
    pilot.beginNavigation(makeGoal(3.0, 0.0));
    // planner_frequency=-1: one plan + one smooth for the first goal, then no repetition.
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(3000);
    while (std::chrono::steady_clock::now() < deadline &&
           controller_probe->set_plan_calls - set_plan_base < 1) {
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    EXPECT_EQ(planner_probe->planner_calls - planner_base, 1);
    EXPECT_EQ(smoother_probe->smooth_calls - smooth_base, 1);
  }

  server->stop();
}
