// goal_update_follow_test: 导航进行中下发新 goal 的端到端回归测试。
//
// 复现并锁死 rviz 场景 bug ("新 goal 已重新规划, 但机器人不跟随新路径"):
// 常驻默认树 + 真实 NavigationServerBase + 真实插件执行线程, 在 controller
// 跟踪 goal1 路径途中投递 goal2, 断言:
//   1. planner 立即按 goal2 重规划 (planned_goal 更新);
//   2. smoother 对新路径重新平滑 (smoothed_goal 更新) —— 修复点: smoother 分支
//      以 {path_id} 为 reset_token, 新路径落地即重平滑;
//   3. controller 经 in-place updatePlan 跟上 (followed_goal 更新, 且首个
//      controller 任务未被 cancel/重建 —— set_plan 连续计数)。

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

// 与 tests/plugins/goal_update_follow_plugins.cpp 内 POD 布局逐字段一致。
// 每个 .so 内嵌独立 Probe 单例 (静态实例按 DSO 隔离), 只写本角色字段;
// 测试侧从三个 .so 分别取指针, 各读各的字段。
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

// ---- 测试专用 checker (直接在测试二进制内实现, 不经插件装载) ----

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

// ---- 测试服务器: 经保护 PluginManager 直接装载 goal_update 插件族 ----

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
    // 永不到达: FollowPath 持续 RUNNING, 供中途 goal 更新断言。
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

// 临时配置目录: 只驱动 pilot (BT 扩展 + 频率)。planner_frequency=-1 = 每次目标
// 修订只重规划一次 (生产配置语义, 精确暴露 smoother 陈旧缓存回归)。
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
  // probe 静态计数按 DSO 存活且跨本进程测试累积: 断言一律用相对基线增量。
  const int planner_base = planner_probe->planner_calls;
  const int smooth_base = smoother_probe->smooth_calls;
  const int set_plan_base = controller_probe->set_plan_calls;

  // 机器人状态: 位姿 = 原点 (map←odom←base_link 全 identity); 速度 = 零。
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
  pilot.loadFromFile("", "");  // 空路径 -> 内嵌生产默认树
  pilot.setNavigationRuntime(server);

  {
    PilotRunner runner(&pilot);
    pilot.beginNavigation(makeGoal(5.0, 0.0));
    // 1) 首个 goal: controller 跟上 goal1 路径。
    ASSERT_TRUE(waitForControllerFollow(controller_probe, 5.0, 0.0, 1, 5000,
                                        set_plan_base))
        << "controller never followed the first plan";

    // 2) 导航进行中投递新 goal (rviz 场景): 不取消 controller, 机器人持续
    //    跟旧路径直到新路径可用。
    pilot.beginNavigation(makeGoal(0.0, 5.0));

    // 3) planner 立即按 goal2 重规划。
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

    // 4) smoother 对新路径重新平滑 (修复点回归断言)。
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

    // 5) 核心回归断言: controller 经 in-place 更新跟上 goal2 路径 (而非永远
    //    跟踪 goal1 旧路径)。
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
    // planner_frequency=-1: 首个 goal 一次规划 + 一次平滑后不再重复。
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
