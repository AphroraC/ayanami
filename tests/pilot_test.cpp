#include <gtest/gtest.h>

#include <ayanami/behavior_tree/blackboard_protocol.hpp>
#include <ayanami/behavior_tree/navigation.hpp>
#include <ayanami/behavior_tree/pilot.hpp>
#include <ayanami/abstract/components/navigation_server_base.hpp>
#include <ayanami/interfaces/callbacks.hpp>
#include <ayanami/messages/actions.hpp>
#include <ayanami/types.hpp>
#include <ayanami/messages/geometry_msgs.hpp>
#include <ayanami/messages/nav_msgs.hpp>

#include <behaviortree_cpp/bt_factory.h>

#include <Eigen/Geometry>

#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

using namespace ayanami::behavior_tree;
using ayanami::ErrorCode;
using ayanami::actions::ExePathGoal;
using ayanami::actions::ExePathResult;
using ayanami::actions::GetPathGoal;
using ayanami::actions::GetPathResult;
using ayanami::actions::GoalStatus;
using ayanami::actions::RecoveryGoal;
using ayanami::actions::RecoveryResult;
using ayanami::actions::SmoothPathGoal;
using ayanami::actions::SmoothPathResult;
using ayanami::components::TaskHandle;
using ayanami::geometry_msgs::PoseStamped;
using ayanami::nav_msgs::Path;

namespace {

PoseStamped makePose(double x, double y = 0.0) {
  PoseStamped p;
  p.header.frame_id = "map";
  p.pose = ayanami::geometry_msgs::Pose::fromIsometry(Eigen::Isometry3d::Identity());
  p.pose.position = Eigen::Vector3d(x, y, 0.0);
  return p;
}

Path makePath(size_t n) {
  Path p;
  p.header.frame_id = "map";
  for (size_t i = 0; i < n; ++i) {
    p.poses.push_back(makePose(static_cast<double>(i)));
  }
  return p;
}

/**
 * @brief MockNavigationServer (同 Task 13): request* 返回固定 TaskHandle。
 * 新增 cmd_vel 旁路: requestExecution 内 emit on_velocity_command (模拟
 * ControllerExecution 的 cmd_vel 输出旁路, spec §3.3)。exe_sync=false 时
 * FollowPath 挂起保持 RUNNING (供抢占/halt 验证)。
 */
class MockNavServer : public ayanami::components::NavigationServerBase {
 public:
  using PathServer = ayanami::components::ActionServer<
      GetPathGoal, GetPathResult, ayanami::actions::GetPathFeedback>;
  using SmoothServer = ayanami::components::ActionServer<
      SmoothPathGoal, SmoothPathResult, ayanami::actions::SmoothPathFeedback>;
  using ExeServer = ayanami::components::ActionServer<
      ExePathGoal, ExePathResult, ayanami::actions::ExePathFeedback>;
  using RecoveryServer = ayanami::components::ActionServer<
      RecoveryGoal, RecoveryResult, ayanami::actions::RecoveryFeedback>;

  MockNavServer() {
    path_server.setCancelCallback([this](uint64_t) {
      cancel_count.fetch_add(1);
    });
    smooth_server.setCancelCallback([this](uint64_t) {
      cancel_count.fetch_add(1);
    });
    exe_server.setCancelCallback([this](uint64_t) {
      cancel_count.fetch_add(1);
    });
    recovery_server.setCancelCallback([this](uint64_t) {
      cancel_count.fetch_add(1);
    });
  }

  TaskHandle<GetPathResult> requestPath(const GetPathGoal& goal) override {
    auto task = path_server.acceptGoal(goal);
    last_path_goal = goal;
    if (path_sync) {
      GetPathResult r;
      r.error = path_failure;
      r.path = path_result;
      r.session_id = goal.session_id;
      r.plan_version = goal.plan_version;
      path_server.publishResult(task.goal_id(), GoalStatus::SUCCEEDED, r);
    }
    return task;
  }

  TaskHandle<SmoothPathResult> requestSmoothing(
      const SmoothPathGoal& goal) override {
    auto task = smooth_server.acceptGoal(goal);
    last_smooth_goal = goal;
    if (smooth_sync) {
      SmoothPathResult r;
      r.error = smooth_failure;
      r.smoothed_path = smooth_result;
      r.session_id = goal.session_id;
      r.plan_version = goal.plan_version;
      smooth_server.publishResult(task.goal_id(), GoalStatus::SUCCEEDED, r);
    }
    return task;
  }

  TaskHandle<ExePathResult> requestExecution(const ExePathGoal& goal) override {
    auto task = exe_server.acceptGoal(goal);
    last_exe_goal = goal;
    // cmd_vel 旁路: 模拟 controller 输出速度 (spec §3.3 on_velocity_command)
    const ayanami::geometry_msgs::TwistStamped stop{};
    ayanami::interfaces::NavigationCallbacks::on_velocity_command(stop);
    if (exe_sync) {
      ExePathResult r;
      r.error = exe_failure;
      r.reached = exe_reached;
      r.session_id = goal.session_id;
      r.plan_version = goal.plan_version;
      exe_server.publishResult(task.goal_id(), GoalStatus::SUCCEEDED, r);
    }
    return task;
  }

  TaskHandle<RecoveryResult> requestRecovery(const RecoveryGoal& goal) override {
    auto task = recovery_server.acceptGoal(goal);
    last_recovery_goal = goal;
    if (recovery_sync) {
      RecoveryResult r;
      r.error = recovery_failure;
      r.session_id = goal.session_id;
      r.plan_version = goal.plan_version;
      recovery_server.publishResult(task.goal_id(), GoalStatus::SUCCEEDED, r);
    }
    return task;
  }

  /// @brief in-place 控制器规划更新 (pilot 树中 SmoothPath 重规划 bump version 后
  /// FollowPathNode 调用; mock 直接接受, 不模拟执行语义)。
  bool requestPlanUpdate(uint64_t, const ExePathGoal&) override { return true; }

  std::atomic<int> cancel_count{0};  // 先声明, 后析构 (servers 的 cancel 回调仍安全)

  bool path_sync = true;
  ErrorCode path_failure = ErrorCode::NONE;
  Path path_result;
  GetPathGoal last_path_goal;

  bool smooth_sync = true;
  ErrorCode smooth_failure = ErrorCode::NONE;
  Path smooth_result;
  SmoothPathGoal last_smooth_goal;

  bool exe_sync = true;
  ErrorCode exe_failure = ErrorCode::NONE;
  bool exe_reached = true;
  ExePathGoal last_exe_goal;

  bool recovery_sync = true;
  ErrorCode recovery_failure = ErrorCode::NONE;
  RecoveryGoal last_recovery_goal;

  PathServer path_server;
  SmoothServer smooth_server;
  ExeServer exe_server;
  RecoveryServer recovery_server;
};

// 运行 pilot.run() 的 RAII 线程: 析构先 stop() (kill_switch + 唤醒) 再 join,
// 保证断言失败提前退出时也不悬挂线程。
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

// 临时配置目录: config.json (global 映射) + costmap_navigation.json
// (navigation 段, spec §9.3)。planner_frequency 用 100Hz 让 RateController
// 近乎每 tick 重规划 (异步 ComputePath 限频轮询, Task 13 minor 5)。
std::filesystem::path makeConfigDir() {
  const auto dir =
      std::filesystem::temp_directory_path() / "ayanami_pilot_test";
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
        "behavior_tree_xml": "BT/navigation.xml",
        "behavior_tree_id": "NavigateWithReplanningAndSmoothing",
        "planner_id": "test_planner",
        "smoother_id": "test_smoother",
        "controller_id": "test_controller",
        "recovery_id": "test_recovery",
        "clear_costmap_id": "clear_costmap",
        "spin_id": "spin",
        "wait_id": "wait",
        "backup_id": "backup",
        "planner_frequency": 100.0,
        "smoother_frequency": -1.0,
        "controller_frequency": 20.0,
        "tick_frequency": 100.0
      }
    })";
  }
  return dir;
}

template <typename T>
bool waitUntil(const BT::Blackboard::Ptr& bb, const char* key,
               const std::function<bool(const T&)>& pred,
               int timeout_ms = 1000) {
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  T val;
  while (std::chrono::steady_clock::now() < deadline) {
    if (bb->get<T>(key, val) && pred(val)) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return false;
}

bool waitForInt(const std::atomic<int>& counter,
                const std::function<bool(int)>& pred, int timeout_ms = 1000) {
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    if (pred(counter.load())) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return false;
}

template <typename Server>
bool waitForActiveGoal(const Server& server, int timeout_ms = 1000) {
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    if (server.activeGoalCount() > 0) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return false;
}

}  // namespace

TEST(BehaviorTreePilotTest, basicNavigationWritesPathsBumpsSessionAndVelocity) {
  const auto dir = makeConfigDir();
  auto server = std::make_shared<MockNavServer>();
  server->path_result = makePath(2);
  server->smooth_result = makePath(2);

  BehaviorTreePilot pilot;
  pilot.configure(dir.string());
  pilot.loadFromFile("", "");  // 空路径 -> 内嵌默认 XML (spec §6.3)
  pilot.setNavigationRuntime(server);

  std::atomic<int> vel_count{0};
  ayanami::CallbackSubscription vel_cb =
      ayanami::interfaces::NavigationCallbacks::on_velocity_command.subscribe(
          [&vel_count](const ayanami::geometry_msgs::TwistStamped&) {
            vel_count.fetch_add(1);
          });

  pilot.beginNavigation(makePose(1.0, 2.0));
  PilotRunner runner(&pilot);

  const auto bb = pilot.blackboard();
  ASSERT_NE(bb, nullptr);

  // 100ms 内 raw_path / smoothed_path 入黑板
  EXPECT_TRUE(waitUntil<Path>(
      bb, k_raw_path, [](const Path& p) { return !p.poses.empty(); }));
  EXPECT_TRUE(waitUntil<Path>(
      bb, k_smoothed_path, [](const Path& p) { return !p.poses.empty(); }));

  // session_id 自增 (pilot 每次 beginNavigation 递增, applyPendingGoal 写黑板)
  uint64_t sid = 0;
  ASSERT_TRUE(bb->get<uint64_t>(k_session_id, sid));
  EXPECT_GE(sid, 1u);

  // plan_version 改变: ComputePath 完成后 server 侧版本自增
  EXPECT_GE(server->currentPlanVersion(), 1u);

  // cmd_vel 旁路被触发 (mock requestExecution 里 emit on_velocity_command)
  EXPECT_GE(vel_count.load(), 1);
}  // vel_cb 析构自动退订

TEST(BehaviorTreePilotTest, newGoalRevisesTaskWithoutStoppingController) {
  const auto dir = makeConfigDir();
  auto server = std::make_shared<MockNavServer>();
  server->path_result = makePath(2);
  server->smooth_result = makePath(2);
  server->exe_sync = false;  // FollowPath 挂起保持 RUNNING

  BehaviorTreePilot pilot;
  pilot.configure(dir.string());
  pilot.loadFromFile("", "");
  pilot.setNavigationRuntime(server);

  pilot.beginNavigation(makePose(1.0, 2.0));
  PilotRunner runner(&pilot);

  const auto bb = pilot.blackboard();
  ASSERT_NE(bb, nullptr);

  // 等到旧 FollowPath task 挂起 (RUNNING 且 exe goal active)
  ASSERT_TRUE(waitForActiveGoal(server->exe_server, 1000));

  uint64_t sid_before = 0;
  ASSERT_TRUE(bb->get<uint64_t>(k_session_id, sid_before));
  EXPECT_GE(sid_before, 1u);
  const void* tree_addr = &pilot.tree();

  uint64_t revision_before = 0;
  ASSERT_TRUE(bb->get<uint64_t>(k_goal_revision, revision_before));

  // New goal revises the active task. The controller keeps its current path
  // until a replacement path is available and the tree remains resident.
  const auto new_goal = makePose(5.0, 6.0);
  pilot.beginNavigation(new_goal);

  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  EXPECT_EQ(server->cancel_count.load(), 0);

  // 树不重建: 同一 tree 实例 (地址不变)
  EXPECT_EQ(&pilot.tree(), tree_addr);

  uint64_t sid_after = 0;
  ASSERT_TRUE(bb->get<uint64_t>(k_session_id, sid_after));
  EXPECT_EQ(sid_after, sid_before);

  EXPECT_TRUE(waitUntil<uint64_t>(bb, k_goal_revision,
      [&](uint64_t value) { return value > revision_before; }));

  // 新 goal 已写入黑板 (pending_goal 覆盖)
  EXPECT_TRUE(waitUntil<PoseStamped>(
      bb, k_goal,
      [&](const PoseStamped& g) {
        return g.pose.position.isApprox(new_goal.pose.position, 1e-6);
      }));
}

TEST(BehaviorTreePilotTest, haltCancelsRunningActions) {
  const auto dir = makeConfigDir();
  auto server = std::make_shared<MockNavServer>();
  server->path_result = makePath(2);
  server->smooth_result = makePath(2);
  server->exe_sync = false;

  BehaviorTreePilot pilot;
  pilot.configure(dir.string());
  pilot.loadFromFile("", "");
  pilot.setNavigationRuntime(server);

  pilot.beginNavigation(makePose(1.0, 2.0));
  PilotRunner runner(&pilot);

  // 等到旧 FollowPath task 挂起
  ASSERT_TRUE(waitForActiveGoal(server->exe_server, 1000));

  // halt() 触发后所有 RUNNING action 被取消
  pilot.halt();
  EXPECT_TRUE(waitForInt(server->cancel_count, [](int v) { return v >= 1; }));
}
