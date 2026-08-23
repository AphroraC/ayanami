#include <gtest/gtest.h>

#include <ayanami/behavior_tree/blackboard_protocol.hpp>
#include <ayanami/behavior_tree/navigation.hpp>
#include <ayanami/behavior_tree/pilot.hpp>
#include <ayanami/abstract/components/navigation_server_base.hpp>
#include <ayanami/interfaces/callbacks.hpp>
#include <ayanami/messages/actions.hpp>
#include <ayanami/messages/geometry_msgs.hpp>
#include <ayanami/messages/nav_msgs.hpp>

#include <behaviortree_cpp/bt_factory.h>

#include <Eigen/Geometry>

#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <functional>
#include <string>
#include <thread>
#include <vector>

using namespace ayanami::behavior_tree;
using ayanami::FailureCode;
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
  p.pose = Eigen::Isometry3d::Identity();
  p.pose.translation() = Eigen::Vector3d(x, y, 0.0);
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
 * ControllerExecution 的 cmd_vel 输出旁路, spec §3.3)。exe_sync_=false 时
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
    path_server_.setCancelCallback([this](uint64_t) {
      cancel_count.fetch_add(1);
    });
    smooth_server_.setCancelCallback([this](uint64_t) {
      cancel_count.fetch_add(1);
    });
    exe_server_.setCancelCallback([this](uint64_t) {
      cancel_count.fetch_add(1);
    });
    recovery_server_.setCancelCallback([this](uint64_t) {
      cancel_count.fetch_add(1);
    });
  }

  TaskHandle<GetPathResult> requestPath(const GetPathGoal& goal) override {
    auto task = path_server_.acceptGoal(goal);
    last_path_goal_ = goal;
    if (path_sync_) {
      GetPathResult r;
      r.failure = path_failure_;
      r.path = path_result_;
      r.session_id = goal.session_id;
      r.plan_version = goal.plan_version;
      path_server_.publishResult(task.goal_id(), GoalStatus::SUCCEEDED, r);
    }
    return task;
  }

  TaskHandle<SmoothPathResult> requestSmoothing(
      const SmoothPathGoal& goal) override {
    auto task = smooth_server_.acceptGoal(goal);
    last_smooth_goal_ = goal;
    if (smooth_sync_) {
      SmoothPathResult r;
      r.failure = smooth_failure_;
      r.smoothed_path = smooth_result_;
      r.session_id = goal.session_id;
      r.plan_version = goal.plan_version;
      smooth_server_.publishResult(task.goal_id(), GoalStatus::SUCCEEDED, r);
    }
    return task;
  }

  TaskHandle<ExePathResult> requestExecution(const ExePathGoal& goal) override {
    auto task = exe_server_.acceptGoal(goal);
    last_exe_goal_ = goal;
    // cmd_vel 旁路: 模拟 controller 输出速度 (spec §3.3 on_velocity_command)
    ayanami::interfaces::NavigationCallbacks::on_velocity_command(
        ayanami::geometry_msgs::TwistStamped{});
    if (exe_sync_) {
      ExePathResult r;
      r.failure = exe_failure_;
      r.reached = exe_reached_;
      r.session_id = goal.session_id;
      r.plan_version = goal.plan_version;
      exe_server_.publishResult(task.goal_id(), GoalStatus::SUCCEEDED, r);
    }
    return task;
  }

  TaskHandle<RecoveryResult> requestRecovery(const RecoveryGoal& goal) override {
    auto task = recovery_server_.acceptGoal(goal);
    last_recovery_goal_ = goal;
    if (recovery_sync_) {
      RecoveryResult r;
      r.failure = recovery_failure_;
      r.session_id = goal.session_id;
      r.plan_version = goal.plan_version;
      recovery_server_.publishResult(task.goal_id(), GoalStatus::SUCCEEDED, r);
    }
    return task;
  }

  std::atomic<int> cancel_count{0};  // 先声明, 后析构 (servers 的 cancel 回调仍安全)

  bool path_sync_ = true;
  FailureCode path_failure_ = FailureCode::None;
  Path path_result_;
  GetPathGoal last_path_goal_;

  bool smooth_sync_ = true;
  FailureCode smooth_failure_ = FailureCode::None;
  Path smooth_result_;
  SmoothPathGoal last_smooth_goal_;

  bool exe_sync_ = true;
  FailureCode exe_failure_ = FailureCode::None;
  bool exe_reached_ = true;
  ExePathGoal last_exe_goal_;

  bool recovery_sync_ = true;
  FailureCode recovery_failure_ = FailureCode::None;
  RecoveryGoal last_recovery_goal_;

  PathServer path_server_;
  SmoothServer smooth_server_;
  ExeServer exe_server_;
  RecoveryServer recovery_server_;
};

// 运行 pilot_.run() 的 RAII 线程: 析构先 stop() (kill_switch_ + 唤醒) 再 join,
// 保证断言失败提前退出时也不悬挂线程。
class PilotRunner {
 public:
  explicit PilotRunner(BehaviorTreePilot* pilot)
      : pilot_(pilot), thread_([this] { pilot_->run(); }) {}

  ~PilotRunner() {
    pilot_->stop();
    if (thread_.joinable()) {
      thread_.join();
    }
  }

  PilotRunner(const PilotRunner&) = delete;
  PilotRunner& operator=(const PilotRunner&) = delete;

 private:
  BehaviorTreePilot* pilot_;
  std::thread thread_;
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

TEST(BehaviorTreePilotTest, BasicNavigationWritesPathsBumpsSessionAndVelocity) {
  const auto dir = makeConfigDir();
  MockNavServer server;
  server.path_result_ = makePath(2);
  server.smooth_result_ = makePath(2);

  BehaviorTreePilot pilot;
  pilot.configure(dir.string());
  pilot.loadFromFile("", "");  // 空路径 -> 内嵌默认 XML (spec §6.3)
  pilot.setNavigationServer(&server);

  std::atomic<int> vel_count{0};
  const int vel_id =
      ayanami::interfaces::NavigationCallbacks::on_velocity_command.add(
          [&vel_count](const ayanami::geometry_msgs::TwistStamped&) {
            vel_count.fetch_add(1);
          });

  pilot.beginNavigation(makePose(1.0, 2.0));
  PilotRunner runner(&pilot);

  const auto bb = pilot.blackboard();
  ASSERT_NE(bb, nullptr);

  // 100ms 内 raw_path / smoothed_path 入黑板
  EXPECT_TRUE(waitUntil<Path>(
      bb, kRawPath, [](const Path& p) { return !p.poses.empty(); }));
  EXPECT_TRUE(waitUntil<Path>(
      bb, kSmoothedPath, [](const Path& p) { return !p.poses.empty(); }));

  // session_id 自增 (pilot 每次 beginNavigation 递增, applyPendingGoal 写黑板)
  uint64_t sid = 0;
  ASSERT_TRUE(bb->get<uint64_t>(kSessionId, sid));
  EXPECT_GE(sid, 1u);

  // plan_version 改变: ComputePath 完成后 server 侧版本自增
  EXPECT_GE(server.currentPlanVersion(), 1u);

  // cmd_vel 旁路被触发 (mock requestExecution 里 emit on_velocity_command)
  EXPECT_GE(vel_count.load(), 1);

  ayanami::interfaces::NavigationCallbacks::on_velocity_command.remove(vel_id);
}

TEST(BehaviorTreePilotTest, NewGoalPreemptsOldTaskWithoutRebuildingTree) {
  const auto dir = makeConfigDir();
  MockNavServer server;
  server.path_result_ = makePath(2);
  server.smooth_result_ = makePath(2);
  server.exe_sync_ = false;  // FollowPath 挂起保持 RUNNING

  BehaviorTreePilot pilot;
  pilot.configure(dir.string());
  pilot.loadFromFile("", "");
  pilot.setNavigationServer(&server);

  pilot.beginNavigation(makePose(1.0, 2.0));
  PilotRunner runner(&pilot);

  const auto bb = pilot.blackboard();
  ASSERT_NE(bb, nullptr);

  // 等到旧 FollowPath task 挂起 (RUNNING 且 exe goal active)
  ASSERT_TRUE(waitForActiveGoal(server.exe_server_, 1000));

  uint64_t sid_before = 0;
  ASSERT_TRUE(bb->get<uint64_t>(kSessionId, sid_before));
  EXPECT_GE(sid_before, 1u);
  const void* tree_addr = &pilot.tree();

  // 新 goal 抢占 (spec §6.1): 树 halt 取消旧 RUNNING task + session_id_++ +
  // pending_goal 覆盖; 树常驻不重建
  const auto new_goal = makePose(5.0, 6.0);
  pilot.beginNavigation(new_goal);

  // 旧 task 被 cancel (haltTree -> onHalted -> task.cancel 透传到 cancel callback)
  EXPECT_TRUE(waitForInt(server.cancel_count, [](int v) { return v >= 1; }));

  // 树不重建: 同一 tree 实例 (地址不变)
  EXPECT_EQ(&pilot.tree(), tree_addr);

  // session_id_ 再次自增
  EXPECT_TRUE(waitUntil<uint64_t>(
      bb, kSessionId,
      [&](uint64_t v) { return v > sid_before; }));

  // plan_version 清零 (applyPendingGoal 每轮导航重写黑板)
  uint64_t pv = 42;
  ASSERT_TRUE(bb->get<uint64_t>(kPlanVersion, pv));
  EXPECT_EQ(pv, 0u);

  // 新 goal 已写入黑板 (pending_goal 覆盖)
  EXPECT_TRUE(waitUntil<PoseStamped>(
      bb, kGoal,
      [&](const PoseStamped& g) {
        return g.pose.translation().isApprox(new_goal.pose.translation(), 1e-6);
      }));
}

TEST(BehaviorTreePilotTest, HaltCancelsRunningActions) {
  const auto dir = makeConfigDir();
  MockNavServer server;
  server.path_result_ = makePath(2);
  server.smooth_result_ = makePath(2);
  server.exe_sync_ = false;

  BehaviorTreePilot pilot;
  pilot.configure(dir.string());
  pilot.loadFromFile("", "");
  pilot.setNavigationServer(&server);

  pilot.beginNavigation(makePose(1.0, 2.0));
  PilotRunner runner(&pilot);

  // 等到旧 FollowPath task 挂起
  ASSERT_TRUE(waitForActiveGoal(server.exe_server_, 1000));

  // halt() 触发后所有 RUNNING action 被取消
  pilot.halt();
  EXPECT_TRUE(waitForInt(server.cancel_count, [](int v) { return v >= 1; }));
}
