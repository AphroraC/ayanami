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

// Poll until the predicate holds or the deadline expires (test helper).
static bool waitUntilCondition(const std::function<bool()>& predicate,
                               std::chrono::milliseconds timeout =
                                   std::chrono::milliseconds(2000)) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (!predicate()) {
    if (std::chrono::steady_clock::now() > deadline) {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  return true;
}

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
 * @brief MockNavigationServer (same idea): request* returns fixed TaskHandles.
 * Adds a cmd_vel bypass: requestExecution emits on_velocity_command (simulating
 * the ControllerExecution cmd_vel output bypass). With exe_sync=false,
 * FollowPath stays pending as RUNNING (for preemption/halt verification).
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
    // cmd_vel bypass: simulates the controller velocity output (on_velocity_command)
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

  /// @brief In-place controller plan update (called by FollowPathNode after the SmoothPath replan bumps the version
  /// in the pilot tree; the mock accepts it directly, without simulating execution semantics).
  bool requestPlanUpdate(uint64_t, const ExePathGoal&) override { return true; }

  std::atomic<int> cancel_count{0};  // Declared before use, destroyed after (the servers cancel callbacks remain safe)

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

/// RAII thread running pilot.run: destruction stops first (kill_switch + wakeup) then joins,
/// so no thread is left hanging even when an assertion fails and exits early.
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

/// Temp config dir: config.json (global mapping) + costmap_navigation.json
/// (navigation section). planner_frequency uses 100Hz so the RateController
/// replans nearly every tick (async ComputePath rate-limited polling, minor 5).
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
  pilot.loadFromFile("", "");  // Empty path -> the embedded default XML
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

  // raw_path / smoothed_path in the blackboard within 100ms
  EXPECT_TRUE(waitUntil<Path>(
      bb, k_raw_path, [](const Path& p) { return !p.poses.empty(); }));
  EXPECT_TRUE(waitUntil<Path>(
      bb, k_smoothed_path, [](const Path& p) { return !p.poses.empty(); }));

  // session_id increments (pilot increments on every beginNavigation; applyPendingGoal writes the blackboard)
  uint64_t sid = 0;
  ASSERT_TRUE(bb->get<uint64_t>(k_session_id, sid));
  EXPECT_GE(sid, 1u);

  // plan_version changed: the server-side version increments after ComputePath completes
  EXPECT_GE(server->currentPlanVersion(), 1u);

  // The cmd_vel bypass fired (the mock requestExecution emits on_velocity_command)
  EXPECT_GE(vel_count.load(), 1);
}  // vel_cb destructor unsubscribes automatically

TEST(BehaviorTreePilotTest, newGoalRevisesTaskWithoutStoppingController) {
  const auto dir = makeConfigDir();
  auto server = std::make_shared<MockNavServer>();
  server->path_result = makePath(2);
  server->smooth_result = makePath(2);
  server->exe_sync = false;  // FollowPath stays pending as RUNNING

  BehaviorTreePilot pilot;
  pilot.configure(dir.string());
  pilot.loadFromFile("", "");
  pilot.setNavigationRuntime(server);

  pilot.beginNavigation(makePose(1.0, 2.0));
  PilotRunner runner(&pilot);

  const auto bb = pilot.blackboard();
  ASSERT_NE(bb, nullptr);

  // Wait for the old FollowPath task to be pending (RUNNING with the exe goal active)
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

  // The tree is not rebuilt: the same tree instance (address unchanged)
  EXPECT_EQ(&pilot.tree(), tree_addr);

  uint64_t sid_after = 0;
  ASSERT_TRUE(bb->get<uint64_t>(k_session_id, sid_after));
  EXPECT_EQ(sid_after, sid_before);

  EXPECT_TRUE(waitUntil<uint64_t>(bb, k_goal_revision,
      [&](uint64_t value) { return value > revision_before; }));

  // The new goal was written to the blackboard (pending_goal overwritten)
  EXPECT_TRUE(waitUntil<PoseStamped>(
      bb, k_goal,
      [&](const PoseStamped& g) {
        return g.pose.position.isApprox(new_goal.pose.position, 1e-6);
      }));
}

TEST(BehaviorTreePilotTest, emitsSessionNavigationResultOncePerTreeLife) {
  const auto dir = makeConfigDir();
  auto server = std::make_shared<MockNavServer>();
  // Everything settles synchronously so the tree reaches a terminal state
  // deterministically: planner fails with a structured code, recovery fails,
  // exe settles (unused while planning keeps failing).
  server->path_result = makePath(2);
  server->smooth_result = makePath(2);
  server->path_sync = true;
  server->path_failure = ErrorCode::PATH_NOT_FOUND;
  server->smooth_sync = true;
  server->exe_sync = true;
  server->exe_failure = ErrorCode::NO_VALID_COMMAND;
  server->recovery_sync = true;
  server->recovery_failure = ErrorCode::RECOVERY_FAILED;

  BehaviorTreePilot pilot;
  pilot.configure(dir.string());
  pilot.loadFromFile("", "");
  pilot.setNavigationRuntime(server);

  std::atomic<int> result_count{0};
  ayanami::actions::GoalStatus last_status = GoalStatus::PENDING;
  ErrorCode last_code = ErrorCode::NONE;
  ayanami::CallbackSubscription result_cb =
      ayanami::interfaces::NavigationCallbacks::on_navigation_result.subscribe(
          [&](const ayanami::actions::NavigationResult& result) {
            last_status = result.status;
            last_code = result.context.code;
            result_count.fetch_add(1);
          });

  pilot.beginNavigation(makePose(1.0, 2.0));
  PilotRunner runner(&pilot);

  // ABORTED emitted exactly once with a structured failure code (the exact
  // code depends on which branch failed last; it must never be NONE).
  EXPECT_TRUE(waitUntilCondition([&] { return result_count.load() >= 1; }));
  EXPECT_EQ(result_count.load(), 1);
  EXPECT_EQ(last_status, GoalStatus::ABORTED);
  EXPECT_NE(last_code, ErrorCode::NONE);

  // A new goal starts a new tree life; with everything succeeding and the
  // controller reporting reached, the outcome is SUCCEEDED (once).
  server->path_failure = ErrorCode::NONE;
  server->exe_failure = ErrorCode::NONE;
  server->exe_reached = true;
  pilot.beginNavigation(makePose(3.0, 4.0));
  EXPECT_TRUE(waitUntilCondition([&] { return result_count.load() >= 2; }));
  EXPECT_EQ(last_status, GoalStatus::SUCCEEDED);
  EXPECT_EQ(last_code, ErrorCode::NONE);
  EXPECT_LE(result_count.load(), 3);  // no storm of emissions
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

  // Wait for the old FollowPath task to be pending
  ASSERT_TRUE(waitForActiveGoal(server->exe_server, 1000));

  // After halt fires, all RUNNING actions are cancelled
  pilot.halt();
  EXPECT_TRUE(waitForInt(server->cancel_count, [](int v) { return v >= 1; }));
}
