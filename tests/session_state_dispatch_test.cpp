// Focused tests (session/version single source + failure context + plugin id + in-place
// controller update + dispatch by id + config read only during configure).
//
// Coverage:
// A. A shared NavigationSessionState single source (pilot and server see the same session;
// the tick thread applyPendingGoal opens a new session and zeroes plan_version).
// B. A BT action node writes a precise FailureContext before failing; RecoveryGoal.cause carries the triggering
// context.
// C. The pilot resets the failure_context blackboard per session.
// D. GetPathGoal/SmoothPathGoal/ExePathGoal plugin_id: filled with the default when empty, preserved when explicit.
// E. Controller in-place plan update end to end (FollowPathNode -> requestPlanUpdate ->
// ControllerAction::updatePlan): no cancel / no new Execution / no intermediate stop,
// and the final result carries the newest plan_version.
// F. NavigationServerBase dispatches multi-plugin configs by id; an invalid id returns a structured failure
// TaskHandle; requests do not re-read the config (config is read only during configure/load).
//
// argv: 1=test_planner_plugin.so, 2=test_smoother_plugin.so,
// 3=test_controller_plugin.so, 4=test_recovery_plugin.so

#include <gtest/gtest.h>

#include <ayanami/abstract/components/controller_action.hpp>
#include <ayanami/abstract/components/navigation_server_base.hpp>
#include <ayanami/abstract/components/robot_state_controller.hpp>
#include <ayanami/abstract/components/navigation_session_state.hpp>
#include <ayanami/abstract/core/abstract_controller.hpp>
#include <ayanami/abstract/core/abstract_goal_checker.hpp>
#include <ayanami/abstract/core/abstract_progress_checker.hpp>
#include <ayanami/behavior_tree/action_nodes.hpp>
#include <ayanami/behavior_tree/blackboard_protocol.hpp>
#include <ayanami/behavior_tree/navigation.hpp>
#include <ayanami/behavior_tree/pilot.hpp>
#include <ayanami/interfaces/callbacks.hpp>
#include <ayanami/messages/actions.hpp>
#include <ayanami/messages/failure.hpp>
#include <ayanami/types.hpp>
#include <ayanami/messages/geometry_msgs.hpp>
#include <ayanami/messages/nav_msgs.hpp>
#include <ayanami/utility/config.hpp>

#include <behaviortree_cpp/bt_factory.h>

#include <Eigen/Geometry>

#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

using namespace ayanami::behavior_tree;
using ayanami::ErrorCode;
using ayanami::ResultT;
using ayanami::FailureContext;
using ayanami::FailureStage;
using ayanami::actions::ExePathGoal;
using ayanami::actions::ExePathResult;
using ayanami::actions::GetPathGoal;
using ayanami::actions::GetPathResult;
using ayanami::actions::GoalStatus;
using ayanami::actions::RecoveryGoal;
using ayanami::actions::RecoveryResult;
using ayanami::actions::SmoothPathGoal;
using ayanami::actions::SmoothPathResult;
using ayanami::components::NavigationServerBase;
using ayanami::components::NavigationSessionState;
using ayanami::components::TaskHandle;
using ayanami::geometry_msgs::PoseStamped;
using ayanami::geometry_msgs::TwistStamped;
using ayanami::nav_msgs::Path;

namespace {

std::string g_planner_so;
std::string g_smoother_so;
std::string g_controller_so;
std::string g_recovery_so;

std::filesystem::path makeTempRoot(const std::string& name) {
  std::filesystem::path dir = std::filesystem::temp_directory_path() / name;
  std::filesystem::remove_all(dir);
  std::filesystem::create_directories(dir);
  return dir;
}

void writeJson(const std::filesystem::path& path,
                const std::string& content) {
  std::ofstream ofs(path);
  ofs << content;
}

PoseStamped makePose(double x, double y = 0.0) {
  PoseStamped p;
  p.header.frame_id = "map";
  p.pose.position = Eigen::Vector3d(x, y, 0.0);
  return p;
}

Path makePath(std::size_t n) {
  Path p;
  p.header.frame_id = "map";
  for (std::size_t i = 0; i < n; ++i) {
    p.poses.push_back(makePose(static_cast<double>(i), 0.0));
  }
  return p;
}

void setNavServer(const BT::Blackboard::Ptr& bb,
                  ayanami::components::NavigationRuntime::Ptr server) {
  bb->set<ayanami::components::NavigationRuntime::Ptr>(k_navigation_runtime,
                                                       server);
}

void setRequiredEntries(const BT::Blackboard::Ptr& bb) {
  bb->set<PoseStamped>(k_goal, makePose(0.0));
  bb->set<Path>(k_raw_path, Path{});
  bb->set<Path>(k_smoothed_path, Path{});
  bb->set<double>(k_planner_frequency, 10.0);
  bb->set<double>(k_smoother_frequency, -1.0);
  bb->set<double>(k_controller_frequency, 20.0);
  bb->set<std::string>(k_clear_costmap_id, "clear_costmap");
  bb->set<std::string>(k_spin_id, "spin");
  bb->set<std::string>(k_wait_id, "wait");
  bb->set<std::string>(k_backup_id, "backup");
  bb->set<uint64_t>(k_session_id, 0u);
  bb->set<uint64_t>(k_plan_version, 0u);
  bb->set<uint64_t>(k_goal_revision, 0u);
  bb->set<FailureContext>(k_failure_context, FailureContext{});
}

template <typename Fn>
bool waitUntil(const Fn& fn, int timeout_ms = 3000) {
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    if (fn()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return fn();
}

template <typename Task>
bool waitReady(Task& t, int max_ms = 3000) {
  for (int i = 0; i < max_ms / 10 && !t.ready(); ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return t.ready();
}

// ---------- Mock NavigationServer (pilot/BT-driven tests) ----------

class MockNavServer : public NavigationServerBase {
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
    path_server.setCancelCallback([this](uint64_t) { cancel_count.fetch_add(1); });
    smooth_server.setCancelCallback([this](uint64_t) { cancel_count.fetch_add(1); });
    exe_server.setCancelCallback([this](uint64_t) { cancel_count.fetch_add(1); });
    recovery_server.setCancelCallback([this](uint64_t) {
      cancel_count.fetch_add(1);
    });
  }

  TaskHandle<GetPathResult> requestPath(const GetPathGoal& goal) override {
    ++path_requests;
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
    ++exe_requests;
    auto task = exe_server.acceptGoal(goal);
    last_exe_goal = goal;
    last_exe_goal_id = task.goal_id();
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

  /// @brief Manually publishes the ExePath result (completes the in-flight task in async mode, for race tests).
  void publishExeResult(ErrorCode error, bool reached,
                        uint64_t plan_version) {
    ExePathResult r;
    r.error = error;
    r.reached = reached;
    r.session_id = currentSessionId();
    r.plan_version = plan_version;
    exe_server.publishResult(last_exe_goal_id, GoalStatus::SUCCEEDED, r);
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

  // Tunable: requestPlanUpdate returns false to simulate an update miss (the execution is already terminal/released).
  bool requestPlanUpdate(uint64_t, const ExePathGoal&) override {
    return plan_update_ok;
  }

  std::atomic<int> cancel_count{0};
  int path_requests{0};
  int exe_requests{0};

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
  RecoveryGoal last_recovery_goal;
  bool recovery_sync = true;
  ErrorCode recovery_failure = ErrorCode::NONE;
  bool exe_reached = true;
  ExePathGoal last_exe_goal;
  uint64_t last_exe_goal_id{0};
  bool plan_update_ok{true};

  PathServer path_server;
  SmoothServer smooth_server;
  ExeServer exe_server;
  RecoveryServer recovery_server;
};

// ---------- Pilot run RAII (focused pilot tests) ----------

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

std::filesystem::path makePilotConfigDir() {
  const auto dir = makeTempRoot("ayanami_session_dispatch_pilot");
  writeJson(dir / "config.json",
             "{ \"global\": { \"config_navigation\": \"nav.json\" } }");
  writeJson(dir / "nav.json", std::string(R"({
    "navigation": {
      "behavior_tree_extension": ")") + AYANAMI_TEST_BT_EXT_SO + R"(",
      "behavior_tree_id": "NavigateWithReplanningAndSmoothing",


      "planner_frequency": 100.0,
      "smoother_frequency": -1.0,
      "controller_frequency": 20.0,
      "tick_frequency": 100.0
    }
  })");
  return dir;
}

// ---------- Test server components ----------

/// Checker stubs: immediate arrival / always progressing.
class ImmediateGoalChecker : public ayanami::abstract::AbstractGoalChecker {
 public:
  void reset() override {}
  bool isGoalReached(const PoseStamped&, const PoseStamped&,
                     const ayanami::geometry_msgs::TwistStamped&) override {
    return true;
  }
  ayanami::abstract::GoalCheckerTolerances getTolerances() const override {
    return {};
  }
};
class AlwaysTrueProgressChecker
    : public ayanami::abstract::AbstractProgressChecker {
 public:
  void reset() override {}
  bool check(const PoseStamped&) override { return true; }
};

// ---------- Dispatch test server (multiple configs loaded + records dispatch) ----------

class DispatchServer : public NavigationServerBase {
 public:
  bool initializePlannerPlugin(
      const std::string& name, const ayanami::Config& /*module_config*/,
      ayanami::abstract::AbstractPlanner::Ptr p) override {
    init_planners.push_back(name);
    return p != nullptr;
  }
  bool initializeSmootherPlugin(
      const std::string& name, const ayanami::Config& /*module_config*/,
      ayanami::abstract::AbstractSmoother::Ptr p) override {
    init_smoothers.push_back(name);
    return p != nullptr;
  }
  bool initializeControllerPlugin(
      const std::string& name, const ayanami::Config& /*module_config*/,
      ayanami::abstract::AbstractController::Ptr p) override {
    init_controllers.push_back(name);
    return p != nullptr;
  }
  bool initializeRecoveryPlugin(
      const std::string& name, const ayanami::Config& /*module_config*/,
      ayanami::abstract::AbstractRecovery::Ptr p) override {
    init_recoveries.push_back(name);
    return p != nullptr;
  }

  std::unique_ptr<ayanami::components::PlannerExecution> newPlannerExecution(
      const std::string& name,
      ayanami::abstract::AbstractPlanner::Ptr p) override {
    factory_planners.push_back(name);
    return NavigationServerBase::newPlannerExecution(name, std::move(p));
  }
  std::unique_ptr<ayanami::components::ControllerExecution>
  newControllerExecution(const std::string& name,
                         ayanami::abstract::AbstractController::Ptr p) override {
    factory_controllers.push_back(name);
    // Injects a checker so requestExecution can complete normally (immediate arrival + always progressing).
    auto exec = NavigationServerBase::newControllerExecution(name, std::move(p));
    exec->setGoalChecker(std::make_shared<ImmediateGoalChecker>());
    exec->setProgressChecker(std::make_shared<AlwaysTrueProgressChecker>());
    return exec;
  }

  std::vector<std::string> init_planners, init_smoothers, init_controllers,
      init_recoveries;
  std::vector<std::string> factory_planners, factory_controllers;
};

std::string pluginJson(const std::string& module, const std::string& name,
                        const std::string& so_name) {
  return "{\n  \"" + module + "\": {\n    \"name\": \"" + name +
         "\",\n    \"so_name\": \"" + so_name + "\"\n  }\n}\n";
}

std::filesystem::path makeDispatchConfigDir() {
  const auto dir = makeTempRoot("ayanami_session_dispatch_cfg");
  writeJson(dir / "config.json",
             "{\n  \"global\": {\n"
             "    \"config_navigation\": \"nav.json\",\n"
             "    \"config_planner\": [\"planner_a.json\", \"planner_b.json\"],\n"
             "    \"config_smoother\": [\"smoother_a.json\", \"smoother_b.json\"],\n"
             "    \"config_controller\": [\"controller_a.json\", \"controller_b.json\"],\n"
             "    \"config_recovery\": [\"recovery_a.json\"]\n"
             "  }\n}\n");
  writeJson(dir / "nav.json", R"({
    "navigation": {


      "recovery_id": "recovery_a"
    }
  })");
  writeJson(dir / "planner_a.json",
             pluginJson("planner", "planner_a", g_planner_so));
  writeJson(dir / "planner_b.json",
             pluginJson("planner", "planner_b", g_planner_so));
  writeJson(dir / "smoother_a.json",
             pluginJson("smoother", "smoother_a", g_smoother_so));
  writeJson(dir / "smoother_b.json",
             pluginJson("smoother", "smoother_b", g_smoother_so));
  writeJson(dir / "controller_a.json",
             pluginJson("controller", "controller_a", g_controller_so));
  writeJson(dir / "controller_b.json",
             pluginJson("controller", "controller_b", g_controller_so));
  writeJson(dir / "recovery_a.json",
             "{\n  \"recovery\": {\n    \"name\": \"recovery_a\",\n"
             "    \"role\": \"clear_costmap\",\n    \"so_name\": \"" +
                 g_recovery_so + "\"\n  }\n}\n");
  return dir;
}

// ---------- In-place controller update test components ----------

/// A controller that never arrives + keeps emitting velocity (arrival can be triggered via the goal_checker).
class LoopingController : public ayanami::abstract::AbstractController {
 public:
  ResultT<void> computeVelocityCommand(const PoseStamped&,
                                           const TwistStamped&, ayanami::geometry_msgs::TwistStamped& cmd) override {
    cmd = TwistStamped{};
    cmd.twist.linear.x() = 0.3;
    return ResultT<void>::success();
  }
  bool setPlan(const Path&) override { return true; }
  bool cancel() override { return true; }
};

/// Controllable goal check: declares arrival only once ready is set true.
class FlagGoalChecker : public ayanami::abstract::AbstractGoalChecker {
 public:
  void reset() override {}
  bool isGoalReached(const PoseStamped&, const PoseStamped&,
                     const ayanami::geometry_msgs::TwistStamped&) override {
    return ready.load();
  }
  ayanami::abstract::GoalCheckerTolerances getTolerances() const override {
    return {};
  }
  std::atomic<bool> ready{false};
};

/// A ControllerAction with checker injection + execution counting (exposes the createExecution override).
class InPlaceControllerAction : public ayanami::components::ControllerAction {
 public:
  InPlaceControllerAction(ayanami::abstract::AbstractController::Ptr controller,
                          std::shared_ptr<FlagGoalChecker> goal,
                          std::shared_ptr<AlwaysTrueProgressChecker> progress)
      : ControllerAction(std::move(controller)),
        goal(std::move(goal)),
        progress(std::move(progress)) {}

 protected:
  ayanami::components::ControllerExecution::Ptr createExecution() override {
    ++executions;
    auto exec = std::make_shared<ayanami::components::ControllerExecution>(
        controller);
    exec->setGoalChecker(goal);
    exec->setProgressChecker(progress);
    return exec;
  }

 public:
  std::atomic<int> executions{0};

 private:
  std::shared_ptr<FlagGoalChecker> goal;
  std::shared_ptr<AlwaysTrueProgressChecker> progress;
};

/// A server with a real ControllerAction (for the FollowPathNode in-place end-to-end).
class FollowServer : public NavigationServerBase {
 public:
  std::shared_ptr<FlagGoalChecker> goal_checker;
  InPlaceControllerAction* controller_action_ptr = nullptr;

  void installController() {
    default_controller = "test_controller";
    goal_checker = std::make_shared<FlagGoalChecker>();
    auto progress = std::make_shared<AlwaysTrueProgressChecker>();
    auto controller = std::make_shared<LoopingController>();
    auto action = std::unique_ptr<InPlaceControllerAction>(
        new InPlaceControllerAction(controller, goal_checker, progress));
    controller_action_ptr = action.get();
    controller_action = std::move(action);
  }
};

constexpr const char* k_follow_xml = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="main">
    <FollowPath path="{smoothed_path}"/>
  </BehaviorTree>
</root>)";

// ========== A. Shared session/version single source + new goal zeroing ==========

TEST(SessionStateDispatchTest, pilotAndServerShareSingleSessionState) {
  const auto dir = makePilotConfigDir();
  auto server = std::make_shared<MockNavServer>();
  server->path_result = makePath(2);
  server->smooth_result = makePath(2);
  server->exe_sync = false;  // FollowPath stays pending as RUNNING

  BehaviorTreePilot pilot;
  pilot.configure(dir.string());
  pilot.loadFromFile("", "");
  pilot.setNavigationRuntime(server);

  // Injected: the server and pilot share the same NavigationSessionState object.
  ASSERT_EQ(server->sessionState().get(), pilot.sessionState().get());

  pilot.beginNavigation(makePose(1.0, 2.0));
  PilotRunner runner(&pilot);

  const auto bb = pilot.blackboard();
  ASSERT_NE(bb, nullptr);

  // The tick thread applyPendingGoal opens a new session: blackboard session == server session == 1.
  ASSERT_TRUE(waitUntil([&] {
    uint64_t sid = 0;
    return bb->get<uint64_t>(k_session_id, sid) && sid >= 1;
  }));
  uint64_t sid = 0;
  ASSERT_TRUE(bb->get<uint64_t>(k_session_id, sid));
  EXPECT_EQ(server->currentSessionId(), sid);
  EXPECT_TRUE(server->sessionIsCurrent(sid));

  // New goals revise the same active session; controller continuity is kept.
  server->incrementPlanVersion();
  server->incrementPlanVersion();
  EXPECT_GE(server->currentPlanVersion(), 2u);

  pilot.beginNavigation(makePose(5.0, 6.0));
  EXPECT_TRUE(waitUntil([&] {
    uint64_t v = 0;
    return bb->get<uint64_t>(k_goal_revision, v) && v > 0;
  }));

  EXPECT_EQ(server->currentSessionId(), sid);
  uint64_t pv = 42;
  ASSERT_TRUE(bb->get<uint64_t>(k_plan_version, pv));
  EXPECT_GE(server->currentPlanVersion(), pv);
}

// ========== B. BT nodes write the FailureContext before failing; RecoveryGoal.cause carries it ==========

TEST(SessionStateDispatchTest, failureContextWrittenAndRecoveryCarriesCause) {
  auto server = std::make_shared<MockNavServer>();
  server->path_sync = true;
  server->path_failure = ErrorCode::PATH_NOT_FOUND;

  auto bb = BT::Blackboard::create();
  setNavServer(bb, server);
  setRequiredEntries(bb);

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<ComputePathToPoseNode>("ComputePathToPose");
  factory.registerNodeType<RecoveryActionNode>("Recovery");
  auto tree = factory.createTreeFromText(R"(
<root BTCPP_format="4">
  <BehaviorTree ID="main">
    <ComputePathToPose goal="{goal}" path="{raw_path}"/>
  </BehaviorTree>
</root>)", bb);

  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::FAILURE);

  FailureContext ctx;
  ASSERT_TRUE(bb->get<FailureContext>(k_failure_context, ctx));
  EXPECT_EQ(ctx.stage, FailureStage::Planner);
  EXPECT_EQ(ctx.code, ErrorCode::PATH_NOT_FOUND);
  EXPECT_EQ(ctx.plugin_id, "planner");
  EXPECT_EQ(ctx.session_id, server->currentSessionId());
  EXPECT_EQ(ctx.plan_version, server->currentPlanVersion());

  // RecoveryGoal.cause carries the same context (the Recovery node reads the blackboard failure_context).
  auto rec_tree = factory.createTreeFromText(R"(
<root BTCPP_format="4">
  <BehaviorTree ID="main">
    <Recovery recovery="{spin_id}"/>
  </BehaviorTree>
</root>)", bb);

  EXPECT_EQ(rec_tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(rec_tree.tickExactlyOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(server->last_recovery_goal.recovery_id, "spin");
  EXPECT_EQ(server->last_recovery_goal.cause.stage, FailureStage::Planner);
  EXPECT_EQ(server->last_recovery_goal.cause.code,
            ErrorCode::PATH_NOT_FOUND);
  EXPECT_EQ(server->last_recovery_goal.cause.plugin_id, "planner");
}

// ========== C. The pilot resets failure_context per session ==========

TEST(SessionStateDispatchTest, pilotResetsFailureContextPerSession) {
  const auto dir = makePilotConfigDir();
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

  const auto bb = pilot.blackboard();
  ASSERT_NE(bb, nullptr);
  uint64_t sid = 0;
  ASSERT_TRUE(waitUntil([&] {
    return bb->get<uint64_t>(k_session_id, sid) && sid >= 1;
  }));

  // A revised goal clears the previous failure context without changing session.
  FailureContext bogus;
  bogus.stage = FailureStage::Controller;
  bogus.code = ErrorCode::TIMEOUT;
  bogus.plugin_id = "controller_a";
  bb->set<FailureContext>(k_failure_context, bogus);

  pilot.beginNavigation(makePose(9.0, 9.0));
  ASSERT_TRUE(waitUntil([&] {
    uint64_t v = 0;
    return bb->get<uint64_t>(k_goal_revision, v) && v > 0;
  }));

  FailureContext reset;
  ASSERT_TRUE(bb->get<FailureContext>(k_failure_context, reset));
  EXPECT_EQ(reset.stage, FailureStage::Unknown);
  EXPECT_EQ(reset.code, ErrorCode::NONE);
  EXPECT_TRUE(reset.plugin_id.empty());
  EXPECT_TRUE(reset.message.empty());
}

/* Legacy multi-plugin dispatch tests are intentionally removed: the public
 * navigation model has one planner, smoother and controller per server. */
#if 0
TEST(SessionStateDispatchTest, pluginIdDefaultedOnlyWhenEmpty) {
  auto dir = makeDispatchConfigDir();
  auto server = std::make_shared<DispatchServer>();
  server->loadPlugins(dir.string());

  {  // Empty plugin_id -> filled with the default planner_a and dispatched to it
    GetPathGoal goal;
    goal.goal = makePose(1.0, 0.0);
    auto task = server->requestPath(goal);
    ASSERT_TRUE(waitReady(task));
    EXPECT_EQ(task.status(), GoalStatus::SUCCEEDED);
  }
  // The empty id was not swallowed: default dispatch = default_planner (navigation.planner_id).
  EXPECT_EQ(server->init_planners, (std::vector<std::string>{
                                       "planner_a", "planner_b"}));

  {  // Explicit id preserved (not overwritten with the default); dispatched to the named plugin
    GetPathGoal goal;
    goal.goal = makePose(1.0, 0.0);
    goal.plugin_id = "planner_b";
    auto task = server->requestPath(goal);
    ASSERT_TRUE(waitReady(task));
    EXPECT_EQ(task.status(), GoalStatus::SUCCEEDED);
    EXPECT_EQ(server->factory_planners.back(), "planner_b");
  }
  {  // Empty id again -> default dispatch
    SmoothPathGoal goal;
    goal.path = makePath(2);
    auto task = server->requestSmoothing(goal);
    ASSERT_TRUE(waitReady(task));
    EXPECT_EQ(task.status(), GoalStatus::SUCCEEDED);
  }
  {  // Explicit controller id preserved
    ExePathGoal goal;
    goal.path = makePath(2);
    goal.plugin_id = "controller_b";
    auto task = server->requestExecution(goal);
    ASSERT_TRUE(waitReady(task));
    EXPECT_EQ(task.status(), GoalStatus::SUCCEEDED);
    EXPECT_EQ(server->factory_controllers.back(), "controller_b");
  }
}
#endif

// ========== E. Controller in-place plan update end to end ==========

TEST(SessionStateDispatchTest, controllerInPlacePlanUpdateEndToEnd) {
  auto& robot_state = ayanami::components::RobotStateController::instance();
  robot_state.initialize("map", "odom", "base_link", 10.0);
  robot_state.setTransform("map", "base_link", ayanami::Timestamp::latest(),
                           Eigen::Isometry3d::Identity());
  auto server = std::make_shared<FollowServer>();
  server->installController();

  // Observes cmd_vel: records zero velocities (cancel/arrival publish the stop command).
  std::atomic<int> zero_cmds{0};
  int zero_cb = ayanami::interfaces::NavigationCallbacks::on_velocity_command.add(
      [&](const TwistStamped& t) {
        if (t.twist.linear.norm() < 1e-9 && t.twist.angular.norm() < 1e-9) {
          zero_cmds.fetch_add(1);
        }
      });

  auto bb = BT::Blackboard::create();
  setNavServer(bb, server);
  setRequiredEntries(bb);
  bb->set<Path>(k_smoothed_path, makePath(2));

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<FollowPathNode>("FollowPath");
  auto tree = factory.createTreeFromText(k_follow_xml, bb);

  // tick 1: onStart -> submits the v0 path; the real ControllerAction starts executing.
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(server->controller_action_ptr->executions.load(), 1);
  std::this_thread::sleep_for(std::chrono::milliseconds(30));

  // plan_version changed: FollowPathNode takes the in-place update (requestPlanUpdate).
  bb->set<Path>(k_smoothed_path, makePath(4));
  server->incrementPlanVersion();
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);

  // Assert: no cancel / no new Execution / no intermediate stop.
  EXPECT_EQ(server->controller_action_ptr->executions.load(), 1);
  EXPECT_EQ(zero_cmds.load(), 0);  // No intermediate zero velocity during the update phase (in-place, no stop)

  // Arrival flag on -> the execution arrives after consuming setNewPlan; the final result carries the newest plan_version.
  server->goal_checker->ready.store(true);
  ASSERT_TRUE(waitUntil([&] { return zero_cmds.load() >= 1; }));  // Final stop

  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  for (int i = 0; i < 50 && status != BT::NodeStatus::SUCCESS; ++i) {
    status = tree.tickExactlyOnce();
    if (status == BT::NodeStatus::RUNNING) {
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  }
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);

  // No intermediate stops: the only zero velocity in the whole run is the final arrival one.
  EXPECT_EQ(zero_cmds.load(), 1);
  // A single Execution throughout (no cancel/join/rebuild).
  EXPECT_EQ(server->controller_action_ptr->executions.load(), 1);

  ayanami::interfaces::NavigationCallbacks::on_velocity_command.remove(zero_cb);
}

// Recovery remains the only runtime-selectable plugin family.
#if 0
TEST(SessionStateDispatchTest, dispatchByPluginIdAndInvalidIsStructuredFailure) {
  auto dir = makeDispatchConfigDir();
  auto server = std::make_shared<DispatchServer>();
  server->loadPlugins(dir.string());

  // multiple loaded configs: 2 loaded for the whole family (1 recovery).
  ASSERT_EQ(server->init_planners.size(), 2u);
  ASSERT_EQ(server->init_smoothers.size(), 2u);
  ASSERT_EQ(server->init_controllers.size(), 2u);
  ASSERT_EQ(server->init_recoveries.size(), 1u);
  ASSERT_NE(server->plannerAction("planner_a"), nullptr);
  ASSERT_NE(server->plannerAction("planner_b"), nullptr);
  ASSERT_NE(server->smootherAction("smoother_b"), nullptr);
  ASSERT_NE(server->controllerAction("controller_b"), nullptr);
  // The no-argument accessor = the action of the default id (navigation.*_id).
  EXPECT_EQ(server->plannerAction(), server->plannerAction("planner_a"));
  EXPECT_EQ(server->controllerAction(), server->controllerAction("controller_a"));
  EXPECT_EQ(server->smootherAction(), server->smootherAction("smoother_a"));

  // Invalid id -> a structured failure TaskHandle (valid + ABORTED + InvalidInput), not a silent
  // fallback to the default.
  auto assertRejected = [&](const TaskHandle<GetPathResult>& task) {
    ASSERT_TRUE(task.valid());
    ASSERT_TRUE(waitReady(task, 200));
    EXPECT_EQ(task.status(), GoalStatus::ABORTED);
    EXPECT_EQ(task.result().error, ErrorCode::INVALID_INPUT);
  };
  {
    GetPathGoal goal;
    goal.goal = makePose(1.0, 0.0);
    goal.plugin_id = "does_not_exist";
    assertRejected(server->requestPath(goal));
  }
  {
    SmoothPathGoal goal;
    goal.path = makePath(2);
    goal.plugin_id = "does_not_exist";
    auto task = server->requestSmoothing(goal);
    ASSERT_TRUE(task.valid());
    ASSERT_TRUE(waitReady(task, 200));
    EXPECT_EQ(task.status(), GoalStatus::ABORTED);
    EXPECT_EQ(task.result().error, ErrorCode::INVALID_INPUT);
  }
  {
    ExePathGoal goal;
    goal.path = makePath(2);
    goal.plugin_id = "does_not_exist";
    auto task = server->requestExecution(goal);
    ASSERT_TRUE(task.valid());
    ASSERT_TRUE(waitReady(task, 200));
    EXPECT_EQ(task.status(), GoalStatus::ABORTED);
    EXPECT_EQ(task.result().error, ErrorCode::INVALID_INPUT);
  }
  {
    RecoveryGoal goal;
    goal.recovery_id = "does_not_exist";
    auto task = server->requestRecovery(goal);
    ASSERT_TRUE(task.valid());
    ASSERT_TRUE(waitReady(task, 200));
    EXPECT_EQ(task.status(), GoalStatus::ABORTED);
    EXPECT_EQ(task.result().error, ErrorCode::INVALID_INPUT);
  }

  // Requests do not re-read the config: after deleting the module json, requests still dispatch successfully per the loaded
  // config (config is read only during configure/load).
  std::filesystem::remove(dir / "planner_a.json");
  std::filesystem::remove(dir / "planner_b.json");
  std::filesystem::remove(dir / "nav.json");
  {
    GetPathGoal goal;
    goal.goal = makePose(1.0, 0.0);
    goal.plugin_id = "planner_a";
    auto task = server->requestPath(goal);
    ASSERT_TRUE(waitReady(task, 200));
    EXPECT_EQ(task.status(), GoalStatus::SUCCEEDED);
  }
}
#endif

// ========== G. Completion/update races + multi-controller cancellation ==========

// An in-place updatePlan on a terminal/finished execution must be rejected (returns false):
// never stages a pending version that would never be consumed; the final result only reflects versions truly applied via setPlan.
TEST(SessionStateDispatchTest, updatePlanRejectedWhenExecutionTerminalKeepsApplied) {
  auto server = std::make_shared<FollowServer>();
  server->installController();
  server->goal_checker->ready.store(true);  // Arrival declared on the first cycle (no new plan consumed before the terminal state)

  const auto action = server->controller_action_ptr;
  ExePathGoal goal;
  goal.path = makePath(2);
  goal.session_id = server->currentSessionId();
  goal.plan_version = 0;
  auto task = action->acceptGoal(goal);
  ASSERT_TRUE(waitReady(task));

  // Submitting the v42 update after the terminal state: rejected, does not occupy the applied version.
  EXPECT_FALSE(action->updatePlan(task.goal_id(), makePath(4), 42u));
  EXPECT_EQ(task.result().plan_version, 0u);  // Reports only the applied version 0, not 42
}

// LoopingController + FlagGoalChecker(ready=false) -> never arrives, stays RUNNING.
#if 0
TEST(SessionStateDispatchTest, multiControllerHaltCancelsAllMapsNotJustDefault) {
  class MultiServer : public NavigationServerBase {
   public:
    void install() {
      default_controller = "controller_a";
      for (const std::string& id : {"controller_a", "controller_b"}) {
        auto controller = std::make_shared<LoopingController>();
        auto goal = std::make_shared<FlagGoalChecker>();
        auto progress = std::make_shared<AlwaysTrueProgressChecker>();
        auto action = std::unique_ptr<InPlaceControllerAction>(
            new InPlaceControllerAction(controller, goal, progress));
        actions[id] = action.get();
        controller_actions[id] = std::move(action);  // Load into the base-class action map
      }
    }
    InPlaceControllerAction* action(const std::string& id) {
      return actions[id];
    }

   private:
    std::map<std::string, InPlaceControllerAction*> actions;
  };

  auto server = std::make_shared<MultiServer>();
  server->install();

  ExePathGoal g1, g2;
  g1.path = makePath(2);
  g2.path = makePath(2);
  g1.plugin_id = "controller_a";
  g2.plugin_id = "controller_b";
  g1.session_id = server->currentSessionId();
  g2.session_id = server->currentSessionId();
  auto t1 = server->requestExecution(g1);
  auto t2 = server->requestExecution(g2);
  ASSERT_EQ(server->action("controller_a")->executions.load(), 1);
  ASSERT_EQ(server->action("controller_b")->executions.load(), 1);

  // Category-level exclusivity: controller_b supersedes controller_a even
  // though they are different plugin instances.
  std::this_thread::sleep_for(std::chrono::milliseconds(40));
  EXPECT_TRUE(t1.ready());
  EXPECT_EQ(t1.status(), GoalStatus::CANCELED);
  EXPECT_FALSE(t2.ready());

  // cancelAll still visits the full action map and stops the remaining owner.
  server->cancelAllControllers();
  ASSERT_TRUE(waitReady(t1));
  ASSERT_TRUE(waitReady(t2));
  // Both tasks finish as CANCELED.
  EXPECT_EQ(t1.result().error, ErrorCode::CANCELED);
  EXPECT_EQ(t2.status(), GoalStatus::CANCELED);
  EXPECT_EQ(t2.result().error, ErrorCode::CANCELED);
}
#endif

/// FollowPathNode update miss: when requestPlanUpdate returns false at registration time (the execution already
/// terminal/released), the node must not report a false FAILURE; it re-queries the finished task and consumes the success result (SUCCESS).
TEST(SessionStateDispatchTest, followPathNodeUpdateMissProcessesCompletedSuccess) {
  auto server = std::make_shared<MockNavServer>();
  server->exe_sync = false;  // Task pending, completed manually by the test
  server->exe_reached = true;
  server->plan_update_ok = false;  // Force an update miss (the execution is "terminal/released")

  auto bb = BT::Blackboard::create();
  setNavServer(bb, server);
  setRequiredEntries(bb);
  bb->set<PoseStamped>(k_goal, makePose(1.0, 2.0));
  bb->set<Path>(k_smoothed_path, makePath(2));

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<FollowPathNode>("FollowPath");
  auto tree = factory.createTreeFromText(k_follow_xml, bb);

  // tick 1: onStart -> submits the v0 execution (async, RUNNING).
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  ASSERT_GE(server->exe_requests, 1);

  // plan_version changed + requestPlanUpdate rejected: an update miss must not immediately FAIL.
  server->incrementPlanVersion();
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);

  // The task completes successfully as "already arrived" (old applied version 0, current version 1) -> re-query -> SUCCESS.
  server->publishExeResult(ErrorCode::NONE, /*reached=*/true, /*plan_version=*/0u);
  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  for (int i = 0; i < 20 && status != BT::NodeStatus::SUCCESS; ++i) {
    status = tree.tickExactlyOnce();
  }
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

}  // namespace

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  if (argc > 1) g_planner_so = argv[1];
  if (argc > 2) g_smoother_so = argv[2];
  if (argc > 3) g_controller_so = argv[3];
  if (argc > 4) g_recovery_so = argv[4];
  return RUN_ALL_TESTS();
}
