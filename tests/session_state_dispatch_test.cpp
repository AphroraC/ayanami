// 焦点测试 (session/version 单一来源 + failure context + plugin id + in-place
// controller 更新 + 按 id 分派 + 配置仅在 configure 期读取)。
//
// 覆盖:
//  A. 共享 NavigationSessionState 单一来源 (pilot 与 server 看到同一 session;
//     tick 线程 applyPendingGoal 开新会话并归零 plan_version)。
//  B. BT action 节点失败前写精确 FailureContext; RecoveryGoal.cause 携带触发
//     上下文。
//  C. pilot 每会话重置 failure_context 黑板。
//  D. GetPathGoal/SmoothPathGoal/ExePathGoal plugin_id: 空时填默认, 显式保留。
//  E. 控制器 in-place plan update 端到端 (FollowPathNode -> requestPlanUpdate ->
//     ControllerAction::updatePlan): 无 cancel / 无新建 Execution / 无中间停止,
//     最终结果携带最新 plan_version。
//  F. NavigationServerBase 按 id 分派多插件配置; 无效 id 返回结构化失败
//     TaskHandle; request 期不重新读配置 (config 仅 configure/load 期读)。
//
// argv: 1=test_planner_plugin.so, 2=test_smoother_plugin.so,
//       3=test_controller_plugin.so, 4=test_recovery_plugin.so

#include <gtest/gtest.h>

#include <ayanami/abstract/components/controller_action.hpp>
#include <ayanami/abstract/components/navigation_server_base.hpp>
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
using ayanami::FailureCode;
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
  bb->set<std::string>(k_planner_id, "test_planner");
  bb->set<std::string>(k_smoother_id, "test_smoother");
  bb->set<std::string>(k_controller_id, "test_controller");
  bb->set<std::string>(k_recovery_id, "test_recovery");
  bb->set<std::string>(k_clear_costmap_id, "clear_costmap");
  bb->set<std::string>(k_spin_id, "spin");
  bb->set<std::string>(k_wait_id, "wait");
  bb->set<std::string>(k_backup_id, "backup");
  bb->set<uint64_t>(k_session_id, 0u);
  bb->set<uint64_t>(k_plan_version, 0u);
  bb->set<uint64_t>(k_reset_stuck_token, 1u);
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
      r.failure = path_failure;
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
      r.failure = smooth_failure;
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
      r.failure = exe_failure;
      r.reached = exe_reached;
      r.session_id = goal.session_id;
      r.plan_version = goal.plan_version;
      exe_server.publishResult(task.goal_id(), GoalStatus::SUCCEEDED, r);
    }
    return task;
  }

  /// @brief 手动发布 ExePath 结果 (async 模式下完成在飞任务, 供竞态测试)。
  void publishExeResult(FailureCode failure, bool reached,
                        uint64_t plan_version) {
    ExePathResult r;
    r.failure = failure;
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
      r.failure = recovery_failure;
      r.session_id = goal.session_id;
      r.plan_version = goal.plan_version;
      recovery_server.publishResult(task.goal_id(), GoalStatus::SUCCEEDED, r);
    }
    return task;
  }

  // 可调: requestPlanUpdate 返回 false 模拟 update miss (执行已终态/释放)。
  bool requestPlanUpdate(uint64_t, const ExePathGoal&) override {
    return plan_update_ok;
  }

  std::atomic<int> cancel_count{0};
  int path_requests{0};
  int exe_requests{0};

  bool path_sync = true;
  FailureCode path_failure = FailureCode::None;
  Path path_result;
  GetPathGoal last_path_goal;

  bool smooth_sync = true;
  FailureCode smooth_failure = FailureCode::None;
  Path smooth_result;
  SmoothPathGoal last_smooth_goal;

  bool exe_sync = true;
  FailureCode exe_failure = FailureCode::None;
  RecoveryGoal last_recovery_goal;
  bool recovery_sync = true;
  FailureCode recovery_failure = FailureCode::None;
  bool exe_reached = true;
  ExePathGoal last_exe_goal;
  uint64_t last_exe_goal_id{0};
  bool plan_update_ok{true};

  PathServer path_server;
  SmoothServer smooth_server;
  ExeServer exe_server;
  RecoveryServer recovery_server;
};

// ---------- Pilot 运行 RAII (focused pilot tests) ----------

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
      "planner_id": "test_planner",
      "smoother_id": "test_smoother",
      "controller_id": "test_controller",
      "planner_frequency": 100.0,
      "smoother_frequency": -1.0,
      "controller_frequency": 20.0,
      "tick_frequency": 100.0
    }
  })");
  return dir;
}

// ---------- 测试服务器组件 ----------

// checker 桩: 立即可达 / 恒有进展。
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

// ---------- Dispatch 测试服务器 (multiple configs 装载 + 记录分派) ----------

class DispatchServer : public NavigationServerBase {
 public:
  bool initializePlannerPlugin(const std::string& name,
                               ayanami::abstract::AbstractPlanner::Ptr p) override {
    init_planners.push_back(name);
    return p != nullptr;
  }
  bool initializeSmootherPlugin(const std::string& name,
                                ayanami::abstract::AbstractSmoother::Ptr p) override {
    init_smoothers.push_back(name);
    return p != nullptr;
  }
  bool initializeControllerPlugin(
      const std::string& name,
      ayanami::abstract::AbstractController::Ptr p) override {
    init_controllers.push_back(name);
    return p != nullptr;
  }
  bool initializeRecoveryPlugin(const std::string& name,
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
    // 注入 checker 保证 requestExecution 能正常走完 (立即可达 + 恒有进展)。
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
      "planner_id": "planner_a",
      "smoother_id": "smoother_a",
      "controller_id": "controller_a",
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

// ---------- In-place controller 更新测试组件 ----------

// 永不到达 + 持续输出速度的控制器 (goal_checker 可触发式到达)。
class LoopingController : public ayanami::abstract::AbstractController {
 public:
  FailureCode computeVelocityCommands(const PoseStamped&, const TwistStamped&,
                                      TwistStamped& cmd_vel, std::string&) override {
    cmd_vel.twist.linear.x() = 0.3;
    return FailureCode::None;
  }
  bool setPlan(const Path&) override { return true; }
  bool cancel() override { return true; }
};

// 可控目标检查: ready 置 true 才判到达。
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

// 带 checker 注入 + 执行计数 的 ControllerAction (保护 createExecution 覆写)。
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

// 带真实 ControllerAction 的 server (供 FollowPathNode in-place 端到端)。
class FollowServer : public NavigationServerBase {
 public:
  std::shared_ptr<FlagGoalChecker> goal_checker;
  InPlaceControllerAction* controller_action = nullptr;

  void installController() {
    default_controller = "test_controller";
    goal_checker = std::make_shared<FlagGoalChecker>();
    auto progress = std::make_shared<AlwaysTrueProgressChecker>();
    auto controller = std::make_shared<LoopingController>();
    auto action = std::unique_ptr<InPlaceControllerAction>(
        new InPlaceControllerAction(controller, goal_checker, progress));
    controller_action = action.get();
    controller_actions["test_controller"] = std::move(action);
  }
};

constexpr const char* k_follow_xml = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="main">
    <FollowPath path="{smoothed_path}" plugin_id="{controller_id}"/>
  </BehaviorTree>
</root>)";

// ========== A. 会话/版本共享单一来源 + 新 goal 归零 ==========

TEST(SessionStateDispatchTest, pilotAndServerShareSingleSessionState) {
  const auto dir = makePilotConfigDir();
  auto server = std::make_shared<MockNavServer>();
  server->path_result = makePath(2);
  server->smooth_result = makePath(2);
  server->exe_sync = false;  // FollowPath 挂起保持 RUNNING

  BehaviorTreePilot pilot;
  pilot.configure(dir.string());
  pilot.loadFromFile("", "");
  pilot.setNavigationRuntime(server);

  // 注入: server 与 pilot 共享同一 NavigationSessionState 对象。
  ASSERT_EQ(server->sessionState().get(), pilot.sessionState().get());

  pilot.beginNavigation(makePose(1.0, 2.0));
  PilotRunner runner(&pilot);

  const auto bb = pilot.blackboard();
  ASSERT_NE(bb, nullptr);

  // tick 线程 applyPendingGoal 开启新会话: 黑板 session == server session == 1。
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

// ========== B. BT 节点失败前写 FailureContext; RecoveryGoal.cause 携带 ==========

TEST(SessionStateDispatchTest, failureContextWrittenAndRecoveryCarriesCause) {
  auto server = std::make_shared<MockNavServer>();
  server->path_sync = true;
  server->path_failure = FailureCode::PathNotFound;

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
  EXPECT_EQ(ctx.code, FailureCode::PathNotFound);
  EXPECT_EQ(ctx.plugin_id, "test_planner");  // 黑板默认族 id 已填入 plugin_id
  EXPECT_EQ(ctx.session_id, server->currentSessionId());
  EXPECT_EQ(ctx.plan_version, server->currentPlanVersion());

  // RecoveryGoal.cause 携带相同上下文 (Recovery 节点读取黑板 failure_context)。
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
            FailureCode::PathNotFound);
  EXPECT_EQ(server->last_recovery_goal.cause.plugin_id, "test_planner");
}

// ========== C. pilot 每会话重置 failure_context ==========

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
  bogus.code = FailureCode::Timeout;
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
  EXPECT_EQ(reset.code, FailureCode::None);
  EXPECT_TRUE(reset.plugin_id.empty());
  EXPECT_TRUE(reset.message.empty());
}

// ========== D. plugin_id: 空填默认 / 显式保留 (server 层) ==========

TEST(SessionStateDispatchTest, pluginIdDefaultedOnlyWhenEmpty) {
  auto dir = makeDispatchConfigDir();
  auto server = std::make_shared<DispatchServer>();
  server->loadPlugins(dir.string());

  {  // 空 plugin_id → 填默认 planner_a 并分派到它
    GetPathGoal goal;
    goal.goal = makePose(1.0, 0.0);
    auto task = server->requestPath(goal);
    ASSERT_TRUE(waitReady(task));
    EXPECT_EQ(task.status(), GoalStatus::SUCCEEDED);
  }
  // 空 id 未被吞掉: 默认分派 = default_planner (navigation.planner_id)。
  EXPECT_EQ(server->init_planners, (std::vector<std::string>{
                                       "planner_a", "planner_b"}));

  {  // 显式 id 保留 (不覆盖成默认), 分派到指定插件
    GetPathGoal goal;
    goal.goal = makePose(1.0, 0.0);
    goal.plugin_id = "planner_b";
    auto task = server->requestPath(goal);
    ASSERT_TRUE(waitReady(task));
    EXPECT_EQ(task.status(), GoalStatus::SUCCEEDED);
    EXPECT_EQ(server->factory_planners.back(), "planner_b");
  }
  {  // 空 id 再次 → 默认分派
    SmoothPathGoal goal;
    goal.path = makePath(2);
    auto task = server->requestSmoothing(goal);
    ASSERT_TRUE(waitReady(task));
    EXPECT_EQ(task.status(), GoalStatus::SUCCEEDED);
  }
  {  // 显式 controller id 保留
    ExePathGoal goal;
    goal.path = makePath(2);
    goal.plugin_id = "controller_b";
    auto task = server->requestExecution(goal);
    ASSERT_TRUE(waitReady(task));
    EXPECT_EQ(task.status(), GoalStatus::SUCCEEDED);
    EXPECT_EQ(server->factory_controllers.back(), "controller_b");
  }
}

// ========== E. 控制器 in-place plan update 端到端 ==========

TEST(SessionStateDispatchTest, controllerInPlacePlanUpdateEndToEnd) {
  auto server = std::make_shared<FollowServer>();
  server->installController();

  // 观测 cmd_vel: 记录零速 (取消/到达发布停止命令)。
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

  // tick 1: onStart → 提交 v0 路径, 真实 ControllerAction 起执行。
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(server->controller_action->executions.load(), 1);
  std::this_thread::sleep_for(std::chrono::milliseconds(30));

  // plan_version 变化: FollowPathNode 走 in-place 更新 (requestPlanUpdate)。
  bb->set<Path>(k_smoothed_path, makePath(4));
  server->incrementPlanVersion();
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);

  // 断言: 无 cancel / 无新建 Execution / 无中间停止。
  EXPECT_EQ(server->controller_action->executions.load(), 1);
  EXPECT_EQ(zero_cmds.load(), 0);  // 更新阶段没有中间零速 (in-place, 无停止)

  // 到达标记打开 → 执行消费 setNewPlan 后到达; 最终结果携带最新 plan_version。
  server->goal_checker->ready.store(true);
  ASSERT_TRUE(waitUntil([&] { return zero_cmds.load() >= 1; }));  // 最终停止

  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  for (int i = 0; i < 50 && status != BT::NodeStatus::SUCCESS; ++i) {
    status = tree.tickExactlyOnce();
  }
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);

  // 无中间停止: 全程只有最终到达那一次零速。
  EXPECT_EQ(zero_cmds.load(), 1);
  // 单一 Execution 贯穿 (未 cancel/未 join/未重建)。
  EXPECT_EQ(server->controller_action->executions.load(), 1);

  ayanami::interfaces::NavigationCallbacks::on_velocity_command.remove(zero_cb);
}

// ========== F. 按 id 分派多配置 + 无效 id 结构化失败 + 无热路径配置读 ==========

TEST(SessionStateDispatchTest, dispatchByPluginIdAndInvalidIsStructuredFailure) {
  auto dir = makeDispatchConfigDir();
  auto server = std::make_shared<DispatchServer>();
  server->loadPlugins(dir.string());

  // multiple loaded configs: 全家族 2 个装载 (recovery 1 个)。
  ASSERT_EQ(server->init_planners.size(), 2u);
  ASSERT_EQ(server->init_smoothers.size(), 2u);
  ASSERT_EQ(server->init_controllers.size(), 2u);
  ASSERT_EQ(server->init_recoveries.size(), 1u);
  ASSERT_NE(server->plannerAction("planner_a"), nullptr);
  ASSERT_NE(server->plannerAction("planner_b"), nullptr);
  ASSERT_NE(server->smootherAction("smoother_b"), nullptr);
  ASSERT_NE(server->controllerAction("controller_b"), nullptr);
  // 无参访问器 = 默认 id 的 action (navigation.*_id)。
  EXPECT_EQ(server->plannerAction(), server->plannerAction("planner_a"));
  EXPECT_EQ(server->controllerAction(), server->controllerAction("controller_a"));
  EXPECT_EQ(server->smootherAction(), server->smootherAction("smoother_a"));

  // 无效 id → 结构化失败 TaskHandle (valid + ABORTED + InvalidInput), 不静默
  // 回退默认。
  auto assertRejected = [&](const TaskHandle<GetPathResult>& task) {
    ASSERT_TRUE(task.valid());
    ASSERT_TRUE(waitReady(task, 200));
    EXPECT_EQ(task.status(), GoalStatus::ABORTED);
    EXPECT_EQ(task.result().failure, FailureCode::InvalidInput);
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
    EXPECT_EQ(task.result().failure, FailureCode::InvalidInput);
  }
  {
    ExePathGoal goal;
    goal.path = makePath(2);
    goal.plugin_id = "does_not_exist";
    auto task = server->requestExecution(goal);
    ASSERT_TRUE(task.valid());
    ASSERT_TRUE(waitReady(task, 200));
    EXPECT_EQ(task.status(), GoalStatus::ABORTED);
    EXPECT_EQ(task.result().failure, FailureCode::InvalidInput);
  }
  {
    RecoveryGoal goal;
    goal.recovery_id = "does_not_exist";
    auto task = server->requestRecovery(goal);
    ASSERT_TRUE(task.valid());
    ASSERT_TRUE(waitReady(task, 200));
    EXPECT_EQ(task.status(), GoalStatus::ABORTED);
    EXPECT_EQ(task.result().failure, FailureCode::InvalidInput);
  }

  // 请求期不重新读配置: 删除模块 json 后请求仍按已装载配置分派成功
  // (config 仅在 configure/load 期读取)。
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

// ========== G. 完成/更新竞态 + 多 controller 取消 (review 修复) ==========

// 终态/已完成的 execution 上 in-place updatePlan 必须被拒绝 (返回 false):
// 不暂存一个永不消费的 pending 版本, 最终结果只反映真正被 setPlan 应用的版本。
TEST(SessionStateDispatchTest, updatePlanRejectedWhenExecutionTerminalKeepsApplied) {
  auto server = std::make_shared<FollowServer>();
  server->installController();
  server->goal_checker->ready.store(true);  // 首周期即判到达 (终态前无新 plan 消费)

  const auto action = server->controller_action;
  ExePathGoal goal;
  goal.path = makePath(2);
  goal.session_id = server->currentSessionId();
  goal.plan_version = 0;
  auto task = action->acceptGoal(goal);
  ASSERT_TRUE(waitReady(task));

  // 终态后再提交 v42 更新: 拒绝, 不占用 applied 版本。
  EXPECT_FALSE(action->updatePlan(task.goal_id(), makePath(4), 42u));
  EXPECT_EQ(task.result().plan_version, 0u);  // 只报已应用版本 0, 不报 42
}

// LoopingController + FlagGoalChecker(ready=false) → 永不到达保持 RUNNING。
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
        controller_actions[id] = std::move(action);  // 装入基类 action map
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
  // 两个任务都以 CANCELED 收尾。
  EXPECT_EQ(t1.result().failure, FailureCode::Canceled);
  EXPECT_EQ(t2.status(), GoalStatus::CANCELED);
  EXPECT_EQ(t2.result().failure, FailureCode::Canceled);
}

// FollowPathNode update miss: registration 期 (execution 已终态/释放) requestPlanUpdate
// 返回 false 时, 节点不得误报 FAILURE; 重查已完成任务并消费成功结果 (SUCCESS)。
TEST(SessionStateDispatchTest, followPathNodeUpdateMissProcessesCompletedSuccess) {
  auto server = std::make_shared<MockNavServer>();
  server->exe_sync = false;  // 任务挂起, 由测试手动完成
  server->exe_reached = true;
  server->plan_update_ok = false;  // 强制 update miss (执行"已终态/释放")

  auto bb = BT::Blackboard::create();
  setNavServer(bb, server);
  setRequiredEntries(bb);
  bb->set<PoseStamped>(k_goal, makePose(1.0, 2.0));
  bb->set<Path>(k_smoothed_path, makePath(2));

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<FollowPathNode>("FollowPath");
  auto tree = factory.createTreeFromText(k_follow_xml, bb);

  // tick 1: onStart → 提交 v0 执行 (async, RUNNING)。
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  ASSERT_GE(server->exe_requests, 1);

  // plan_version 变化 + requestPlanUpdate 拒绝: update miss 不得立即 FAILURE。
  server->incrementPlanVersion();
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);

  // 任务以"已到达"成功完成 (旧 applied 版本 0, 当前版本 1) → 重查 → SUCCESS。
  server->publishExeResult(FailureCode::None, /*reached=*/true, /*plan_version=*/0u);
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
