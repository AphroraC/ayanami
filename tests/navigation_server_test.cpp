#include <gtest/gtest.h>

#include "test_transform_source.hpp"

#include <ayanami/abstract/components/controller_action.hpp>
#include <ayanami/abstract/components/navigation_server_base.hpp>
#include <ayanami/abstract/components/robot_state_controller.hpp>
#include <ayanami/abstract/components/planner_action.hpp>
#include <ayanami/abstract/components/recovery_action.hpp>
#include <ayanami/abstract/components/smoother_action.hpp>
#include <ayanami/abstract/core/abstract_controller.hpp>
#include <ayanami/abstract/core/abstract_goal_checker.hpp>
#include <ayanami/abstract/core/abstract_planner.hpp>
#include <ayanami/abstract/core/abstract_progress_checker.hpp>
#include <ayanami/abstract/core/abstract_recovery.hpp>
#include <ayanami/interfaces/callbacks.hpp>
#include <ayanami/messages/failure.hpp>
#include <ayanami/types.hpp>
#include <ayanami/utility/config.hpp>

#include <Eigen/Geometry>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

using ayanami::testing::testTransforms;
namespace {

using ayanami::geometry_msgs::PoseStamped;
using ayanami::nav_msgs::Path;

// Read by main from argv (ctest passes the $<TARGET_FILE:...> of each test module)
std::string g_planner_so;
std::string g_fail_planner_so;
std::string g_smoother_so;
std::string g_controller_so;
std::string g_recovery_so;

std::filesystem::path makeTempDir() {
  std::filesystem::path dir = std::filesystem::temp_directory_path() /
                              "ayanami_navigation_server_test";
  std::filesystem::remove_all(dir);
  std::filesystem::create_directories(dir);
  return dir;
}

void writeJson(const std::filesystem::path& path,
                const std::string& content) {
  std::ofstream ofs(path);
  ofs << content;
}

/// Checker test stubs: immediate arrival + always progressing (requestExecution contract: controller success plus
/// goal_checker arrival -> SUCCEEDED).
class ImmediateGoalChecker : public ayanami::abstract::AbstractGoalChecker {
 public:
  void reset() override {}
  bool isGoalReached(const PoseStamped&, const PoseStamped&,
                     const ayanami::geometry_msgs::TwistStamped&) override {
    return true;
  }
  ayanami::GoalCheckerTolerances getTolerances() const override {
    return {};
  }
};

class TrueProgressChecker : public ayanami::abstract::AbstractProgressChecker {
 public:
  void reset() override {}
  bool check(const PoseStamped&) override { return true; }
};

/// Derived server: overrides the virtual hooks to record initialization/factory calls and configures the planner
/// execution for termination (max_retries=0, give up on failure -> ABORTED).
class TestNavServer : public ayanami::components::NavigationServerBase {
public:
  using NavigationServerBase::NavigationServerBase;

  bool initializePlannerModule(
      const std::string& name, const ayanami::Config& /*module_config*/,
      ayanami::abstract::AbstractPlanner::Ptr p) override {
    init_names.push_back("planner:" + name);
    return p != nullptr;
  }
  bool initializeSmootherModule(
      const std::string& name, const ayanami::Config& /*module_config*/,
      ayanami::abstract::AbstractSmoother::Ptr p) override {
    init_names.push_back("smoother:" + name);
    return p != nullptr;
  }
  bool initializeControllerModule(
      const std::string& name, const ayanami::Config& /*module_config*/,
      ayanami::abstract::AbstractController::Ptr p) override {
    init_names.push_back("controller:" + name);
    return p != nullptr;
  }
  bool initializeRecoveryModule(
      const std::string& name, const ayanami::Config& /*module_config*/,
      ayanami::abstract::AbstractRecovery::Ptr p) override {
    init_names.push_back("recovery:" + name);
    return p != nullptr;
  }

  std::unique_ptr<ayanami::components::PlannerExecution> newPlannerExecution(
      const std::string& name,
      ayanami::abstract::AbstractPlanner::Ptr p) override {
    factory_names.push_back(name);
    // The new execution has no patience/retry loop: a failure is terminal (the give-up-on-failure semantics are guaranteed by the core).
    return NavigationServerBase::newPlannerExecution(name, std::move(p));
  }
  std::unique_ptr<ayanami::components::ControllerExecution> newControllerExecution(
      const std::string& name,
      ayanami::abstract::AbstractController::Ptr p) override {
    factory_names.push_back(name);
    auto exec =
        NavigationServerBase::newControllerExecution(name, std::move(p));
    // Injects the checker (the production path injects via createActions per config; this test config has no checker ids).
    exec->setGoalChecker(std::make_shared<ImmediateGoalChecker>());
    exec->setProgressChecker(std::make_shared<TrueProgressChecker>());
    return exec;
  }
  std::unique_ptr<ayanami::components::RecoveryExecution> newRecoveryExecution(
      const std::string& name,
      ayanami::abstract::AbstractRecovery::Ptr p) override {
    factory_names.push_back(name);
    return NavigationServerBase::newRecoveryExecution(name, std::move(p));
  }

  void bumpSessionId() { startNewSession(); }
  const std::vector<std::string>& initNames() const { return init_names; }
  const std::vector<std::string>& factoryNames() const {
    return factory_names;
  }

private:
  std::vector<std::string> init_names;
  std::vector<std::string> factory_names;
};

// Child module json (name/so_name; the factory is uniformly create_extension_module).
std::string moduleJson(const std::string& module, const std::string& name,
                        const std::string& so_name) {
  // The module's own (empty) parameter section: the loader requires a section
  // named after the module, so that "this module reads no parameters" is
  // declared rather than indistinguishable from a name/section mismatch.
  return "{\n  \"" + module + "\": {\n    \"name\": \"" + name +
         "\",\n    \"so_name\": \"" + so_name + "\"\n  },\n  \"" + name +
         "\": {}\n}\n";
}

/// Generates a temp config dir; the planner so can be a success or failure module (for the requestPath failure contract).
std::filesystem::path makeConfigDir(const std::string& planner_so) {
  auto dir = makeTempDir() / ("cfg" + planner_so);
  std::filesystem::remove_all(dir);
  std::filesystem::create_directories(dir);

  writeJson(dir / "config.json",
             "{\n  \"global\": {\n"
             "    \"config_navigation\": \"navigation.json\"\n"
             "  }\n}\n");
  // Module file pointers live uniformly in the navigation section; module id = the name in each module json.
  writeJson(dir / "navigation.json",
             "{\n  \"navigation\": {\n"
             "    \"config_planner\": \"planner.json\",\n"
             "    \"config_smoother\": \"smoother.json\",\n"
             "    \"config_controller\": \"controller.json\",\n"
             "    \"config_recovery\": [\"recovery.json\"]\n"
             "  }\n}\n");
  writeJson(dir / "planner.json",
             moduleJson("planner", "test_planner", planner_so));
  writeJson(dir / "smoother.json",
             moduleJson("smoother", "test_smoother", g_smoother_so));
  writeJson(dir / "controller.json",
             moduleJson("controller", "test_controller", g_controller_so));
  writeJson(dir / "recovery.json",
             "{\n  \"recovery\": {\n    \"name\": \"test_recovery\",\n"
             "    \"role\": \"clear_costmap\",\n    \"so_name\": \"" +
             g_recovery_so + "\"\n  },\n  \"test_recovery\": {}\n}\n");
  return dir;
}

template <typename Task>
bool waitReady(Task& t, int max_ms = 3000) {
  for (int i = 0; i < max_ms / 10 && !t.ready(); ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return t.ready();
}

PoseStamped pose(const std::string& frame, double x, double y) {
  auto p = PoseStamped::fromIsometry(frame, Eigen::Isometry3d::Identity());
  p.pose.position = Eigen::Vector3d(x, y, 0.0);
  return p;
}

// ---- Recovery exclusivity test stubs ----
// Blocking planner/smoother/controller: makePlan/smoothPath/computeVelocityCommands
// block after entry until cancel triggers release_all; exit_* record when the blocked calls returned.
// Periodic recovery: the first onCycleUpdate records whether all three families have exited.
namespace recovery_exclusive {

struct Coordinator {
  std::mutex mutex;
  std::condition_variable cv;
  // Per-family independent release signals: each cancel releases only its own family, keeping the other two blocked
  // until their own cancel arrives (otherwise an early exit would let a preempted task finish SUCCESS instead of CANCELED).
  bool planner_release{false};
  bool smoother_release{false};
  bool controller_release{false};
  std::atomic<bool> planner_entered{false};
  std::atomic<bool> smoother_entered{false};
  std::atomic<bool> controller_entered{false};
  std::atomic<bool> planner_exited{false};
  std::atomic<bool> smoother_exited{false};
  std::atomic<bool> controller_exited{false};
  std::atomic<bool> recovery_started{false};
  std::atomic<bool> all_exited_when_recovery_started{false};
  std::atomic<int> recovery_cycles{0};
  int active_recovery_cycles{30};  // 30 * 10ms ~ 300ms active window
};

class BlockingPlanner : public ayanami::abstract::AbstractPlanner {
 public:
  explicit BlockingPlanner(std::shared_ptr<Coordinator> c) : coord(std::move(c)) {}
  ayanami::ResultT<void> makePlan(
      const ayanami::geometry_msgs::PoseStamped&,
      const ayanami::geometry_msgs::PoseStamped&, double, ayanami::nav_msgs::Path& plan_out) override {
    coord->planner_entered.store(true);
    {
      std::unique_lock<std::mutex> lock(coord->mutex);
      coord->cv.wait(lock, [this] { return coord->planner_release; });
    }
    coord->planner_exited.store(true);
    ayanami::nav_msgs::Path plan;
    plan.poses.push_back(ayanami::geometry_msgs::PoseStamped{});
    plan_out = std::move(plan);
    return ayanami::ResultT<void>::success();
  }
  bool cancel() override {
    {
      std::lock_guard<std::mutex> lock(coord->mutex);
      coord->planner_release = true;
    }
    coord->cv.notify_all();
    return true;
  }
 private:
  std::shared_ptr<Coordinator> coord;
};

class BlockingSmoother : public ayanami::abstract::AbstractSmoother {
 public:
  explicit BlockingSmoother(std::shared_ptr<Coordinator> c) : coord(std::move(c)) {}
  ayanami::ResultT<void> smoothPath(ayanami::nav_msgs::Path& path,
                                    double)  override {
    coord->smoother_entered.store(true);
    {
      std::unique_lock<std::mutex> lock(coord->mutex);
      coord->cv.wait(lock, [this] { return coord->smoother_release; });
    }
    coord->smoother_exited.store(true);
    return ayanami::ResultT<void>::success();
  }
  bool cancel() override {
    {
      std::lock_guard<std::mutex> lock(coord->mutex);
      coord->smoother_release = true;
    }
    coord->cv.notify_all();
    return true;
  }
 private:
  std::shared_ptr<Coordinator> coord;
};

class BlockingController : public ayanami::abstract::AbstractController {
 public:
  explicit BlockingController(std::shared_ptr<Coordinator> c) : coord(std::move(c)) {}
  ayanami::ResultT<void> computeVelocityCommand(
      const ayanami::geometry_msgs::PoseStamped&,
      const ayanami::geometry_msgs::TwistStamped&, ayanami::geometry_msgs::TwistStamped& cmd) override {
    coord->controller_entered.store(true);
    {
      std::unique_lock<std::mutex> lock(coord->mutex);
      coord->cv.wait(lock, [this] { return coord->controller_release; });
    }
    coord->controller_exited.store(true);
    cmd = ayanami::geometry_msgs::TwistStamped{};
    cmd.twist.linear.x() = 0.1;
    return ayanami::ResultT<void>::success();
  }
  bool setPlan(const ayanami::nav_msgs::Path&) override { return true; }
  bool cancel() override {
    {
      std::lock_guard<std::mutex> lock(coord->mutex);
      coord->controller_release = true;
    }
    coord->cv.notify_all();
    return true;
  }
 private:
  std::shared_ptr<Coordinator> coord;
};

class ActiveRecovery : public ayanami::abstract::AbstractRecovery {
 public:
  explicit ActiveRecovery(std::shared_ptr<Coordinator> c) : coord(std::move(c)) {}
  ayanami::ResultT<void> tick(ayanami::RecoveryCycleData& cycle) override {
    if (!coord->recovery_started.exchange(true)) {
      coord->all_exited_when_recovery_started.store(
          coord->planner_exited.load() && coord->smoother_exited.load() &&
          coord->controller_exited.load());
    }
    ayanami::RecoveryCycleData d;
    d.status =
        coord->recovery_cycles.fetch_add(1) < coord->active_recovery_cycles
            ? ayanami::RecoveryStatus::RUNNING
            : ayanami::RecoveryStatus::SUCCESS;
    cycle = std::move(d);
    return ayanami::ResultT<void>::success();
  }
  bool cancel() override { return true; }
 private:
  std::shared_ptr<Coordinator> coord;
};

/// A controller execution needs an injected checker to enter the run loop (the default abstract execution has no
/// checker -> INTERNAL_ERROR).
class NeverGoalChecker : public ayanami::abstract::AbstractGoalChecker {
 public:
  void reset() override {}
  bool isGoalReached(const ayanami::geometry_msgs::PoseStamped&,
                     const ayanami::geometry_msgs::PoseStamped&,
                     const ayanami::geometry_msgs::TwistStamped&) override {
    return false;
  }
  ayanami::GoalCheckerTolerances getTolerances() const override {
    return {};
  }
};

class RecoveryTestControllerAction : public ayanami::components::ControllerAction {
 public:
  using ControllerAction::ControllerAction;
 protected:
  ayanami::components::ControllerExecution::Ptr createExecution() override {
    auto exec =
        std::make_shared<ayanami::components::ControllerExecution>(controller);
    exec->setGoalChecker(std::make_shared<NeverGoalChecker>());
    exec->setProgressChecker(std::make_shared<TrueProgressChecker>());
    return exec;
  }
};

/// Installs the 4 actions directly (no loadModules): mainly to exercise requestRecovery exclusivity +
/// the cancelAll* action maps of the three families.
class RecoveryExclusiveServer : public ayanami::components::NavigationServerBase {
 public:
  explicit RecoveryExclusiveServer(std::shared_ptr<Coordinator> coord) {
    default_planner = "blocker";
    default_smoother = "blocker";
    default_controller = "blocker";
    planner_actions["blocker"] = std::make_unique<ayanami::components::PlannerAction>(
        std::make_shared<BlockingPlanner>(coord));
    smoother_actions["blocker"] = std::make_unique<ayanami::components::SmootherAction>(
        std::make_shared<BlockingSmoother>(coord));
    controller_actions["blocker"] = std::make_unique<RecoveryTestControllerAction>(
        std::make_shared<BlockingController>(coord));
    recovery_actions["test_recovery"] = std::make_unique<ayanami::components::RecoveryAction>(
        std::make_shared<ActiveRecovery>(coord));
  }
};

inline bool waitFlag(const std::atomic<bool>& flag, int max_ms = 3000) {
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(max_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    if (flag.load()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  return flag.load();
}

}  // namespace recovery_exclusive

}  // namespace

TEST(NavigationServerBaseTest, loadModulesSucceedsAndCreatesActions) {
  auto dir = makeConfigDir(g_planner_so);
  TestNavServer server;
  server.loadModules(dir.string());

  ASSERT_NE(server.plannerAction(), nullptr);
  ASSERT_NE(server.smootherAction(), nullptr);
  ASSERT_NE(server.controllerAction(), nullptr);
  ASSERT_NE(server.recoveryAction("test_recovery"), nullptr);
  ASSERT_EQ(server.recoveryAction("missing"), nullptr);

  // The virtual initialize*Module chain was invoked (including name pass-through)
  ASSERT_EQ(server.initNames().size(), 4u);
}

TEST(NavigationServerBaseTest, requestPathSucceedsAndFillsSessionAndVersion) {
  auto dir = makeConfigDir(g_planner_so);
  TestNavServer server;
  server.loadModules(dir.string());

  server.bumpSessionId();  // session=1, plan_version zeroed (new-session semantics)
  server.incrementPlanVersion();  // plan_version=1

  ayanami::actions::GetPathGoal goal;
  goal.start = pose("map", 0.0, 0.0);
  goal.goal = pose("map", 1.0, 1.0);
  auto task = server.requestPath(goal);
  ASSERT_TRUE(task.valid());
  ASSERT_TRUE(waitReady(task));
  EXPECT_EQ(task.status(), ayanami::actions::GoalStatus::SUCCEEDED);
  EXPECT_FALSE(task.result().path.poses.empty());
  EXPECT_EQ(task.result().session_id, 1u);
  EXPECT_EQ(task.result().plan_version, 1u);
  // Took effect: session/plan_version match the server current values
  EXPECT_TRUE(server.sessionIsCurrent(task.result().session_id));
  EXPECT_TRUE(server.planVersionIsCurrent(task.result().plan_version));
  // The virtual new*Execution factory was invoked (evidence the TestNavServer override is used by the server)
  ASSERT_FALSE(server.factoryNames().empty());
  EXPECT_EQ(server.factoryNames().front(), "test_planner");
}

TEST(NavigationServerBaseTest, requestExecutionAndRecoveryAndSmoothingSucceed) {
  auto& robot_state = ayanami::components::RobotStateController::instance();
  robot_state.initialize("map", "odom", "base_link", 10.0);
  testTransforms().set("map", "base_link", ayanami::Timestamp::latest(),
                           Eigen::Isometry3d::Identity());
  auto dir = makeConfigDir(g_planner_so);
  TestNavServer server;
  server.loadModules(dir.string());

  {
    ayanami::actions::ExePathGoal goal;
    goal.path.poses.push_back(pose("map", 0.0, 0.0));
    auto task = server.requestExecution(goal);
    ASSERT_TRUE(task.valid());
    ASSERT_TRUE(waitReady(task));
    EXPECT_EQ(task.status(), ayanami::actions::GoalStatus::SUCCEEDED);
  }
  {
    ayanami::actions::RecoveryGoal goal;
    goal.recovery_id = "test_recovery";
    auto task = server.requestRecovery(goal);
    ASSERT_TRUE(task.valid());
    ASSERT_TRUE(waitReady(task));
    EXPECT_EQ(task.status(), ayanami::actions::GoalStatus::SUCCEEDED);
  }
  {
    ayanami::actions::SmoothPathGoal goal;
    goal.path.poses.push_back(pose("map", 0.0, 0.0));
    auto task = server.requestSmoothing(goal);
    ASSERT_TRUE(task.valid());
    ASSERT_TRUE(waitReady(task));
    EXPECT_EQ(task.status(), ayanami::actions::GoalStatus::SUCCEEDED);
    EXPECT_FALSE(task.result().smoothed_path.poses.empty());
  }
}

TEST(NavigationServerBaseTest, plannerResultCallbackDoesNotScheduleRecovery) {
  auto dir = makeConfigDir(g_fail_planner_so);
  TestNavServer server;
  server.loadModules(dir.string());

  std::atomic<int> recovery_goal_hits{0};
  int cb = ayanami::interfaces::NavigationCallbacks::on_recovery_goal.add(
      [&](const ayanami::actions::RecoveryGoal&) { recovery_goal_hits++; });

  // A planning failure result is emitted directly via the on_get_path_result bypass: the server no longer registers any
  // fallback and must not trigger a recovery action out-of-band.
  ayanami::actions::GoalResult<ayanami::actions::GetPathResult> gr;
  gr.status = ayanami::actions::GoalStatus::ABORTED;
  gr.result.error = ayanami::ErrorCode::PATH_NOT_FOUND;
  gr.result.session_id = server.currentSessionId();
  gr.result.plan_version = server.currentPlanVersion();
  ayanami::interfaces::NavigationCallbacks::on_get_path_result(gr);

  // A real requestPath failure (the Action also emits on_get_path_result internally): still no
  // out-of-band recovery -- recovery must only be scheduled by the RecoveryNode inside the BT.
  ayanami::actions::GetPathGoal goal;
  goal.start = pose("map", 0.0, 0.0);
  goal.goal = pose("map", 1.0, 1.0);
  auto task = server.requestPath(goal);
  ASSERT_TRUE(task.valid());
  ASSERT_TRUE(waitReady(task));
  EXPECT_EQ(task.status(), ayanami::actions::GoalStatus::ABORTED);

  // Leave a time window: if an out-of-band fallback existed, the recovery_goal count would be > 0.
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  EXPECT_EQ(recovery_goal_hits.load(), 0);

  ayanami::interfaces::NavigationCallbacks::on_recovery_goal.remove(cb);
}

TEST(NavigationServerBaseTest, destructorStopsActionsAndDoesNotCrash) {
  auto dir = makeConfigDir(g_planner_so);
  {
    TestNavServer server;
    server.loadModules(dir.string());

    ayanami::actions::GetPathGoal goal;
    goal.start = pose("map", 0.0, 0.0);
    goal.goal = pose("map", 1.0, 1.0);
    auto task = server.requestPath(goal);
    ASSERT_TRUE(waitReady(task));
    EXPECT_EQ(task.status(), ayanami::actions::GoalStatus::SUCCEEDED);

    // Explicit stop: cancels all RUNNING actions (none RUNNING here; idempotent)
    server.start();
    server.stop();
  }
  // End of scope -> destruction: stop -> reset actions -> clearModules, no hang
}

TEST(NavigationServerBaseTest, requestRecoveryIsExclusiveWithoutServerPreemption) {
  // requestRecovery owns the controller/recovery resource boundary. It stops
  // controller execution before starting the recovery action.
  auto coord = std::make_shared<recovery_exclusive::Coordinator>();
  auto& robot_state = ayanami::components::RobotStateController::instance();
  robot_state.initialize("map", "odom", "base_link", 10.0);
  testTransforms().set("map", "base_link", ayanami::Timestamp::latest(),
                           Eigen::Isometry3d::Identity());
  recovery_exclusive::RecoveryExclusiveServer server(coord);

  // 1. Start three blocking tasks (they block after entering the module).
  ayanami::actions::GetPathGoal pgoal;
  pgoal.start = pose("map", 0.0, 0.0);
  pgoal.goal = pose("map", 1.0, 1.0);
  auto ptask = server.requestPath(pgoal);

  ayanami::actions::SmoothPathGoal sgoal;
  sgoal.path.poses.push_back(pose("map", 0.0, 0.0));
  auto stask = server.requestSmoothing(sgoal);

  ayanami::actions::ExePathGoal egoal;
  egoal.path.poses.push_back(pose("map", 0.0, 0.0));
  auto etask = server.requestExecution(egoal);

  ASSERT_TRUE(ptask.valid());
  ASSERT_TRUE(stask.valid());
  ASSERT_TRUE(etask.valid());

  ASSERT_TRUE(recovery_exclusive::waitFlag(coord->planner_entered));
  ASSERT_TRUE(recovery_exclusive::waitFlag(coord->smoother_entered));
  ASSERT_TRUE(recovery_exclusive::waitFlag(coord->controller_entered));

  // 2. requestRecovery: synchronously stops controller before recovery starts.
  ayanami::actions::RecoveryGoal rgoal;
  rgoal.recovery_id = "test_recovery";
  auto rtask = server.requestRecovery(rgoal);
  ASSERT_TRUE(rtask.valid());

  ASSERT_TRUE(recovery_exclusive::waitFlag(coord->recovery_started, 5000));
  EXPECT_TRUE(coord->controller_exited.load());

  // 3. While recovery is active: ordinary requests are structurally rejected (ABORTED +
  // ErrorCode::RESOURCE_BUSY).
  auto rp = server.requestPath(pgoal);
  ASSERT_TRUE(rp.valid());
  ASSERT_TRUE(rp.ready());
  EXPECT_EQ(rp.status(), ayanami::actions::GoalStatus::ABORTED);
  EXPECT_EQ(rp.result().error, ayanami::ErrorCode::RESOURCE_BUSY);

  auto re = server.requestExecution(egoal);
  ASSERT_TRUE(re.valid());
  ASSERT_TRUE(re.ready());
  EXPECT_EQ(re.status(), ayanami::actions::GoalStatus::ABORTED);
  EXPECT_EQ(re.result().error, ayanami::ErrorCode::RESOURCE_BUSY);

  // A second recovery is rejected as well.
  auto rr = server.requestRecovery(rgoal);
  ASSERT_TRUE(rr.valid());
  ASSERT_TRUE(rr.ready());
  EXPECT_EQ(rr.status(), ayanami::actions::GoalStatus::ABORTED);
  EXPECT_EQ(rr.result().error, ayanami::ErrorCode::RESOURCE_BUSY);

  // 4. Recovery completes -> ordinary requests are accepted again.
  std::atomic<bool> recovery_done{false};
  rtask.onComplete([&recovery_done] { recovery_done.store(true); });
  ASSERT_TRUE(waitReady(rtask, 5000));
  EXPECT_EQ(rtask.status(), ayanami::actions::GoalStatus::SUCCEEDED);
  {
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(2000);
    while (!recovery_done.load() &&
           std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    ASSERT_TRUE(recovery_done.load());
  }

  // 5. Cleanup: in the runtime resource protocol requestRecovery internally calls cancelAllControllers before
  // starting a recovery (cancel+join). Calling it directly here verifies the join semantics;
  // planner/smoother are called the same way to guarantee thread exit.
  server.cancelAllControllers();
  server.cancelAllPlanners();
  server.cancelAllSmoothers();
  ASSERT_TRUE(waitReady(ptask, 2000));
  EXPECT_EQ(ptask.status(), ayanami::actions::GoalStatus::CANCELED);
  ASSERT_TRUE(waitReady(stask, 2000));
  EXPECT_EQ(stask.status(), ayanami::actions::GoalStatus::CANCELED);
  ASSERT_TRUE(waitReady(etask, 2000));
  EXPECT_EQ(etask.status(), ayanami::actions::GoalStatus::CANCELED);
  EXPECT_TRUE(coord->planner_exited.load());
  EXPECT_TRUE(coord->smoother_exited.load());
  EXPECT_TRUE(coord->controller_exited.load());

  // After cancel+join the controller can accept new tasks again (FollowPath resumes after recovery).
  auto exe_after = server.requestExecution(egoal);
  ASSERT_TRUE(exe_after.valid());
  EXPECT_FALSE(exe_after.ready());
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  if (argc > 1) g_planner_so = argv[1];
  if (argc > 2) g_fail_planner_so = argv[2];
  if (argc > 3) g_smoother_so = argv[3];
  if (argc > 4) g_controller_so = argv[4];
  if (argc > 5) g_recovery_so = argv[5];
  return RUN_ALL_TESTS();
}
