// Checker unit tests: AbstractGoalChecker / AbstractProgressChecker abstractions +
// SimpleGoalChecker / SimpleProgressChecker modules + ModuleManager<Checker family>
// dlopen loading + NavigationServerBase injection (createActions -> ControllerExecution).
//
// argv: 1=simple_goal_checker.so, 2=simple_progress_checker.so,
// 3=test_planner_module.so, 4=test_smoother_module.so,
// 5=test_looping_controller_module.so, 6=test_recovery_module.so

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <thread>

#include <ayanami/abstract/components/navigation_server_base.hpp>
#include <ayanami/abstract/components/module_manager.hpp>
#include <ayanami/abstract/core/abstract_goal_checker.hpp>
#include <ayanami/abstract/core/abstract_progress_checker.hpp>
#include <simple_goal_checker/simple_goal_checker.hpp>
#include <simple_progress_checker/simple_progress_checker.hpp>
#include <ayanami/messages/actions.hpp>

namespace {

using ayanami::abstract::AbstractGoalChecker;
using ayanami::abstract::AbstractProgressChecker;
using ayanami::components::NavigationServerBase;
using ayanami::components::ModuleManager;
using ayanami::geometry_msgs::PoseStamped;
using ayanami::geometry_msgs::TwistStamped;
using ayanami::modules::SimpleGoalChecker;
using ayanami::modules::SimpleProgressChecker;

std::string g_goal_checker_so;
std::string g_progress_checker_so;
std::string g_planner_so;
std::string g_smoother_so;
std::string g_looping_controller_so;
std::string g_recovery_so;

std::filesystem::path makeTempDir() {
  std::filesystem::path dir =
      std::filesystem::temp_directory_path() / "ayanami_checker_test";
  std::filesystem::remove_all(dir);
  std::filesystem::create_directories(dir);
  return dir;
}

void write_file(const std::filesystem::path& path,
                const std::string& content) {
  std::ofstream ofs(path);
  ofs << content;
}

PoseStamped pose(double x, double y, double yaw = 0.0) {
  PoseStamped p = PoseStamped::fromIsometry("map", Eigen::Isometry3d::Identity());
  p.pose.position = Eigen::Vector3d(x, y, 0.0);
  p.pose.orientation =
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return p;
}

TwistStamped twist(double linear_x, double angular_z) {
  TwistStamped t;
  t.twist.linear.x() = linear_x;
  t.twist.angular.z() = angular_z;
  return t;
}

// --- Direct module API tests (config-driven tolerance / time window) ---

std::filesystem::path makeModuleConfigDir() {
  const auto dir = makeTempDir() / "module_cfg";
  std::filesystem::remove_all(dir);
  std::filesystem::create_directories(dir);
  // The parameter section has the same name as the module (injected via ModuleContext at configure time).
  write_file(dir / "simple_goal_checker.json",
             "{\n  \"simple_goal_checker\": {\n"
             "    \"position_tolerance\": 0.1,\n"
             "    \"angle_tolerance\": 0.2,\n"
             "    \"linear_tolerance\": 0.1,\n"
             "    \"angular_tolerance\": 0.1\n"
             "  }\n}\n");
  write_file(dir / "simple_progress_checker.json",
             "{\n  \"simple_progress_checker\": {\n"
             "    \"required_movement\": 0.5,\n"
             "    \"movement_angle_required\": 0.2,\n"
             "    \"time_period\": 0.01\n"
             "  }\n}\n");
  return dir;
}

/// Direct module tests inject uniformly: call configure(ModuleContext) explicitly after construction.
ayanami::ModuleContext checkerContext(const std::filesystem::path& dir,
                                      const std::string& json_name,
                                      const std::string& name) {
  return ayanami::ModuleContext{name, ayanami::Config((dir / json_name).string())};
}

TEST(SimpleGoalCheckerTest, reachedWithinTolerances) {
  const auto dir = makeModuleConfigDir();
  SimpleGoalChecker checker;
  checker.configure(checkerContext(dir, "simple_goal_checker.json",
                                   "simple_goal_checker"));
  checker.reset();

  EXPECT_TRUE(checker.isGoalReached(pose(0.0, 0.0), pose(0.05, 0.0),
                                    twist(0.0, 0.0)));
  EXPECT_TRUE(checker.isGoalReached(pose(0.0, 0.0), pose(0.0, 0.0),
                                    twist(0.05, 0.02)));
}

TEST(SimpleGoalCheckerTest, notReachedOutsideTolerances) {
  const auto dir = makeModuleConfigDir();
  SimpleGoalChecker checker;
  checker.configure(checkerContext(dir, "simple_goal_checker.json",
                                   "simple_goal_checker"));
  checker.reset();

  EXPECT_FALSE(checker.isGoalReached(pose(0.0, 0.0), pose(1.0, 0.0),
                                     twist(0.0, 0.0)));
  EXPECT_FALSE(checker.isGoalReached(pose(0.0, 0.0), pose(0.0, 0.0),
                                     twist(0.5, 0.0)));
  EXPECT_FALSE(checker.isGoalReached(pose(0.0, 0.0), pose(0.0, 0.0),
                                     twist(0.0, 0.5)));
  // Heading error too large (90 deg > 0.2 rad) does not count as reached.
  EXPECT_FALSE(checker.isGoalReached(
      pose(0.0, 0.0, 1.5708), pose(0.0, 0.0), twist(0.0, 0.0)));
}

TEST(SimpleGoalCheckerTest, getTolerancesLoadedFromConfig) {
  const auto dir = makeModuleConfigDir();
  SimpleGoalChecker checker;
  checker.configure(checkerContext(dir, "simple_goal_checker.json",
                                   "simple_goal_checker"));
  const auto tol = checker.getTolerances();
  EXPECT_DOUBLE_EQ(tol.pose.position, 0.1);
  EXPECT_DOUBLE_EQ(tol.pose.angle, 0.2);
  EXPECT_DOUBLE_EQ(tol.velocity.linear, 0.1);
  EXPECT_DOUBLE_EQ(tol.velocity.angular, 0.1);
}

TEST(SimpleProgressCheckerTest, movesWithinPeriodHasProgress) {
  const auto dir = makeModuleConfigDir();
  SimpleProgressChecker checker;
  checker.configure(checkerContext(dir, "simple_progress_checker.json",
                                   "simple_progress_checker"));
  checker.reset();
  EXPECT_TRUE(checker.check(pose(0.0, 0.0)));
  // Movement within the time window exceeds required_movement -> progress.
  EXPECT_TRUE(checker.check(pose(1.0, 0.0)));
}

TEST(SimpleProgressCheckerTest, noMovementWithinPeriodIsNoProgress) {
  const auto dir = makeModuleConfigDir();
  SimpleProgressChecker checker;
  checker.configure(checkerContext(dir, "simple_progress_checker.json",
                                   "simple_progress_checker"));
  checker.reset();
  EXPECT_TRUE(checker.check(pose(0.0, 0.0)));
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  // Time window (10ms) elapsed without movement -> no progress.
  EXPECT_FALSE(checker.check(pose(0.0, 0.0)));
}

TEST(SimpleProgressCheckerTest, angleMovementCountsAsProgress) {
  const auto dir = makeModuleConfigDir();
  SimpleProgressChecker checker;
  checker.configure(checkerContext(dir, "simple_progress_checker.json",
                                   "simple_progress_checker"));
  checker.reset();
  EXPECT_TRUE(checker.check(pose(0.0, 0.0)));
  // Rotation > movement_angle_required -> progress.
  EXPECT_TRUE(checker.check(pose(0.0, 0.0, 1.0)));
}

// --- ModuleManager<Checker family> dlopen loading ---

TEST(CheckerModuleManagerTest, loadGoalCheckerFromSo) {
  ModuleManager<AbstractGoalChecker> mgr;
  ayanami::components::ModuleSpec spec;
  spec.name = "simple_goal_checker";
  spec.so_name = g_goal_checker_so;
  // Loading-focused test: parameters are covered by the checker's own tests,
  // so an empty config is enough here (the module keeps its defaults).
  ASSERT_TRUE(mgr.loadModule(
      spec, ayanami::ModuleContext{spec.name, ayanami::Config("")},
      [](const std::string&, std::shared_ptr<AbstractGoalChecker> module) {
        return module != nullptr;
      }));
  ASSERT_TRUE(mgr.hasModule("simple_goal_checker"));
  auto checker = mgr.getModule("simple_goal_checker");
  ASSERT_NE(checker, nullptr);
  EXPECT_TRUE(checker->isGoalReached(pose(0.0, 0.0), pose(0.01, 0.0),
                                     twist(0.0, 0.0)));
}

TEST(CheckerModuleManagerTest, loadProgressCheckerFromSo) {
  ModuleManager<AbstractProgressChecker> mgr;
  ayanami::components::ModuleSpec spec;
  spec.name = "simple_progress_checker";
  spec.so_name = g_progress_checker_so;
  ASSERT_TRUE(mgr.loadModule(
      spec, ayanami::ModuleContext{spec.name, ayanami::Config("")},
      [](const std::string&, std::shared_ptr<AbstractProgressChecker> module) {
        return module != nullptr;
      }));
  ASSERT_TRUE(mgr.hasModule("simple_progress_checker"));
}

// ---- Parameter loading happens after a successful load, not at construction ----
// The factory stays argument-free; the loader hands the module its context once
// the object exists and was accepted. This asserts the sequencing is real: the
// module is loaded through ModuleManager only (no configure() call), and its
// behaviour already reflects the configured values rather than the field
// defaults on SimpleGoalCheckerParam.
TEST(CheckerModuleManagerTest, loaderInjectsParametersWithoutConfigureCall) {
  const auto dir = makeTempDir() / "loader_params";
  std::filesystem::remove_all(dir);
  std::filesystem::create_directories(dir);
  // Deliberately far from the defaults (0.10 m / 0.20 rad): accepting a 4 m
  // offset is only possible if the config was actually read.
  write_file(dir / "goal_checker.json",
             "{\n  \"goal_checker\": {\n"
             "    \"name\": \"loader_probe_checker\",\n"
             "    \"so_name\": \"" + g_goal_checker_so + "\"\n"
             "  },\n"
             "  \"loader_probe_checker\": {\n"
             "    \"position_tolerance\": 5.0,\n"
             "    \"angle_tolerance\": 3.0,\n"
             "    \"linear_tolerance\": 0.1,\n"
             "    \"angular_tolerance\": 0.1\n"
             "  }\n}\n");

  ModuleManager<AbstractGoalChecker> mgr;
  ASSERT_TRUE(mgr.loadSingleFromConfig(
      ayanami::Config((dir / "goal_checker.json").string()), "goal_checker",
      "loader_probe_checker",
      [](const std::string&, std::shared_ptr<AbstractGoalChecker> module) {
        return module != nullptr;
      }));

  auto checker = mgr.getModule("loader_probe_checker");
  ASSERT_NE(checker, nullptr);
  const auto tolerances = checker->getTolerances();
  EXPECT_DOUBLE_EQ(tolerances.pose.position, 5.0)
      << "the loader must read the module's parameters before use";
  EXPECT_DOUBLE_EQ(tolerances.pose.angle, 3.0);
  // Behaviour follows: 4 m away is inside the configured 5 m tolerance.
  EXPECT_TRUE(checker->isGoalReached(pose(0.0, 0.0), pose(4.0, 0.0),
                                     twist(0.0, 0.0)));
}

// --- NavigationServerBase: createActions injection -> ControllerExecution ----

/// Exposes the protected velocity-limit members (the new execution has no getMaximumVelocity output getter;
/// semantically equivalent: assert the navigation.max_vel_{x,y,theta} injected by the server).
class ExposedControllerExecution
    : public ayanami::components::ControllerExecution {
 public:
  using ControllerExecution::ControllerExecution;
  void maximumVelocity(double& x, double& y, double& theta) const {
    x = max_linear_x;
    y = max_linear_y;
    theta = max_angular_z;
  }
};

/// Derived server: captures the execution from each newControllerExecution (non-owning) so tests
/// can assert checker injection. The checker is loaded by the base loadModules (default override accepted).
class CheckerNavServer : public NavigationServerBase {
 public:
  // Weak capture: the Action slot owns the execution lifetime (safe to access while the task is RUNNING).
  ExposedControllerExecution* running_controller_exec = nullptr;

  std::unique_ptr<ayanami::components::ControllerExecution>
  newControllerExecution(
      const std::string& name,
      ayanami::abstract::AbstractController::Ptr controller) override {
    (void)name;
    auto exec =
        std::make_unique<ExposedControllerExecution>(std::move(controller));
    running_controller_exec = exec.get();
    return exec;
  }



  bool goalCheckerLoaded(const std::string& name) const {
    return goal_checker_modules.hasModule(name);
  }
  bool progressCheckerLoaded(const std::string& name) const {
    return progress_checker_modules.hasModule(name);
  }
};

std::filesystem::path makeServerConfigDir() {
  const auto dir = makeTempDir() / ("srv" + std::to_string(std::rand()));
  std::filesystem::create_directories(dir);
  write_file(dir / "config.json",
             "{\n  \"global\": {\n"
             "    \"config_navigation\": \"navigation.json\"\n"
             "  }\n}\n");
  // Module file pointers live uniformly in the navigation section (single source), no separate *_id keys.
  write_file(dir / "navigation.json",
             "{\n  \"navigation\": {\n"
             "    \"config_planner\": \"planner.json\",\n"
             "    \"config_smoother\": \"smoother.json\",\n"
             "    \"config_controller\": \"controller.json\",\n"
             "    \"config_recovery\": [\"recovery.json\"],\n"
             "    \"config_goal_checker\": \"goal_checker.json\",\n"
             "    \"config_progress_checker\": \"progress_checker.json\",\n"
             "    \"max_vel_x\": 0.5,\n"
             "    \"max_vel_y\": 0.0,\n"
             "    \"max_vel_theta\": 1.5,\n"
             "    \"controller_patience\": 1.5,\n"
             "    \"recovery_patience\": 2.5\n"
             "  }\n}\n");

  auto moduleJson = [](const std::string& module, const std::string& name,
                        const std::string& so) {
    // Includes the module's own (empty) parameter section: the loader requires
    // a section named after the module, so "reads no parameters" must be
    // declared rather than look like a name/section mismatch.
    return "{\n  \"" + module + "\": {\n    \"name\": \"" + name +
           "\",\n    \"so_name\": \"" + so + "\"\n  },\n  \"" + name +
           "\": {}\n}\n";
  };

  write_file(dir / "planner.json",
             moduleJson("planner", "test_planner", g_planner_so));
  write_file(dir / "smoother.json",
             moduleJson("smoother", "test_smoother", g_smoother_so));
  write_file(dir / "controller.json",
             moduleJson("controller", "test_controller",
                         g_looping_controller_so));
  write_file(dir / "recovery.json",
             "{\n  \"recovery\": {\n    \"name\": \"test_recovery\",\n"
             "    \"role\": \"clear_costmap\",\n    \"so_name\": \"" +
             g_recovery_so +
             "\"\n  },\n  \"test_recovery\": {}\n}\n");
  write_file(dir / "goal_checker.json",
             moduleJson("goal_checker", "simple_goal_checker",
                         g_goal_checker_so));
  write_file(dir / "progress_checker.json",
             moduleJson("progress_checker", "simple_progress_checker",
                         g_progress_checker_so));
  return dir;
}

template <typename Task>
bool waitReady(Task& t, int max_ms = 3000) {
  for (int i = 0; i < max_ms / 10 && !t.ready(); ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return t.ready();
}

TEST(CheckerServerInjectionTest, controllerExecutionGetsCheckers) {
  CheckerNavServer server;
  auto dir = makeServerConfigDir();
  server.loadModules(dir.string());

  ASSERT_NE(server.controllerAction(), nullptr);
  // The checker modules are loaded and the default ids resolved.
  EXPECT_TRUE(server.goalCheckerLoaded("simple_goal_checker"));
  EXPECT_TRUE(server.progressCheckerLoaded("simple_progress_checker"));

  // The looping controller never arrives -> the execution stays RUNNING; read the injected checker safely meanwhile.
  ayanami::actions::ExePathGoal goal;
  goal.path.poses.push_back(pose(0.0, 0.0));
  goal.path.poses.push_back(pose(1.0, 0.0));
  auto task = server.requestExecution(goal);
  ASSERT_TRUE(task.valid());

  for (int i = 0; i < 200 && !task.ready() &&
                   server.running_controller_exec == nullptr;
       ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  ASSERT_NE(server.running_controller_exec, nullptr);
  EXPECT_NE(server.running_controller_exec->getGoalChecker(), nullptr);
  EXPECT_NE(server.running_controller_exec->getProgressChecker(), nullptr);

  // createActions injects navigation.max_vel_{x,y,theta} -> the execution velocity limits.
  double max_x = 0.0, max_y = 0.0, max_theta = 0.0;
  server.running_controller_exec->maximumVelocity(max_x, max_y, max_theta);
  EXPECT_DOUBLE_EQ(max_x, 0.5);
  EXPECT_DOUBLE_EQ(max_y, 0.0);  // max_vel_y=0: lateral motion forbidden
  EXPECT_DOUBLE_EQ(max_theta, 1.5);
  // createActions injects navigation.controller_patience -> the execution.
  EXPECT_DOUBLE_EQ(server.running_controller_exec->getPatience(), 1.5);
  // navigation.recovery_patience cached on the server (injected into every
  // recovery execution at factory time).
  EXPECT_DOUBLE_EQ(server.recoveryPatience(), 2.5);

  task.cancel();
  ASSERT_TRUE(waitReady(task));
}

}  // namespace

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  if (argc > 1) g_goal_checker_so = argv[1];
  if (argc > 2) g_progress_checker_so = argv[2];
  if (argc > 3) g_planner_so = argv[3];
  if (argc > 4) g_smoother_so = argv[4];
  if (argc > 5) g_looping_controller_so = argv[5];
  if (argc > 6) g_recovery_so = argv[6];
  return RUN_ALL_TESTS();
}
