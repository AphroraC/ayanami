#include <gtest/gtest.h>

#include <ayanami/abstract/components/navigation_server_base.hpp>
#include <ayanami/abstract/core/abstract_goal_checker.hpp>
#include <ayanami/abstract/core/abstract_progress_checker.hpp>
#include <ayanami/abstract/core/map_context.hpp>
#include <ayanami/interfaces/callbacks.hpp>

#include <Eigen/Geometry>

#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace {

using ayanami::geometry_msgs::PoseStamped;
using ayanami::nav_msgs::Path;

// 由 main() 从 argv 读取 (ctest 传 $<TARGET_FILE:navigation_plugins>)
std::string g_plugins_so;

std::filesystem::path make_temp_dir() {
  std::filesystem::path dir = std::filesystem::temp_directory_path() /
                              "ayanami_navigation_server_test";
  std::filesystem::remove_all(dir);
  std::filesystem::create_directories(dir);
  return dir;
}

void write_json(const std::filesystem::path& path,
                const std::string& content) {
  std::ofstream ofs(path);
  ofs << content;
}

// 通用 MapContext 桩 (Task 7 AbstractMapContext 派生, 测试注入链)。
class TestMapContext : public ayanami::abstract::AbstractMapContext {
public:
  const std::string& getGlobalFrame() const override {
    static const std::string frame = "map";
    return frame;
  }
  const std::string& getRobotFrame() const override {
    static const std::string frame = "base_link";
    return frame;
  }
  bool getRobotPose(PoseStamped& pose) const override {
    pose = PoseStamped::fromIsometry("map", Eigen::Isometry3d::Identity());
    return true;
  }
};

// checker 测试桩: 立即可达 + 恒有进展 (requestExecution 契约: 控制器成功即
// goal_checker 判到达 → SUCCEEDED)。
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

class TrueProgressChecker : public ayanami::abstract::AbstractProgressChecker {
 public:
  void reset() override {}
  bool check(const PoseStamped&) override { return true; }
};

// 派生 server: 覆写虚钩子以注入 TestMapContext、记录初始化/工厂调用、配置
// planner execution 的终止性 (max_retries=0, 失败即放弃 → ABORTED)。
class TestNavServer : public ayanami::components::NavigationServerBase {
public:
  using NavigationServerBase::NavigationServerBase;

  bool initializePlannerPlugin(const std::string& name,
                               ayanami::abstract::AbstractPlanner::Ptr p) override {
    init_names_.push_back("planner:" + name);
    map_context_ = std::make_shared<TestMapContext>();
    return p != nullptr;
  }
  bool initializeSmootherPlugin(const std::string& name,
                                ayanami::abstract::AbstractSmoother::Ptr p) override {
    init_names_.push_back("smoother:" + name);
    return p != nullptr;
  }
  bool initializeControllerPlugin(const std::string& name,
                                  ayanami::abstract::AbstractController::Ptr p) override {
    init_names_.push_back("controller:" + name);
    return p != nullptr;
  }
  bool initializeRecoveryPlugin(const std::string& name,
                                ayanami::abstract::AbstractRecovery::Ptr p) override {
    init_names_.push_back("recovery:" + name);
    return p != nullptr;
  }

  std::unique_ptr<ayanami::components::PlannerExecution> newPlannerExecution(
      const std::string& name,
      ayanami::abstract::AbstractPlanner::Ptr p) override {
    factory_names_.push_back(name);
    auto exec =
        NavigationServerBase::newPlannerExecution(name, std::move(p));
    exec->setMaxRetries(0);  // 失败即放弃 → 保证失败路径终止性
    return exec;
  }
  std::unique_ptr<ayanami::components::ControllerExecution> newControllerExecution(
      const std::string& name,
      ayanami::abstract::AbstractController::Ptr p) override {
    factory_names_.push_back(name);
    auto exec =
        NavigationServerBase::newControllerExecution(name, std::move(p));
    // 注入 checker (生产路径由 createActions 按配置注入; 本测试配置无 checker ids)。
    exec->setGoalChecker(std::make_shared<ImmediateGoalChecker>());
    exec->setProgressChecker(std::make_shared<TrueProgressChecker>());
    return exec;
  }
  std::unique_ptr<ayanami::components::RecoveryExecution> newRecoveryExecution(
      const std::string& name,
      ayanami::abstract::AbstractRecovery::Ptr p) override {
    factory_names_.push_back(name);
    return NavigationServerBase::newRecoveryExecution(name, std::move(p));
  }

  void bumpSessionId() { session_id_.fetch_add(1); }
  const std::vector<std::string>& initNames() const { return init_names_; }
  const std::vector<std::string>& factoryNames() const {
    return factory_names_;
  }
  bool mapInjected() const { return map_context_ != nullptr; }

private:
  std::vector<std::string> init_names_;
  std::vector<std::string> factory_names_;
};

// 子插件 json (name/so_name/create_symbol, spec §9.5)。
std::string plugin_json(const std::string& module, const std::string& name,
                        const std::string& symbol) {
  return "{\n  \"" + module + "\": {\n    \"name\": \"" + name +
         "\",\n    \"so_name\": \"" + g_plugins_so +
         "\",\n    \"create_symbol\": \"" + symbol + "\"\n  }\n}\n";
}

// 生成临时 config 目录; planner 符号可选成功/失败工厂 (验证旁路 fallback 用)。
std::filesystem::path make_config_dir(const std::string& planner_symbol) {
  auto dir = make_temp_dir() / ("cfg_" + planner_symbol);
  std::filesystem::remove_all(dir);
  std::filesystem::create_directories(dir);

  write_json(dir / "config.json",
             "{\n  \"global\": {\n"
             "    \"config_navigation\": \"navigation.json\",\n"
             "    \"config_planner\": \"planner.json\",\n"
             "    \"config_smoother\": \"smoother.json\",\n"
             "    \"config_controller\": \"controller.json\",\n"
             "    \"config_recovery\": [\"recovery.json\"]\n"
             "  }\n}\n");
  write_json(dir / "navigation.json",
             "{\n  \"navigation\": {\n"
             "    \"planner_id\": \"test_planner\",\n"
             "    \"smoother_id\": \"test_smoother\",\n"
             "    \"controller_id\": \"test_controller\",\n"
             "    \"recovery_id\": \"test_recovery\"\n"
             "  }\n}\n");
  write_json(dir / "planner.json",
             plugin_json("planner", "test_planner", planner_symbol));
  write_json(dir / "smoother.json",
             plugin_json("smoother", "test_smoother",
                         "create_test_smoother_plugin"));
  write_json(dir / "controller.json",
             plugin_json("controller", "test_controller",
                         "create_test_controller_plugin"));
  write_json(dir / "recovery.json",
             "{\n  \"recovery\": {\n    \"name\": \"test_recovery\",\n"
             "    \"role\": \"clear_costmap\",\n    \"so_name\": \"" +
             g_plugins_so +
             "\",\n    \"create_symbol\": \"create_test_recovery_plugin\"\n  }\n}\n");
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
  p.pose.translation() = Eigen::Vector3d(x, y, 0.0);
  return p;
}

}  // namespace

TEST(NavigationServerBaseTest, LoadPluginsSucceedsAndCreatesActions) {
  auto dir = make_config_dir("create_test_planner_plugin");
  TestNavServer server;
  server.loadPlugins(dir.string());

  ASSERT_NE(server.plannerAction(), nullptr);
  ASSERT_NE(server.smootherAction(), nullptr);
  ASSERT_NE(server.controllerAction(), nullptr);
  ASSERT_NE(server.recoveryAction("test_recovery"), nullptr);
  ASSERT_EQ(server.recoveryAction("missing"), nullptr);

  // 虚 initialize*Plugin 链被调用 (含 name 透传) + TestMapContext 注入生效
  ASSERT_EQ(server.initNames().size(), 4u);
  EXPECT_TRUE(server.mapInjected());
}

TEST(NavigationServerBaseTest, RequestPathSucceedsAndFillsSessionAndVersion) {
  auto dir = make_config_dir("create_test_planner_plugin");
  TestNavServer server;
  server.loadPlugins(dir.string());

  server.incrementPlanVersion();
  server.bumpSessionId();

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
  // 生效: session/plan_version 与 server 当前值一致
  EXPECT_TRUE(server.sessionIsCurrent(task.result().session_id));
  EXPECT_TRUE(server.planVersionIsCurrent(task.result().plan_version));
  // 虚 new*Execution 工厂被调用 (TestNavServer 覆写被服务器使用的证据)
  ASSERT_FALSE(server.factoryNames().empty());
  EXPECT_EQ(server.factoryNames().front(), "test_planner");
}

TEST(NavigationServerBaseTest, RequestExecutionAndRecoveryAndSmoothingSucceed) {
  auto dir = make_config_dir("create_test_planner_plugin");
  TestNavServer server;
  server.loadPlugins(dir.string());

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

TEST(NavigationServerBaseTest, PlannerFallbackTriggersDefaultRecovery) {
  auto dir = make_config_dir("create_fail_planner_plugin");
  TestNavServer server;
  server.loadPlugins(dir.string());

  std::atomic<int> recovery_goal_hits{0};
  int cb = ayanami::interfaces::NavigationCallbacks::on_recovery_goal.add(
      [&](const ayanami::actions::RecoveryGoal&) { recovery_goal_hits++; });

  server.registerPlannerFallback();

  ayanami::actions::GetPathGoal goal;
  goal.start = pose("map", 0.0, 0.0);
  goal.goal = pose("map", 1.0, 1.0);
  auto task = server.requestPath(goal);
  ASSERT_TRUE(task.valid());
  ASSERT_TRUE(waitReady(task));
  EXPECT_EQ(task.status(), ayanami::actions::GoalStatus::ABORTED);

  // 旁路链: on_get_path_result(失败) → fallback → default recovery acceptGoal
  for (int i = 0; i < 100 && recovery_goal_hits.load() == 0; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_GE(recovery_goal_hits.load(), 1);

  ayanami::interfaces::NavigationCallbacks::on_recovery_goal.remove(cb);
}

TEST(NavigationServerBaseTest, RegisterPlannerFallbackOnlyOnce) {
  auto dir = make_config_dir("create_fail_planner_plugin");
  TestNavServer server;
  server.loadPlugins(dir.string());

  std::atomic<int> recovery_goal_hits{0};
  int cb = ayanami::interfaces::NavigationCallbacks::on_recovery_goal.add(
      [&](const ayanami::actions::RecoveryGoal&) { recovery_goal_hits++; });

  server.registerPlannerFallback();
  server.registerPlannerFallback();  // 重复注册应被忽略

  ayanami::actions::GetPathGoal goal;
  goal.start = pose("map", 0.0, 0.0);
  goal.goal = pose("map", 1.0, 1.0);
  auto task = server.requestPath(goal);
  ASSERT_TRUE(waitReady(task));

  for (int i = 0; i < 100 && recovery_goal_hits.load() == 0; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_EQ(recovery_goal_hits.load(), 1);

  ayanami::interfaces::NavigationCallbacks::on_recovery_goal.remove(cb);
}

TEST(NavigationServerBaseTest, DestructorStopsActionsAndDoesNotCrash) {
  auto dir = make_config_dir("create_test_planner_plugin");
  {
    TestNavServer server;
    server.loadPlugins(dir.string());

    ayanami::actions::GetPathGoal goal;
    goal.start = pose("map", 0.0, 0.0);
    goal.goal = pose("map", 1.0, 1.0);
    auto task = server.requestPath(goal);
    ASSERT_TRUE(waitReady(task));
    EXPECT_EQ(task.status(), ayanami::actions::GoalStatus::SUCCEEDED);

    // 显式 stop: 取消所有 RUNNING action (此时无 RUNNING, 幂等)
    server.start();
    server.stop();
  }
  // 作用域结束 → 析构: stop → reset actions → clearPlugins, 不挂
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  if (argc > 1) {
    g_plugins_so = argv[1];
  }
  return RUN_ALL_TESTS();
}
