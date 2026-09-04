#include <gtest/gtest.h>

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

namespace {

using ayanami::geometry_msgs::PoseStamped;
using ayanami::nav_msgs::Path;

// 由 main() 从 argv 读取 (ctest 传各测试插件 $<TARGET_FILE:...>)
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

// 派生 server: 覆写虚钩子记录初始化/工厂调用、配置 planner execution 的
// 终止性 (max_retries=0, 失败即放弃 → ABORTED)。
class TestNavServer : public ayanami::components::NavigationServerBase {
public:
  using NavigationServerBase::NavigationServerBase;

  bool initializePlannerPlugin(
      const std::string& name, const ayanami::Config& /*module_config*/,
      ayanami::abstract::AbstractPlanner::Ptr p) override {
    init_names.push_back("planner:" + name);
    return p != nullptr;
  }
  bool initializeSmootherPlugin(
      const std::string& name, const ayanami::Config& /*module_config*/,
      ayanami::abstract::AbstractSmoother::Ptr p) override {
    init_names.push_back("smoother:" + name);
    return p != nullptr;
  }
  bool initializeControllerPlugin(
      const std::string& name, const ayanami::Config& /*module_config*/,
      ayanami::abstract::AbstractController::Ptr p) override {
    init_names.push_back("controller:" + name);
    return p != nullptr;
  }
  bool initializeRecoveryPlugin(
      const std::string& name, const ayanami::Config& /*module_config*/,
      ayanami::abstract::AbstractRecovery::Ptr p) override {
    init_names.push_back("recovery:" + name);
    return p != nullptr;
  }

  std::unique_ptr<ayanami::components::PlannerExecution> newPlannerExecution(
      const std::string& name,
      ayanami::abstract::AbstractPlanner::Ptr p) override {
    factory_names.push_back(name);
    // 新 execution 无耐心/重试循环: 失败即终态 (失败即放弃语义由核心保证)。
    return NavigationServerBase::newPlannerExecution(name, std::move(p));
  }
  std::unique_ptr<ayanami::components::ControllerExecution> newControllerExecution(
      const std::string& name,
      ayanami::abstract::AbstractController::Ptr p) override {
    factory_names.push_back(name);
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

// 子插件 json (name/so_name, spec §9.5; 工厂统一 create_extension_module)。
std::string pluginJson(const std::string& module, const std::string& name,
                        const std::string& so_name) {
  return "{\n  \"" + module + "\": {\n    \"name\": \"" + name +
         "\",\n    \"so_name\": \"" + so_name + "\"\n  }\n}\n";
}

// 生成临时 config 目录; planner so 可选成功/失败插件 (requestPath 失败契约用)。
std::filesystem::path makeConfigDir(const std::string& planner_so) {
  auto dir = makeTempDir() / ("cfg" + planner_so);
  std::filesystem::remove_all(dir);
  std::filesystem::create_directories(dir);

  writeJson(dir / "config.json",
             "{\n  \"global\": {\n"
             "    \"config_navigation\": \"navigation.json\",\n"
             "    \"config_planner\": \"planner.json\",\n"
             "    \"config_smoother\": \"smoother.json\",\n"
             "    \"config_controller\": \"controller.json\",\n"
             "    \"config_recovery\": [\"recovery.json\"]\n"
             "  }\n}\n");
  writeJson(dir / "navigation.json",
             "{\n  \"navigation\": {\n"
             "    \"planner_id\": \"test_planner\",\n"
             "    \"smoother_id\": \"test_smoother\",\n"
             "    \"controller_id\": \"test_controller\",\n"
             "    \"recovery_id\": \"test_recovery\"\n"
             "  }\n}\n");
  writeJson(dir / "planner.json",
             pluginJson("planner", "test_planner", planner_so));
  writeJson(dir / "smoother.json",
             pluginJson("smoother", "test_smoother", g_smoother_so));
  writeJson(dir / "controller.json",
             pluginJson("controller", "test_controller", g_controller_so));
  writeJson(dir / "recovery.json",
             "{\n  \"recovery\": {\n    \"name\": \"test_recovery\",\n"
             "    \"role\": \"clear_costmap\",\n    \"so_name\": \"" +
             g_recovery_so + "\"\n  }\n}\n");
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

// ---- Recovery 排他性测试桩 ----
// 阻塞 planner/smoother/controller: makePlan/smoothPath/computeVelocityCommands
// 进入后一直阻塞到 cancel() 触发 release_all; exit_* 记录阻塞调用返回时刻。
// 周期 recovery: 首个 onCycleUpdate 记录"三族是否已全部退出"。
namespace recovery_exclusive {

struct Coordinator {
  std::mutex mutex;
  std::condition_variable cv;
  // 每族独立 release 信号: 各自的 cancel() 只释放自己, 保证另两族继续阻塞到
  // 自己的 cancel 到来 (否则早退会让被抢占任务以 SUCCESS 而非 CANCELED 收场)。
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
  int active_recovery_cycles{30};  // 30 * 10ms ≈ 300ms 的 active 窗口
};

class BlockingPlanner : public ayanami::abstract::AbstractPlanner {
 public:
  explicit BlockingPlanner(std::shared_ptr<Coordinator> c) : coord(std::move(c)) {}
  ayanami::PlannerResultT makePlan(
      const ayanami::geometry_msgs::PoseStamped&,
      const ayanami::geometry_msgs::PoseStamped&, double) override {
    coord->planner_entered.store(true);
    {
      std::unique_lock<std::mutex> lock(coord->mutex);
      coord->cv.wait(lock, [this] { return coord->planner_release; });
    }
    coord->planner_exited.store(true);
    ayanami::nav_msgs::Path plan;
    plan.poses.push_back(ayanami::geometry_msgs::PoseStamped{});
    return ayanami::PlannerResultT::success(std::move(plan));
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
  ayanami::SmootherResultT smoothPath(ayanami::nav_msgs::Path& path,
                                      double, bool) override {
    coord->smoother_entered.store(true);
    {
      std::unique_lock<std::mutex> lock(coord->mutex);
      coord->cv.wait(lock, [this] { return coord->smoother_release; });
    }
    coord->smoother_exited.store(true);
    return ayanami::SmootherResultT::success(std::move(path));
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
  ayanami::ControllerResultT computeVelocityCommand(
      const ayanami::geometry_msgs::PoseStamped&,
      const ayanami::geometry_msgs::TwistStamped&) override {
    coord->controller_entered.store(true);
    {
      std::unique_lock<std::mutex> lock(coord->mutex);
      coord->cv.wait(lock, [this] { return coord->controller_release; });
    }
    coord->controller_exited.store(true);
    ayanami::ControllerCommand cmd;
    cmd.velocity = ayanami::geometry_msgs::TwistStamped{};
    cmd.velocity.twist.linear.x() = 0.1;
    return ayanami::ControllerResultT::success(std::move(cmd));
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
  ayanami::RecoveryResultT tick() override {
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
    return ayanami::RecoveryResultT::success(std::move(d));
  }
  bool cancel() override { return true; }
 private:
  std::shared_ptr<Coordinator> coord;
};

// 控制器 execution 需要注入 checker 才进入运行循环 (缺省 abstract execution 无
// checker → INTERNAL_ERROR)。
class NeverGoalChecker : public ayanami::abstract::AbstractGoalChecker {
 public:
  void reset() override {}
  bool isGoalReached(const ayanami::geometry_msgs::PoseStamped&,
                     const ayanami::geometry_msgs::PoseStamped&,
                     const ayanami::geometry_msgs::TwistStamped&) override {
    return false;
  }
  ayanami::abstract::GoalCheckerTolerances getTolerances() const override {
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

// 直接安装 4 个 action (不 loadPlugins): 主用 requestRecovery 的排他逻辑 +
// cancelAll* 三族 action map。
class RecoveryExclusiveServer : public ayanami::components::NavigationServerBase {
 public:
  explicit RecoveryExclusiveServer(std::shared_ptr<Coordinator> coord) {
    default_planner = "blocker";
    default_smoother = "blocker";
    default_controller = "blocker";
    planner_action = std::make_unique<ayanami::components::PlannerAction>(
        std::make_shared<BlockingPlanner>(coord));
    smoother_action = std::make_unique<ayanami::components::SmootherAction>(
        std::make_shared<BlockingSmoother>(coord));
    controller_action = std::make_unique<RecoveryTestControllerAction>(
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

TEST(NavigationServerBaseTest, loadPluginsSucceedsAndCreatesActions) {
  auto dir = makeConfigDir(g_planner_so);
  TestNavServer server;
  server.loadPlugins(dir.string());

  ASSERT_NE(server.plannerAction(), nullptr);
  ASSERT_NE(server.smootherAction(), nullptr);
  ASSERT_NE(server.controllerAction(), nullptr);
  ASSERT_NE(server.recoveryAction("test_recovery"), nullptr);
  ASSERT_EQ(server.recoveryAction("missing"), nullptr);

  // 虚 initialize*Plugin 链被调用 (含 name 透传)
  ASSERT_EQ(server.initNames().size(), 4u);
}

TEST(NavigationServerBaseTest, requestPathSucceedsAndFillsSessionAndVersion) {
  auto dir = makeConfigDir(g_planner_so);
  TestNavServer server;
  server.loadPlugins(dir.string());

  server.bumpSessionId();  // session=1, plan_version 归零 (新会话语义)
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
  // 生效: session/plan_version 与 server 当前值一致
  EXPECT_TRUE(server.sessionIsCurrent(task.result().session_id));
  EXPECT_TRUE(server.planVersionIsCurrent(task.result().plan_version));
  // 虚 new*Execution 工厂被调用 (TestNavServer 覆写被服务器使用的证据)
  ASSERT_FALSE(server.factoryNames().empty());
  EXPECT_EQ(server.factoryNames().front(), "test_planner");
}

TEST(NavigationServerBaseTest, requestExecutionAndRecoveryAndSmoothingSucceed) {
  auto& robot_state = ayanami::components::RobotStateController::instance();
  robot_state.initialize("map", "odom", "base_link", 10.0);
  robot_state.setTransform("map", "base_link", ayanami::Timestamp::latest(),
                           Eigen::Isometry3d::Identity());
  auto dir = makeConfigDir(g_planner_so);
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

TEST(NavigationServerBaseTest, plannerResultCallbackDoesNotScheduleRecovery) {
  auto dir = makeConfigDir(g_fail_planner_so);
  TestNavServer server;
  server.loadPlugins(dir.string());

  std::atomic<int> recovery_goal_hits{0};
  int cb = ayanami::interfaces::NavigationCallbacks::on_recovery_goal.add(
      [&](const ayanami::actions::RecoveryGoal&) { recovery_goal_hits++; });

  // 规划失败结果直接经 on_get_path_result 旁路 emit: server 不再注册任何
  // fallback, 不得 out-of-band 触发 recovery action。
  ayanami::actions::GoalResult<ayanami::actions::GetPathResult> gr;
  gr.status = ayanami::actions::GoalStatus::ABORTED;
  gr.result.error = ayanami::ErrorCode::PATH_NOT_FOUND;
  gr.result.session_id = server.currentSessionId();
  gr.result.plan_version = server.currentPlanVersion();
  ayanami::interfaces::NavigationCallbacks::on_get_path_result(gr);

  // 真实 requestPath 失败 (Action 内部同样 emit on_get_path_result): 仍无
  // out-of-band recovery —— recovery 只应由 BT 内 RecoveryNode 调度。
  ayanami::actions::GetPathGoal goal;
  goal.start = pose("map", 0.0, 0.0);
  goal.goal = pose("map", 1.0, 1.0);
  auto task = server.requestPath(goal);
  ASSERT_TRUE(task.valid());
  ASSERT_TRUE(waitReady(task));
  EXPECT_EQ(task.status(), ayanami::actions::GoalStatus::ABORTED);

  // 留时间窗口: 若存在 out-of-band fallback, recovery_goal 计数应 > 0。
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  EXPECT_EQ(recovery_goal_hits.load(), 0);

  ayanami::interfaces::NavigationCallbacks::on_recovery_goal.remove(cb);
}

TEST(NavigationServerBaseTest, destructorStopsActionsAndDoesNotCrash) {
  auto dir = makeConfigDir(g_planner_so);
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

TEST(NavigationServerBaseTest, requestRecoveryIsExclusiveWithoutServerPreemption) {
  // requestRecovery owns the controller/recovery resource boundary. It stops
  // controller execution before starting the recovery action.
  auto coord = std::make_shared<recovery_exclusive::Coordinator>();
  auto& robot_state = ayanami::components::RobotStateController::instance();
  robot_state.initialize("map", "odom", "base_link", 10.0);
  robot_state.setTransform("map", "base_link", ayanami::Timestamp::latest(),
                           Eigen::Isometry3d::Identity());
  recovery_exclusive::RecoveryExclusiveServer server(coord);

  // 1. 启动三个阻塞任务 (进入插件后一直阻塞)。
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

  // 3. recovery active 期间: 普通 request 被结构化拒绝 (ABORTED +
  // ErrorCode::RESOURCE_BUSY)。
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

  // 二次 recovery 同样被拒绝。
  auto rr = server.requestRecovery(rgoal);
  ASSERT_TRUE(rr.valid());
  ASSERT_TRUE(rr.ready());
  EXPECT_EQ(rr.status(), ayanami::actions::GoalStatus::ABORTED);
  EXPECT_EQ(rr.result().error, ayanami::ErrorCode::RESOURCE_BUSY);

  // 4. recovery 完成 -> 恢复接受普通请求。
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

  // 5. 清理: 运行时资源协议里 recovery 启动前由 requestRecovery 内部调
  // cancelAllControllers (cancel+join)。这里直接调用验证 join 语义;
  // planner/smoother 同样调用以保证线程退出。
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

  // 取消+join 完成后 controller 可再次接受新任务 (恢复后重新 FollowPath)。
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
