// modules/costmap_navigation (AbstractNavigation 宿主插件)。
//
// M1 关键里程碑: 把核心库全部组件 (CostmapNavigationServer + 4 族插件 +
// BehaviorTreePilot) 组装为 dlopen 入口插件。本测试:
// - ExtensionModule 加载 libayanami_costmap_navigation.so
// - initialize: configureCoreMaps + loadPlugins (dlopen smac/savitzky/spin
// 真实算法模块 + test controller) + createActions + pilot wiring (
// configure → loadFromFile → setNavigationRuntime)
// - start 首发可视化 (on_costmap_visualization 旁路被捕获)
// - setGoal → navigation_thread 跑 BT → raw_path/smoothed_path 入黑板 +
// on_velocity_command 收到非零速度 (首帧 cmd_vel.linear 已初始化非 NaN)
// - 抢占: 第二次 setGoal → 先发零速度 + session_id 自增 + BT 树不重建
// - 析构顺序不崩

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Geometry>

#include <ayanami/behavior_tree/blackboard_protocol.hpp>
#include <ayanami/interfaces/callbacks.hpp>
#include <ayanami/messages/geometry_msgs.hpp>
#include <ayanami/messages/nav_msgs.hpp>
#include <costmap_navigation/costmap_navigation.hpp>
#include <ayanami/abstract/core/abstract_navigation.hpp>
#include <ayanami/costmap/core/costmap_planner.hpp>
#include <ayanami/costmap/core/costmap_smoother.hpp>
#include <ayanami/costmap/core/costmap_controller.hpp>
#include <ayanami/costmap/core/costmap_recovery.hpp>
#include <ayanami/utility/extension_module.hpp>

namespace {

using ayanami::behavior_tree::k_raw_path;
using ayanami::behavior_tree::k_goal_revision;
using ayanami::behavior_tree::k_session_id;
using ayanami::behavior_tree::k_smoothed_path;
using ayanami::geometry_msgs::PoseStamped;
using ayanami::geometry_msgs::TwistStamped;
using ayanami::modules::CostmapNavigation;
using ayanami::nav_msgs::Path;

// argv: module .so + 7 插件 .so (smac/savitzky/pid/test_controller/spin/
// goal_checker/progress_checker)。
std::string g_module_so;
std::string g_smac_so;
std::string g_savitzky_so;
std::string g_pid_so;
std::string g_test_controller_so;
std::string g_spin_so;
std::string g_goal_checker_so;
std::string g_progress_checker_so;

// 捕获 on_velocity_command 事件。
std::vector<TwistStamped> g_cmds;
std::mutex g_cmd_mutex;
// 抢占标记: setGoal 预抢占后观察者记录零速度计数。
std::atomic<bool> g_preempt_armed{false};
std::atomic<int> g_zero_after_preempt{0};
// 可视化捕获。
std::vector<std::string> g_viz_names;
std::mutex g_viz_mutex;

// ---------------- 测试工具 ----------------

bool isZeroTwist(const TwistStamped& t) {
  return t.twist.linear.norm() < 1e-9 && t.twist.angular.norm() < 1e-9;
}

bool allFinite(const TwistStamped& t) {
  return std::isfinite(t.twist.linear.x()) && std::isfinite(t.twist.linear.y()) &&
         std::isfinite(t.twist.linear.z()) && std::isfinite(t.twist.angular.x()) &&
         std::isfinite(t.twist.angular.y()) && std::isfinite(t.twist.angular.z());
}

PoseStamped makePose(double x, double y = 0.0) {
  auto p = PoseStamped::fromIsometry("map", Eigen::Isometry3d::Identity());
  p.pose.position = Eigen::Vector3d(x, y, 0.0);
  return p;
}

std::filesystem::path makeTempDir() {
  std::filesystem::path dir =
      std::filesystem::temp_directory_path() / "ayanami_costmap_navigation_test";
  std::filesystem::remove_all(dir);
  std::filesystem::create_directories(dir);
  return dir;
}

void write_file(const std::filesystem::path& path,
                const std::string& content) {
  std::ofstream ofs(path);
  ofs << content;
}

std::string pluginJson(const std::string& module, const std::string& name,
                        const std::string& so_name) {
  return "{\n  \"" + module + "\": {\n    \"name\": \"" + name +
         "\",\n    \"so_name\": \"" + so_name + "\"\n  }\n}\n";
}

// 未闭合版 (供与模块参数段拼接成单文件)。
std::string pluginJsonOpen(const std::string& module, const std::string& name,
                             const std::string& so_name) {
  return "{\n  \"" + module + "\": {\n    \"name\": \"" + name +
         "\",\n    \"so_name\": \"" + so_name + "\"\n  },\n";
}

// 生成临时 config 目录。so_name 用 argv 注入的真实模块绝对路径。
std::filesystem::path makeConfigDir() {
  const auto dir = makeTempDir() / "cfg";
  std::filesystem::remove_all(dir);
  std::filesystem::create_directories(dir);
  std::filesystem::create_directories(dir / "BT");

  write_file(dir / "config.json",
             "{\n  \"global\": {\n"
              "    \"config_navigation\": \"costmap_navigation.json\"\n"
             "  }\n}\n");

  write_file(dir / "planner.json",
             pluginJsonOpen("planner", "test_planner", g_smac_so) +
             "  \"costmap_smac_planner\": {\n"
             "    \"use_diagonal\": true,\n    \"tolerance\": 0.0,\n"
             "    \"cost_factor\": 0.5,\n    \"lethal_cost\": 254,\n"
             "    \"neutral_cost\": 1.0,\n    \"use_final_approach\": false\n"
             "  }\n}\n");
  write_file(dir / "smoother.json",
             pluginJsonOpen("smoother", "test_smoother", g_savitzky_so) +
             "  \"costmap_savitzky_smoother\": {\n"
             "    \"do_refinement\": true,\n    \"refinement_num\": 2\n"
             "  }\n}\n");
  write_file(dir / "controller.json",
             pluginJson("controller", "test_controller", g_test_controller_so));
  write_file(dir / "recovery.json",
             "{\n  \"recovery\": {\n    \"name\": \"test_spin\",\n"
             "    \"role\": \"spin\",\n    \"so_name\": \"" + g_spin_so +
             "\",\n"
             "    \"spin_dist\": 1.57,\n    \"max_w\": 0.5,\n"
             "    \"sim_cycle_dt\": 0.01,\n    \"collision_radius\": 0.3\n"
             "  }\n}\n");
  // goal/progress checker 插件 (execution 需要 injected checkers, 缺失判 InternalError)。
  write_file(dir / "goal_checker.json",
             pluginJsonOpen("goal_checker", "simple_goal_checker",
                         g_goal_checker_so) +
             "  \"goal_checker\": {\n"
             "    \"position_tolerance\": 0.10,\n"
             "    \"angle_tolerance\": 0.20,\n"
             "    \"linear_tolerance\": 0.10,\n"
             "    \"angular_tolerance\": 0.10\n"
             "  }\n}\n");
  write_file(dir / "progress_checker.json",
             pluginJsonOpen("progress_checker", "simple_progress_checker",
                         g_progress_checker_so) +
             "  \"progress_checker\": {\n"
             "    \"required_movement\": 0.1,\n"
             "    \"movement_angle_required\": 0.2,\n"
             "    \"time_period\": 5.0\n"
             "  }\n}\n");

  // 20x20 @ 0.5 = 10m x 10m, origin (0,0); 全 FREE 静态地图 (A* 从 (0,0)
  // 到 (2,0) 可寻路)。
  const std::string pgm_path = (dir / "map.pgm").string();
  {
    std::ofstream out(pgm_path, std::ios::binary);
    out << "P5\n20 20\n255\n";
    for (int i = 0; i < 400; ++i) {
      out.put(static_cast<char>(255));
    }
  }

  // costmap 参数并入 costmap_navigation.json (原 costmap.json 移除)。
  {
    std::ofstream nav(dir / "costmap_navigation.json");
    nav << "{\n  \"navigation\": {\n"
        << "    \"behavior_tree_extension\": \"" AYANAMI_TEST_BT_EXT_SO "\",\n"
        << "    \"behavior_tree_xml\": \"BT/navigation.xml\",\n"
        << "    \"behavior_tree_id\": \"NavigateWithReplanningAndSmoothing\",\n"
        << "    \"planner_id\": \"test_planner\",\n"
        << "    \"smoother_id\": \"test_smoother\",\n"
        << "    \"controller_id\": \"test_controller\",\n"
        << "    \"config_planner\": \"planner.json\",\n"
        << "    \"config_smoother\": \"smoother.json\",\n"
        << "    \"config_controller\": \"controller.json\",\n"
        << "    \"config_goal_checker\": \"goal_checker.json\",\n"
        << "    \"config_progress_checker\": \"progress_checker.json\",\n"
        << "    \"config_recovery\": [\"recovery.json\"],\n"
        << "    \"clear_costmap_id\": \"test_spin\",\n"
        << "    \"spin_id\": \"test_spin\",\n"
        << "    \"wait_id\": \"test_spin\",\n"
        << "    \"backup_id\": \"test_spin\",\n"
        << "    \"goal_checker_id\": \"simple_goal_checker\",\n"
        << "    \"progress_checker_id\": \"simple_progress_checker\",\n"
        << "    \"planner_frequency\": 100.0,\n"
        << "    \"smoother_frequency\": -1.0,\n"
        << "    \"controller_frequency\": 20.0,\n"
        << "    \"tick_frequency\": 100.0,\n"
        << "    \"costmap_visualization_rate\": 100.0\n"
        << "  },\n"
        << "  \"costmap\": {\n"
        << "    \"global_frame\": \"map\",\n"
        << "    \"local_frame\": \"odom\",\n"
        << "    \"robot_frame\": \"base_link\",\n"
        << "    \"resolution\": 0.5\n"
        << "  },\n"
        << "  \"costmap_global\": {\n"
        << "    \"size_x\": 20,\n    \"size_y\": 20,\n"
        << "    \"rolling_window\": false,\n"
        << "    \"track_unknown\": false,\n"
        << "    \"static_map\": \"" << pgm_path << "\",\n"
        << "    \"inflation_radius\": 1.0,\n"
        << "    \"inscribed_radius\": 0.8,\n"
        << "    \"cost_scaling_factor\": 2.0\n"
        << "  },\n"
        << "  \"costmap_local\": {\n"
        << "    \"size_x\": 20,\n    \"size_y\": 20,\n"
        << "    \"rolling_window\": true,\n"
        << "    \"track_unknown\": false,\n"
        << "    \"inflation_radius\": 1.0,\n"
        << "    \"inscribed_radius\": 0.8,\n"
        << "    \"cost_scaling_factor\": 2.0\n"
        << "  }\n"
        << "}\n";
  }

  write_file(dir / "BT" / "navigation.xml",
             R"(<root BTCPP_format="4" main_tree_to_execute="NavigateWithReplanningAndSmoothing">
  <BehaviorTree ID="NavigateWithReplanningAndSmoothing">
    <RecoveryNode name="NavigateRecovery" number_of_retries="6">
      <PipelineSequence name="NavigateWithReplanning">
        <RateController hz="{planner_frequency}">
          <RecoveryNode name="ComputePath" number_of_retries="1">
            <Sequence>
              <ComputePathToPose goal="{goal}" path="{raw_path}"/>
              <IsPathValid path="{raw_path}"/>
              <SmoothPath  input_path="{raw_path}" path="{smoothed_path}"/>
              <IsPathValid path="{smoothed_path}"/>
            </Sequence>
            <Sequence>
              <Recovery recovery="{clear_costmap_id}"/>
              <AlwaysFailure/>
            </Sequence>
          </RecoveryNode>
        </RateController>
        <RecoveryNode name="FollowPath" number_of_retries="1">
          <FollowPath path="{smoothed_path}" frequency="{controller_frequency}"/>
          <Sequence>
            <Recovery recovery="{clear_costmap_id}"/>
            <AlwaysFailure/>
          </Sequence>
        </RecoveryNode>
      </PipelineSequence>
      <ReactiveFallback name="RecoveryActions">
        <GoalUpdated goal="{goal}"/>
        <RoundRobin name="RecoveryActions">
          <Sequence><Recovery recovery="{spin_id}"/>    <AlwaysFailure/></Sequence>
          <Sequence><Recovery recovery="{wait_id}"/>    <AlwaysFailure/></Sequence>
          <Sequence><Recovery recovery="{backup_id}"/>  <AlwaysFailure/></Sequence>
        </RoundRobin>
      </ReactiveFallback>
    </RecoveryNode>
  </BehaviorTree>
</root>)");

  return dir;
}

// RAII 订阅句柄: attach 时替换, detach 时析构退订 (无悬挂 id)。
ayanami::CallbackSubscription g_cmd_cb;
ayanami::CallbackSubscription g_viz_cb;

void attachObservers() {
  g_cmds.clear();
  g_zero_after_preempt.store(0);
  g_preempt_armed.store(false);
  g_cmd_cb = ayanami::interfaces::NavigationCallbacks::on_velocity_command.subscribe(
      [](const TwistStamped& t) {
        std::lock_guard<std::mutex> lock(g_cmd_mutex);
        g_cmds.push_back(t);
        if (g_preempt_armed.load() && isZeroTwist(t)) {
          g_zero_after_preempt.fetch_add(1);
        }
      });
  g_viz_names.clear();
  g_viz_cb =
      ayanami::interfaces::NavigationCallbacks::on_costmap_visualization.subscribe(
          [](const std::string& name, const ayanami::nav_msgs::OccupancyGrid&) {
            std::lock_guard<std::mutex> lock(g_viz_mutex);
            g_viz_names.push_back(name);
          });
}

void detachObservers() {
  g_cmd_cb.reset();
  g_viz_cb.reset();
}

int nonZeroCmdCount() {
  std::lock_guard<std::mutex> lock(g_cmd_mutex);
  int count = 0;
  for (const auto& t : g_cmds) {
    if (!isZeroTwist(t)) {
      ++count;
    }
  }
  return count;
}

bool anyCmdHasNaNLinear() {
  std::lock_guard<std::mutex> lock(g_cmd_mutex);
  for (const auto& t : g_cmds) {
    if (!allFinite(t)) {
      return true;
    }
  }
  return false;
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

// 公共工厂装载 host 模块 → AbstractNavigation::Ptr → dynamic_cast<CostmapNavigation*>。
// 亲引用: 组件寿命 = 进程 (不 dlclose), 宿主 shared_ptr 特意泄漏保值,
// 测试仍负责 delete host。
CostmapNavigation* createHost(const std::filesystem::path& config_dir) {
  auto nav = std::dynamic_pointer_cast<ayanami::abstract::AbstractNavigation>(
      ayanami::ExtensionModule::load_module(g_module_so));
  EXPECT_NE(nav, nullptr);
  if (!nav) {
    return nullptr;
  }
  auto* host = dynamic_cast<CostmapNavigation*>(nav.get());
  EXPECT_NE(host, nullptr);
  if (host == nullptr) {
    return nullptr;
  }
  host->initialize(config_dir.string());
  (void)new std::shared_ptr<ayanami::abstract::AbstractNavigation>(
      std::move(nav));  // leak-persist
  return host;
}

}  // namespace

// ---------------- 宿主组装 (configureCoreMaps + loadPlugins +) ----------------

TEST(CostmapNavigationTest, dlopenAllAlgorithmModulesLoad) {
  // 4 个真实算法模块 .so 都走公共工厂 create_extension_module, 且经类型化
  // dynamic_pointer_cast 得到正确的插件族类型。
  EXPECT_NE(std::dynamic_pointer_cast<ayanami::costmap::CostmapPlanner>(
                ayanami::ExtensionModule::load_module(g_smac_so)),
            nullptr);
  EXPECT_NE(std::dynamic_pointer_cast<ayanami::costmap::CostmapSmoother>(
                ayanami::ExtensionModule::load_module(g_savitzky_so)),
            nullptr);
  EXPECT_NE(std::dynamic_pointer_cast<ayanami::abstract::AbstractController>(
                ayanami::ExtensionModule::load_module(g_pid_so)),
            nullptr);
  EXPECT_NE(std::dynamic_pointer_cast<ayanami::costmap::CostmapRecovery>(
                ayanami::ExtensionModule::load_module(g_spin_so)),
            nullptr);
}

TEST(CostmapNavigationTest, initializeBuildsFullStack) {
  const auto dir = makeConfigDir();
  CostmapNavigation* host = createHost(dir);
  ASSERT_NE(host, nullptr);

  // configureCoreMaps: global/local costmap 已建
  ASSERT_NE(host->navigationServer().costmapContext(), nullptr);
  ASSERT_NE(host->navigationServer().globalCostmap(), nullptr);
  ASSERT_NE(host->navigationServer().localCostmap(), nullptr);

  // loadPlugins: planner/smoother/controller/recovery action 已建
  ASSERT_NE(host->navigationServer().plannerAction(), nullptr);
  ASSERT_NE(host->navigationServer().smootherAction(), nullptr);
  ASSERT_NE(host->navigationServer().controllerAction(), nullptr);
  ASSERT_NE(host->navigationServer().recoveryAction("test_spin"), nullptr);

  // pilot 已 configure + loadFromFile + setNavigationRuntime
  // (loadFromFile 校验 17 板项通过 → 树已建)
  ASSERT_NE(host->pilot().blackboard(), nullptr);

  host->stop();
  delete host;
}

// ---------------- start / spinOnce 可视化旁路 ----------------

TEST(CostmapNavigationTest, startPublishesVisualizationAndSpinOnceRepeats) {
  attachObservers();
  const auto dir = makeConfigDir();
  CostmapNavigation* host = createHost(dir);
  ASSERT_NE(host, nullptr);

  host->start();
  EXPECT_TRUE(waitUntil([&] {
    std::lock_guard<std::mutex> lock(g_viz_mutex);
    return !g_viz_names.empty();
  }));

  bool has_global = false;
  bool has_local = false;
  {
    std::lock_guard<std::mutex> lock(g_viz_mutex);
    for (const auto& n : g_viz_names) {
      if (n == "global_costmap") has_global = true;
      if (n == "local_costmap") has_local = true;
    }
  }
  EXPECT_TRUE(has_global);
  EXPECT_TRUE(has_local);

  // spinOnce: 限频 (100Hz) 下连续调用应再次触发。
  const size_t before = g_viz_names.size();
  for (int i = 0; i < 5; ++i) {
    host->spinOnce();
    std::this_thread::sleep_for(std::chrono::milliseconds(15));
  }
  EXPECT_GT(g_viz_names.size(), before);

  host->stop();
  delete host;
  detachObservers();
}

// ---------------- setGoal: BT 真跑通 path→cmd_vel ----------------

TEST(CostmapNavigationTest, setGoalRunsBtAndEmitsVelocity) {
  attachObservers();
  const auto dir = makeConfigDir();
  CostmapNavigation* host = createHost(dir);
  ASSERT_NE(host, nullptr);
  host->start();

  host->setGoal(makePose(2.0, 0.0));

  const auto bb = host->pilot().blackboard();
  ASSERT_NE(bb, nullptr);

  // 100ms 内 raw_path / smoothed_path 写入 pilot 黑板
  EXPECT_TRUE(waitUntil([&] {
    Path raw;
    if (!bb->get<Path>(k_raw_path, raw) || raw.poses.empty()) {
      return false;
    }
    Path smoothed;
    return bb->get<Path>(k_smoothed_path, smoothed) && !smoothed.poses.empty();
  }, 2000));

  // BT 主流程推进 → controller 经 on_velocity_command 旁路发非零速度
  EXPECT_TRUE(waitUntil([&] { return nonZeroCmdCount() >= 1; }, 2000));

  // 首帧 cmd_vel.twist.linear 已初始化 (非 NaN)
  EXPECT_FALSE(anyCmdHasNaNLinear());

  host->stop();
  delete host;
  detachObservers();
}

// ---------------- Goal revision: controller keeps running + same session/tree ----------------

TEST(CostmapNavigationTest, newGoalKeepsControllerAndSameSessionTree) {
  attachObservers();
  const auto dir = makeConfigDir();
  CostmapNavigation* host = createHost(dir);
  ASSERT_NE(host, nullptr);
  host->start();

  host->setGoal(makePose(2.0, 0.0));
  const auto bb = host->pilot().blackboard();
  ASSERT_NE(bb, nullptr);

  ASSERT_TRUE(waitUntil([&] { return nonZeroCmdCount() >= 1; }, 2000));

  uint64_t sid_before = 0;
  ASSERT_TRUE(bb->get<uint64_t>(k_session_id, sid_before));
  const void* tree_addr = &host->pilot().tree();

  uint64_t revision_before = 0;
  ASSERT_TRUE(bb->get<uint64_t>(k_goal_revision, revision_before));
  g_preempt_armed.store(true);
  host->setGoal(makePose(0.5, 0.5));
  EXPECT_TRUE(waitUntil([&] {
    uint64_t revision = 0;
    return bb->get<uint64_t>(k_goal_revision, revision) &&
           revision > revision_before;
  }, 1000));
  g_preempt_armed.store(false);
  EXPECT_TRUE(waitUntil([&] { return nonZeroCmdCount() >= 1; }, 1000));

  uint64_t sid_after = 0;
  ASSERT_TRUE(bb->get<uint64_t>(k_session_id, sid_after));
  EXPECT_EQ(sid_after, sid_before);

  // BT 树不重建: 同一 tree 实例
  EXPECT_EQ(&host->pilot().tree(), tree_addr);

  host->stop();
  delete host;
  detachObservers();
}

// ---------------- 析构顺序 ----------------

TEST(CostmapNavigationTest, destructorDoesNotCrash) {
  const auto dir = makeConfigDir();
  {
    CostmapNavigation* host = createHost(dir);
    ASSERT_NE(host, nullptr);
    host->start();
    host->setGoal(makePose(2.0, 0.0));
    // 作用域结束 → delete: stop join nav thread → pilot stop → server stop
    // (clearActions 先于 clearPlugins, BT 节点不再持有过期插件指针)
    delete host;
  }
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  if (argc > 1) g_module_so = argv[1];
  if (argc > 2) g_smac_so = argv[2];
  if (argc > 3) g_savitzky_so = argv[3];
  if (argc > 4) g_pid_so = argv[4];
  if (argc > 5) g_test_controller_so = argv[5];
  if (argc > 6) g_spin_so = argv[6];
  if (argc > 7) g_goal_checker_so = argv[7];
  if (argc > 8) g_progress_checker_so = argv[8];
  return RUN_ALL_TESTS();
}
