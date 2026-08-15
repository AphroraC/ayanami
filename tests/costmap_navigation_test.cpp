// Task 25: modules/costmap_navigation (AbstractNavigation 宿主插件)。
//
// M1 关键里程碑: 把核心库全部组件 (CostmapNavigationServer + 4 族插件 +
// BehaviorTreePilot) 组装为 dlopen 入口插件。本测试:
// - dlopen libayanami_costmap_navigation.so → create_navigation_module
// - initialize: configureCoreMaps + loadPlugins (dlopen smac/savitzky/spin
//   真实算法模块 + test controller) + createActions + pilot wiring (Ruling #5:
//   configure → loadFromFile → setNavigationServer)
// - start() 首发可视化 (on_costmap_visualization 旁路被捕获)
// - setGoal → navigation_thread_ 跑 BT → raw_path/smoothed_path 入黑板 +
//   on_velocity_command 收到非零速度 (首帧 cmd_vel.linear 已初始化非 NaN)
// - 抢占: 第二次 setGoal → 先发零速度 + session_id 自增 + BT 树不重建
// - 析构顺序不崩 (Ruling #3)

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
#include <ayanami/modules/costmap_navigation/costmap_navigation.hpp>
#include <ayanami/utility/load_module.hpp>

namespace {

using ayanami::behavior_tree::kRawPath;
using ayanami::behavior_tree::kSessionId;
using ayanami::behavior_tree::kSmoothedPath;
using ayanami::geometry_msgs::PoseStamped;
using ayanami::geometry_msgs::TwistStamped;
using ayanami::modules::CostmapNavigation;
using ayanami::nav_msgs::Path;

// argv: module .so + 5 插件 .so (smac/savitzky/pid/test_controller/spin)。
std::string g_module_so;
std::string g_smac_so;
std::string g_savitzky_so;
std::string g_pid_so;
std::string g_test_controller_so;
std::string g_spin_so;

// 捕获 on_velocity_command 事件。
std::vector<TwistStamped> g_cmds;
std::mutex g_cmd_mutex;
int g_cmd_cb_id = -1;
// 抢占标记: setGoal 预抢占后观察者记录零速度计数。
std::atomic<bool> g_preempt_armed{false};
std::atomic<int> g_zero_after_preempt{0};
// 可视化捕获。
std::vector<std::string> g_viz_names;
std::mutex g_viz_mutex;
int g_viz_cb_id = -1;

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
  p.pose.translation() = Eigen::Vector3d(x, y, 0.0);
  return p;
}

std::filesystem::path make_temp_dir() {
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

std::string plugin_json(const std::string& module, const std::string& name,
                        const std::string& so_name,
                        const std::string& symbol) {
  return "{\n  \"" + module + "\": {\n    \"name\": \"" + name +
         "\",\n    \"so_name\": \"" + so_name +
         "\",\n    \"create_symbol\": \"" + symbol + "\"\n  }\n}\n";
}

// 生成临时 config 目录。so_name 用 argv 注入的真实模块绝对路径。
std::filesystem::path make_config_dir() {
  const auto dir = make_temp_dir() / "cfg";
  std::filesystem::remove_all(dir);
  std::filesystem::create_directories(dir);
  std::filesystem::create_directories(dir / "BT");

  write_file(dir / "config.json",
             "{\n  \"global\": {\n"
             "    \"config_navigation\": \"costmap_navigation.json\",\n"
             "    \"config_costmap\": \"costmap.json\",\n"
             "    \"config_planner\": \"planner.json\",\n"
             "    \"config_smoother\": \"smoother.json\",\n"
             "    \"config_controller\": \"controller.json\",\n"
             "    \"config_recovery\": \"recovery.json\",\n"
             "    \"costmap_smac_planner\": \"costmap_smac_planner.json\",\n"
             "    \"costmap_savitzky_smoother\": \"costmap_savitzky_smoother.json\",\n"
             "    \"costmap_spin_recovery\": \"costmap_spin_recovery.json\"\n"
             "  }\n}\n");

  write_file(dir / "costmap_navigation.json",
             "{\n  \"navigation\": {\n"
             "    \"behavior_tree_xml\": \"BT/navigation.xml\",\n"
             "    \"behavior_tree_id\": \"NavigateWithReplanningAndSmoothing\",\n"
             "    \"planner_id\": \"test_planner\",\n"
             "    \"smoother_id\": \"test_smoother\",\n"
             "    \"controller_id\": \"test_controller\",\n"
             "    \"recovery_id\": \"test_spin\",\n"
             "    \"clear_costmap_id\": \"test_spin\",\n"
             "    \"spin_id\": \"test_spin\",\n"
             "    \"wait_id\": \"test_spin\",\n"
             "    \"backup_id\": \"test_spin\",\n"
             "    \"planner_frequency\": 100.0,\n"
             "    \"smoother_frequency\": -1.0,\n"
             "    \"controller_frequency\": 20.0,\n"
             "    \"tick_frequency\": 100.0,\n"
             "    \"costmap_visualization_rate\": 100.0\n"
             "  }\n}\n");

  write_file(dir / "planner.json",
             plugin_json("planner", "test_planner", g_smac_so,
                         "create_costmap_planner_module"));
  write_file(dir / "smoother.json",
             plugin_json("smoother", "test_smoother", g_savitzky_so,
                         "create_costmap_smoother_module"));
  write_file(dir / "controller.json",
             plugin_json("controller", "test_controller",
                         g_test_controller_so,
                         "create_navigation_test_controller_plugin"));
  write_file(dir / "recovery.json",
             plugin_json("recovery", "test_spin", g_spin_so,
                         "create_costmap_recovery_module"));

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

  write_file(dir / "costmap.json",
             "{\n  \"costmap\": {\n"
             "    \"global_frame\": \"map\",\n"
             "    \"local_frame\": \"odom\",\n"
             "    \"robot_frame\": \"base_link\",\n"
             "    \"resolution\": 0.5\n"
             "  },\n"
             "  \"costmap_global\": {\n"
             "    \"size_x\": 20,\n    \"size_y\": 20,\n"
             "    \"rolling_window\": false,\n"
             "    \"track_unknown\": false,\n"
             "    \"static_map\": \"" +
             pgm_path +
             "\",\n"
             "    \"inflation_radius\": 1.0,\n"
             "    \"inscribed_radius\": 0.8,\n"
             "    \"cost_scaling_factor\": 2.0\n"
             "  },\n"
             "  \"costmap_local\": {\n"
             "    \"size_x\": 20,\n    \"size_y\": 20,\n"
             "    \"rolling_window\": true,\n"
             "    \"track_unknown\": false,\n"
             "    \"inflation_radius\": 1.0,\n"
             "    \"inscribed_radius\": 0.8,\n"
             "    \"cost_scaling_factor\": 2.0\n"
             "  }\n}\n");

  // 各模块私有配置 (缺失也能跑, 提供则避免 spdlog 报错噪音)。
  write_file(dir / "costmap_smac_planner.json",
             "{\n  \"costmap_smac_planner\": {\n"
             "    \"use_diagonal\": true,\n    \"tolerance\": 0.0,\n"
             "    \"cost_factor\": 0.5,\n    \"lethal_cost\": 254,\n"
             "    \"neutral_cost\": 1.0,\n    \"use_final_approach\": false\n"
             "  }\n}\n");
  write_file(dir / "costmap_savitzky_smoother.json",
             "{\n  \"costmap_savitzky_smoother\": {\n"
             "    \"do_refinement\": true,\n    \"refinement_num\": 2\n"
             "  }\n}\n");
  write_file(dir / "costmap_spin_recovery.json",
             "{\n  \"recovery\": {\n"
             "    \"spin_dist\": 1.57,\n    \"max_w\": 0.5,\n"
             "    \"sim_cycle_dt\": 0.01,\n    \"collision_radius\": 0.3\n"
             "  }\n}\n");

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

void attachObservers() {
  g_cmds.clear();
  g_zero_after_preempt.store(0);
  g_preempt_armed.store(false);
  g_cmd_cb_id = ayanami::interfaces::NavigationCallbacks::on_velocity_command.add(
      [](const TwistStamped& t) {
        std::lock_guard<std::mutex> lock(g_cmd_mutex);
        g_cmds.push_back(t);
        if (g_preempt_armed.load() && isZeroTwist(t)) {
          g_zero_after_preempt.fetch_add(1);
        }
      });
  g_viz_names.clear();
  g_viz_cb_id =
      ayanami::interfaces::NavigationCallbacks::on_costmap_visualization.add(
          [](const std::string& name, const ayanami::nav_msgs::OccupancyGrid&) {
            std::lock_guard<std::mutex> lock(g_viz_mutex);
            g_viz_names.push_back(name);
          });
}

void detachObservers() {
  if (g_cmd_cb_id >= 0) {
    ayanami::interfaces::NavigationCallbacks::on_velocity_command.remove(
        g_cmd_cb_id);
    g_cmd_cb_id = -1;
  }
  if (g_viz_cb_id >= 0) {
    ayanami::interfaces::NavigationCallbacks::on_costmap_visualization.remove(
        g_viz_cb_id);
    g_viz_cb_id = -1;
  }
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

// dlopen 模块 → create_navigation_module → CostmapNavigation* (测试访问 pilot)。
CostmapNavigation* createHost(const std::filesystem::path& config_dir) {
  void* handle = ayanami::open_so(g_module_so);
  EXPECT_NE(handle, nullptr);
  using Factory = ayanami::abstract::AbstractNavigation* (*)();
  void* symbol = ayanami::load_symbol(handle, "create_navigation_module");
  EXPECT_NE(symbol, nullptr);
  auto factory = reinterpret_cast<Factory>(symbol);
  ayanami::abstract::AbstractNavigation* nav = factory();
  EXPECT_NE(nav, nullptr);
  auto* host = dynamic_cast<CostmapNavigation*>(nav);
  EXPECT_NE(host, nullptr);
  if (host == nullptr) {
    delete nav;
    return nullptr;
  }
  host->initialize(config_dir.string());
  return host;
}

}  // namespace

// ---------------- 宿主组装 (configureCoreMaps + loadPlugins + Ruling #5) ----------------

TEST(CostmapNavigationTest, DlopenAllAlgorithmModulesLoad) {
  // 4 个真实算法模块 .so 必须都能 dlopen 且导出 spec §4.3 工厂符号
  // (pid 是 AbstractController 族, 工厂返回 abstract::AbstractController*)。
  struct Symbol {
    const std::string& so;
    const char* sym;
  };
  const Symbol checks[] = {
      {g_smac_so, "create_costmap_planner_module"},
      {g_savitzky_so, "create_costmap_smoother_module"},
      {g_pid_so, "create_controller_module"},
      {g_spin_so, "create_costmap_recovery_module"},
  };
  for (const auto& c : checks) {
    void* h = ayanami::open_so(c.so);
    ASSERT_NE(h, nullptr) << "dlopen failed: " << c.so;
    void* s = ayanami::load_symbol(h, c.sym);
    ASSERT_NE(s, nullptr) << "symbol missing: " << c.sym;
  }
}

TEST(CostmapNavigationTest, InitializeBuildsFullStack) {
  const auto dir = make_config_dir();
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

  // Ruling #5: pilot 已 configure + loadFromFile + setNavigationServer
  // (loadFromFile 校验 16 板项通过 → 树已建)
  ASSERT_NE(host->pilot().blackboard(), nullptr);

  host->stop();
  delete host;
}

// ---------------- start / spinOnce 可视化旁路 ----------------

TEST(CostmapNavigationTest, StartPublishesVisualizationAndSpinOnceRepeats) {
  attachObservers();
  const auto dir = make_config_dir();
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

TEST(CostmapNavigationTest, SetGoalRunsBtAndEmitsVelocity) {
  attachObservers();
  const auto dir = make_config_dir();
  CostmapNavigation* host = createHost(dir);
  ASSERT_NE(host, nullptr);
  host->start();

  host->setGoal(makePose(2.0, 0.0));

  const auto bb = host->pilot().blackboard();
  ASSERT_NE(bb, nullptr);

  // 100ms 内 raw_path / smoothed_path 写入 pilot 黑板
  EXPECT_TRUE(waitUntil([&] {
    Path raw;
    if (!bb->get<Path>(kRawPath, raw) || raw.poses.empty()) {
      return false;
    }
    Path smoothed;
    return bb->get<Path>(kSmoothedPath, smoothed) && !smoothed.poses.empty();
  }, 2000));

  // BT 主流程推进 → controller 经 on_velocity_command 旁路发非零速度
  EXPECT_TRUE(waitUntil([&] { return nonZeroCmdCount() >= 1; }, 2000));

  // 首帧 cmd_vel.twist.linear 已初始化 (非 NaN)
  EXPECT_FALSE(anyCmdHasNaNLinear());

  host->stop();
  delete host;
  detachObservers();
}

// ---------------- 抢占: 零速度 + session_id 自增 + BT 树不重建 ----------------

TEST(CostmapNavigationTest, NewGoalPreemptsWithZeroVelocityAndSameTree) {
  attachObservers();
  const auto dir = make_config_dir();
  CostmapNavigation* host = createHost(dir);
  ASSERT_NE(host, nullptr);
  host->start();

  host->setGoal(makePose(2.0, 0.0));
  const auto bb = host->pilot().blackboard();
  ASSERT_NE(bb, nullptr);

  ASSERT_TRUE(waitUntil([&] { return nonZeroCmdCount() >= 1; }, 2000));

  uint64_t sid_before = 0;
  ASSERT_TRUE(bb->get<uint64_t>(kSessionId, sid_before));
  const void* tree_addr = &host->pilot().tree();

  // 新 goal 抢占: 先发零速度
  g_preempt_armed.store(true);
  host->setGoal(makePose(0.5, 0.5));
  EXPECT_TRUE(waitUntil([&] { return g_zero_after_preempt.load() >= 1; }, 1000));
  g_preempt_armed.store(false);

  // session_id 自增 (pilot.beginNavigation 内 session_id_++)
  EXPECT_TRUE(waitUntil([&] {
    uint64_t sid = 0;
    return bb->get<uint64_t>(kSessionId, sid) && sid > sid_before;
  }, 1000));

  // BT 树不重建: 同一 tree 实例
  EXPECT_EQ(&host->pilot().tree(), tree_addr);

  host->stop();
  delete host;
  detachObservers();
}

// ---------------- 析构顺序 (Ruling #3) ----------------

TEST(CostmapNavigationTest, DestructorDoesNotCrash) {
  const auto dir = make_config_dir();
  {
    CostmapNavigation* host = createHost(dir);
    ASSERT_NE(host, nullptr);
    host->start();
    host->setGoal(makePose(2.0, 0.0));
    // 作用域结束 → delete: stop() join nav thread → pilot stop → server stop
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
  return RUN_ALL_TESTS();
}
