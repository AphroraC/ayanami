// Task 26: config/*.json 全套 + id↔name 一致性检查 (spec §6.6 / §9)。
// R1 (Controller Ruling #8 选项 a): *_id 值统一为插件 full name (含 _recovery 后缀),
// 使 BT `{spin_id}` 等黑板书值直接等于 PluginManager/recovery_actions 的 name key,
// requestRecovery 不再 miss (修历史 recovery_id 与插件 name 不匹配的坑)。
//
// 测试分层:
//   A. 纯配置层: 加载 config/ → 读 costmap_navigation.json ids → 断言每个 id 值
//      就是对应插件 json 的 name 字段 (full plugin name)。
//   B. 真实 dlopen: PluginManager<Abstract* 族> 装载真实模块 .so, 装载名与
//      costmap_navigation.json id 值一致。
//   C. 宿主初始化: dlopen host → initialize(config/) → 枚举 loaded recovery action。
//   D. requestRecovery 实调: full name → valid TaskHandle; unsuffixed id → invalid
//      (NOT FOUND, 反向证明旧无后缀 id 现真是 miss)。
//   E. BT 恢复分支: 跑真实 BT 树 `<Recovery recovery="{spin_id}"/>` → 触发
//      requestRecovery(full name) → 插件 costmap_spin_recovery 实际被调用
//      (on_velocity_command 角速度事件 = runRecovery 计数)。
//
// argv: 1=config_dir, 2=host.so, 3=smac.so, 4=savitzky.so, 5=pid.so,
//       6=spin.so, 7=back_up.so, 8=wait.so, 9=clear.so

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <behaviortree_cpp/bt_factory.h>

#include <ayanami/behavior_tree/action_nodes.hpp>
#include <ayanami/behavior_tree/blackboard_protocol.hpp>
#include <ayanami/behavior_tree/navigation.hpp>
#include <ayanami/abstract/components/plugin_manager.hpp>
#include <ayanami/abstract/core/abstract_controller.hpp>
#include <ayanami/abstract/core/abstract_navigation.hpp>
#include <ayanami/abstract/core/abstract_planner.hpp>
#include <ayanami/abstract/core/abstract_recovery.hpp>
#include <ayanami/abstract/core/abstract_smoother.hpp>
#include <ayanami/interfaces/callbacks.hpp>
#include <ayanami/messages/actions.hpp>
#include <ayanami/types.hpp>
#include <ayanami/messages/geometry_msgs.hpp>
#include <costmap_navigation/costmap_navigation.hpp>
#include <ayanami/utility/config.hpp>
#include <ayanami/utility/extension_module.hpp>
#include <ayanami/utility/load_module.hpp>

namespace {

using ayanami::abstract::AbstractController;
using ayanami::abstract::AbstractNavigation;
using ayanami::abstract::AbstractPlanner;
using ayanami::abstract::AbstractRecovery;
using ayanami::abstract::AbstractSmoother;
using ayanami::actions::RecoveryGoal;
using ayanami::behavior_tree::k_navigation_runtime;
using ayanami::behavior_tree::k_spin_id;
using ayanami::components::PluginManager;
using ayanami::geometry_msgs::TwistStamped;
using ayanami::modules::CostmapNavigation;

std::string g_config_dir;
std::string g_host_so;
std::string g_smac_so;
std::string g_savitzky_so;
std::string g_pid_so;
std::string g_spin_so;
std::string g_backup_so;
std::string g_wait_so;
std::string g_clear_so;

// costmap_navigation.json 的 *_id 字段集合 (spec §9.3)。R1 后 id 值 == 插件 full name。
struct NavIds {
  std::string planner_id;
  std::string smoother_id;
  std::string controller_id;
  std::string recovery_id;
  std::string clear_costmap_id;
  std::string spin_id;
  std::string wait_id;
  std::string backup_id;
};

// 严格单层索引: main config.json global.config_recovery 为单字符串或字符串数组
// (数组即多插件文件族), 解析为相对 config_path 的完整路径。
std::vector<std::string> recoveryFamilyFiles() {
  const ayanami::GlobalConfig* g = ayanami::GlobalConfig::instance();
  std::vector<std::string> entries;
  if (g->hasParam("global", "config_recovery")) {
    try {
      entries = g->param<std::vector<std::string>>("global", "config_recovery",
                                                   {});
    } catch (const std::exception&) {
      // global.config_recovery 是单字符串。
    }
    if (entries.empty()) {
      const std::string single =
          g->param<std::string>("global", "config_recovery", "");
      if (!single.empty()) {
        entries.push_back(single);
      }
    }
  }
  const std::string directory =
      g->param<std::string>("global", "config_path", ".");
  std::vector<std::string> paths;
  paths.reserve(entries.size());
  for (const auto& entry : entries) {
    const std::filesystem::path p(entry);
    paths.push_back(p.is_absolute() ? entry : (directory + "/" + entry));
  }
  return paths;
}

NavIds readNavIds() {
  NavIds ids;
  const ayanami::GlobalConfig* global = ayanami::GlobalConfig::instance();
  const std::string directory =
      global->param<std::string>("global", "config_path", ".");
  const ayanami::Config navigation(ayanami::GlobalConfig::getConfigPath(
      "config_navigation"));
  const auto navPath = [&](const char* key) {
    const auto relative = navigation.param<std::string>("navigation", key, "");
    const std::filesystem::path path(relative);
    return path.is_absolute() ? path.string() : directory + "/" + relative;
  };
  {
    ayanami::Config cfg(navPath("config_planner"));
    ids.planner_id = cfg.param<std::string>("planner", "name", "");
  }
  {
    ayanami::Config cfg(navPath("config_smoother"));
    ids.smoother_id = cfg.param<std::string>("smoother", "name", "");
  }
  {
    ayanami::Config cfg(navPath("config_controller"));
    ids.controller_id = cfg.param<std::string>("controller", "name", "");
  }
  std::vector<std::string> recovery_files = navigation.param<std::vector<std::string>>(
      "navigation", "config_recovery", {});
  for (const auto& relative : recovery_files) {
    const std::filesystem::path candidate(relative);
    const std::string path = candidate.is_absolute()
                                 ? candidate.string()
                                 : directory + "/" + relative;
    ayanami::Config cfg(path);
    const std::string name = cfg.param<std::string>("recovery", "name", "");
    const std::string role = cfg.param<std::string>("recovery", "role", "");
    if (ids.recovery_id.empty() || role == "clear_costmap") {
      ids.recovery_id = name;
    }
    if (role == "clear_costmap") ids.clear_costmap_id = name;
    if (role == "spin") ids.spin_id = name;
    if (role == "wait") ids.wait_id = name;
    if (role == "backup") ids.backup_id = name;
  }
  return ids;
}

// 读某插件 json 的 name 字段。module_section 为 json 顶层段名。
std::string pluginName(const std::string& config_key,
                       const std::string& module_section) {
  const auto nav = ayanami::Config(ayanami::GlobalConfig::getConfigPath(
      "config_navigation"));
  const std::string directory = ayanami::GlobalConfig::instance()->param<std::string>(
      "global", "config_path", ".");
  const std::string relative = nav.param<std::string>(
      "navigation", config_key, "");
  const std::filesystem::path p(relative);
  ayanami::Config cfg(p.is_absolute() ? p.string() : directory + "/" + relative);
  return cfg.param<std::string>(module_section, "name", "");
}

// config_recovery 数组按 role 定位插件 json, 读 name (工厂统一公共符号)。
std::string recoveryPluginName(const std::string& role) {
  const auto nav = ayanami::Config(ayanami::GlobalConfig::getConfigPath(
      "config_navigation"));
  const std::string directory = ayanami::GlobalConfig::instance()->param<std::string>(
      "global", "config_path", ".");
  const auto files = nav.param<std::vector<std::string>>(
      "navigation", "config_recovery", {});
  for (const auto& relative : files) {
    const std::filesystem::path p(relative);
    const std::string path = p.is_absolute() ? p.string() : directory + "/" + relative;
    ayanami::Config cfg(path);
    if (cfg.param<std::string>("recovery", "role", "") == role) {
      return cfg.param<std::string>("recovery", "name", "");
    }
  }
  return "";
}

// dlopen host 模块 (公共工厂) → AbstractNavigation → CostmapNavigation*, 已 initialize。
// 亲引用: 组件寿命 = 进程 (spec §4.1, 不 dlclose), 宿主 shared_ptr 特意泄漏保值,
// 测试仍负责 delete host。
CostmapNavigation* createHost(const std::string& config_dir) {
  auto nav = std::dynamic_pointer_cast<AbstractNavigation>(
      ayanami::ExtensionModule::load_module(g_host_so));
  if (!nav) {
    return nullptr;
  }
  auto* host = dynamic_cast<CostmapNavigation*>(nav.get());
  if (host == nullptr) {
    return nullptr;
  }
  host->initialize(config_dir);
  (void)new std::shared_ptr<AbstractNavigation>(std::move(nav));  // leak-persist
  return host;
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

// ---- A. 纯配置层: id 值 == 插件 full name ----

TEST(ConfigConsistencyTest, allIdsNonEmpty) {
  ayanami::GlobalConfig::instance(g_config_dir, /*override_path=*/true);
  const NavIds ids = readNavIds();
  EXPECT_FALSE(ids.planner_id.empty());
  EXPECT_FALSE(ids.smoother_id.empty());
  EXPECT_FALSE(ids.controller_id.empty());
  EXPECT_FALSE(ids.recovery_id.empty());
  EXPECT_FALSE(ids.clear_costmap_id.empty());
  EXPECT_FALSE(ids.spin_id.empty());
  EXPECT_FALSE(ids.wait_id.empty());
  EXPECT_FALSE(ids.backup_id.empty());
}

TEST(ConfigConsistencyTest, idsAreFullPluginNames) {
  ayanami::GlobalConfig::instance(g_config_dir, /*override_path=*/true);
  const NavIds ids = readNavIds();

  // planner/controller/smoother: name == id (本就等于插件 name / PluginManager key)。
  EXPECT_EQ(pluginName("config_planner", "planner"), ids.planner_id);
  EXPECT_EQ(pluginName("config_controller", "controller"), ids.controller_id);
  EXPECT_EQ(pluginName("config_smoother", "smoother"), ids.smoother_id);

  // recovery: id 值直接 == 插件 name (full plugin name, 含 _recovery 后缀)。
  // BT `{spin_id}` 黑板书值 = costmap_spin_recovery → requestRecovery 直接命中。
  EXPECT_EQ(ids.spin_id, "costmap_spin_recovery");
  EXPECT_EQ(recoveryPluginName("spin"), ids.spin_id);

  EXPECT_EQ(ids.wait_id, "costmap_wait_recovery");
  EXPECT_EQ(recoveryPluginName("wait"), ids.wait_id);

  EXPECT_EQ(ids.backup_id, "costmap_back_up_recovery");
  EXPECT_EQ(recoveryPluginName("backup"), ids.backup_id);

  // clear_costmap_id 与 recovery_id 指向同一插件 (full name)。
  EXPECT_EQ(ids.recovery_id, ids.clear_costmap_id);
  EXPECT_EQ(ids.clear_costmap_id, "costmap_clear_costmap_recovery");
  EXPECT_EQ(recoveryPluginName("clear_costmap"), ids.recovery_id);
}

// ---- B. 真实 dlopen: PluginManager 装载名与 costmap_navigation.json 一致 ----

TEST(ConfigConsistencyTest, pluginManagersLoadMatchingNames) {
  ayanami::GlobalConfig::instance(g_config_dir, /*override_path=*/true);
  const NavIds ids = readNavIds();

  {
    PluginManager<AbstractPlanner> mgr;
    ayanami::components::PluginSpec spec;
    spec.name = pluginName("config_planner", "planner");
    spec.so_name = g_smac_so;
    ASSERT_TRUE(
        mgr.loadPlugin(spec, [](const std::string&, std::shared_ptr<AbstractPlanner>) {
          return true;
        }));
    EXPECT_TRUE(mgr.hasPlugin(ids.planner_id));
    EXPECT_EQ(mgr.getLoadedNames().size(), 1u);
  }
  {
    PluginManager<AbstractSmoother> mgr;
    ayanami::components::PluginSpec spec;
    spec.name = pluginName("config_smoother", "smoother");
    spec.so_name = g_savitzky_so;
    ASSERT_TRUE(
        mgr.loadPlugin(spec, [](const std::string&, std::shared_ptr<AbstractSmoother>) {
          return true;
        }));
    EXPECT_TRUE(mgr.hasPlugin(ids.smoother_id));
  }
  {
    PluginManager<AbstractController> mgr;
    ayanami::components::PluginSpec spec;
    spec.name = pluginName("config_controller", "controller");
    spec.so_name = g_pid_so;
    ASSERT_TRUE(
        mgr.loadPlugin(spec, [](const std::string&, std::shared_ptr<AbstractController>) {
          return true;
        }));
    EXPECT_TRUE(mgr.hasPlugin(ids.controller_id));
  }
  {
    // 4 个 recovery 全装载, 名必须 == id 值 (full plugin name)。
    PluginManager<AbstractRecovery> mgr;
    const struct {
      const char* role;
      const char* so;
    } k_rec[] = {
        {"spin", g_spin_so.c_str()},
        {"backup", g_backup_so.c_str()},
        {"wait", g_wait_so.c_str()},
        {"clear_costmap", g_clear_so.c_str()},
    };
    for (const auto& r : k_rec) {
      ayanami::components::PluginSpec spec;
      spec.name = recoveryPluginName(r.role);
      spec.so_name = r.so;
      ASSERT_TRUE(mgr.loadPlugin(
          spec, [](const std::string&, std::shared_ptr<AbstractRecovery>) {
            return true;
          }))
          << r.role;
    }
    EXPECT_TRUE(mgr.hasPlugin(ids.spin_id));
    EXPECT_TRUE(mgr.hasPlugin(ids.backup_id));
    EXPECT_TRUE(mgr.hasPlugin(ids.wait_id));
    EXPECT_TRUE(mgr.hasPlugin(ids.clear_costmap_id));
    EXPECT_EQ(mgr.getLoadedNames().size(), 4u);
  }
}

// ---- C. 宿主初始化: dlopen host → initialize(config/) → 装载名与 json 一致 ----

TEST(ConfigConsistencyTest, hostInitializeLoadsConsistentPlugins) {
  CostmapNavigation* host = createHost(g_config_dir);
  ASSERT_NE(host, nullptr);

  // loadPlugins 按 config_planner/.../config_recovery 装载 → createActions。
  // pid 已是 costmap 族 (CostmapController), 可被 costmap 宿主注入,
  // controller action 一并创建 (四族 costmap 全接受)。
  ASSERT_NE(host->navigationServer().plannerAction(), nullptr);
  ASSERT_NE(host->navigationServer().smootherAction(), nullptr);
  ASSERT_NE(host->navigationServer().controllerAction(), nullptr);

  // recovery: config_recovery 指向的 clear_costmap 插件 action 已建
  // (装载名 = clear_costmap_id 值, 即 full plugin name)。
  const NavIds ids = readNavIds();
  ASSERT_NE(host->navigationServer().recoveryAction(ids.clear_costmap_id),
            nullptr);

  // 枚举恢复 action 名, 必须含 clear_costmap 对应插件名。
  bool found = false;
  for (const auto& name : host->navigationServer().recoveryActionNames()) {
    if (name == ids.clear_costmap_id) {
      found = true;
    }
  }
  EXPECT_TRUE(found);

  host->stop();
  delete host;
}

// ---- D. requestRecovery 实调: full name → valid; unsuffixed → invalid ----

TEST(ConfigConsistencyTest, requestRecoveryUsesFullPluginName) {
  CostmapNavigation* host = createHost(g_config_dir);
  ASSERT_NE(host, nullptr);

  const NavIds ids = readNavIds();

  // full plugin name (recovery_id 值 = costmap_clear_costmap_recovery) → valid。
  {
    RecoveryGoal goal;
    goal.recovery_id = ids.recovery_id;
    auto task = host->navigationServer().requestRecovery(goal);
    EXPECT_TRUE(task.valid());
    if (task.valid()) {
      task.cancel();
      // recovery 是独占的: 上一条 recovery 任务完成后其 onComplete 才释放
      // recovery_exclusive。等待其终态, 否则下一条 requestRecovery 会因
      // "another recovery is active" 误报 RecoveryFailed (与无效 id 语义混叠)。
      ASSERT_TRUE(waitUntil([&task] { return task.ready(); }));
    }
  }
  // 反向证明: 旧无后缀 id (costmap_clear_costmap) 现真是 NOT FOUND → 结构化
  // 失败 TaskHandle (valid + ABORTED + InvalidInput), 不静默回退默认。
  {
    RecoveryGoal goal;
    goal.recovery_id = "costmap_clear_costmap";
    auto task = host->navigationServer().requestRecovery(goal);
    ASSERT_TRUE(task.valid());
    ASSERT_TRUE(task.ready());
    EXPECT_EQ(task.status(), ayanami::actions::GoalStatus::ABORTED);
    EXPECT_EQ(task.result().error, ayanami::ErrorCode::INVALID_INPUT);
  }
  // config_recovery 数组现装载全部 4 个 recovery 插件 (WIP 一配多文件),
  // spin full name 也已装载 → valid。
  {
    RecoveryGoal goal;
    goal.recovery_id = "costmap_spin_recovery";
    auto task = host->navigationServer().requestRecovery(goal);
    EXPECT_TRUE(task.valid());
    if (task.valid()) {
      task.cancel();
    }
  }

  host->stop();
  delete host;
}

// ---- E. BT 恢复分支: `<Recovery recovery="{spin_id}"/>` 实际调用 spin 插件 ----

namespace {

// 最小 recovery BT 树: 单 Recovery 节点, 端口来自黑板 {spin_id} (R1 后为 full name)。
constexpr const char* k_recovery_only_xml = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="recovery_only">
    <Recovery recovery="{spin_id}"/>
  </BehaviorTree>
</root>)";

std::atomic<int> g_spin_velocity_cmds{0};
int g_spin_cb_id = -1;

void attachSpinObserver() {
  g_spin_velocity_cmds.store(0);
  g_spin_cb_id = ayanami::interfaces::NavigationCallbacks::on_velocity_command.add(
      [](const TwistStamped& t) {
        // spin recovery runRecovery 每帧发非零角速度 → 证明插件被实际调用。
        if (std::abs(t.twist.angular.z()) > 1e-6) {
          g_spin_velocity_cmds.fetch_add(1);
        }
      });
}

void detachSpinObserver() {
  if (g_spin_cb_id >= 0) {
    ayanami::interfaces::NavigationCallbacks::on_velocity_command.remove(
        g_spin_cb_id);
    g_spin_cb_id = -1;
  }
}

void writeFile(const std::filesystem::path& path,
               const std::string& content) {
  std::ofstream ofs(path);
  ofs << content;
}

// 建临时 config 目录: config_recovery 指向 spin 恢复插件 (full name), 让宿主
// loadPlugins 把 costmap_spin_recovery 装入 recovery_actions。其余插件 json
// 缺省即可 (loadPlugins 对缺失文件回退空配置, 不 crash)。
std::filesystem::path makeSpinConfigDir() {
  std::filesystem::path dir = std::filesystem::temp_directory_path() /
                              "ayanami_config_consistency_spin";
  std::filesystem::remove_all(dir);
  std::filesystem::create_directories(dir);
  std::filesystem::create_directories(dir / "BT");

  writeFile(dir / "config.json",
             "{\n  \"global\": {\n"
             "    \"config_navigation\": \"costmap_navigation.json\",\n"
             "    \"config_logging\": \"config_logging.json\",\n"
             "    \"config_recovery\": [\"spin_recovery.json\"],\n"
             "    \"config_spin_recovery\": \"spin_recovery.json\"\n"
             "  }\n}\n");

  writeFile(dir / "costmap_navigation.json",
             "{\n  \"navigation\": {\n"
             "    \"behavior_tree_extension\": \"" AYANAMI_TEST_BT_EXT_SO "\",\n"
             "    \"behavior_tree_xml\": \"BT/navigation.xml\",\n"
             "    \"behavior_tree_id\": \"NavigateWithReplanningAndSmoothing\",\n"
             "    \"recovery_id\": \"costmap_spin_recovery\",\n"
             "    \"clear_costmap_id\": \"costmap_spin_recovery\",\n"
             "    \"spin_id\": \"costmap_spin_recovery\",\n"
             "    \"wait_id\": \"costmap_spin_recovery\",\n"
             "    \"backup_id\": \"costmap_spin_recovery\",\n"
             "    \"planner_frequency\": -1.0,\n"
             "    \"smoother_frequency\": -1.0,\n"
             "    \"controller_frequency\": 20.0,\n"
             "    \"tick_frequency\": 100.0\n"
             "  }\n");

  // spin 恢复插件: full name + 真实 .so。spin_dist 设小以缩短测试时长。
  writeFile(dir / "spin_recovery.json",
             "{\n  \"recovery\": {\n"
             "    \"name\": \"costmap_spin_recovery\",\n"
             "    \"so_name\": \"" +
                 g_spin_so +
                 "\",\n"
                 "    \"spin_dist\": 0.1,\n    \"max_w\": 1.0,\n"
                 "    \"sim_cycle_dt\": 0.01,\n    \"collision_radius\": 0.3\n"
                 "  }\n}\n");

  // 最小 global/local costmap (configureCoreMaps 需要; 已并入 navigation 文件)。
  {
    std::ofstream nav(dir / "costmap_navigation.json", std::ios::app);
    nav << ",\n  \"costmap\": {\n"
        << "    \"global_frame\": \"map\",\n"
        << "    \"local_frame\": \"odom\",\n"
        << "    \"robot_frame\": \"base_link\",\n"
        << "    \"resolution\": 0.5\n"
        << "  },\n"
        << "  \"costmap_global\": {\n"
        << "    \"size_x\": 20, \"size_y\": 20,\n"
        << "    \"rolling_window\": false, \"track_unknown\": false,\n"
        << "    \"static_map\": \"\",\n"
        << "    \"inflation_radius\": 0.6, \"inscribed_radius\": 0.3,\n"
        << "    \"cost_scaling_factor\": 3.0\n"
        << "  },\n"
        << "  \"costmap_local\": {\n"
        << "    \"size_x\": 20, \"size_y\": 20,\n"
        << "    \"rolling_window\": true, \"track_unknown\": false,\n"
        << "    \"inflation_radius\": 0.6, \"inscribed_radius\": 0.3,\n"
        << "    \"cost_scaling_factor\": 3.0\n"
        << "  }\n"
        << "}\n";
  }

  // pilot.loadFromFile 需要 BT/navigation.xml (id 用默认树名)。
  writeFile(dir / "BT" / "navigation.xml",
             R"(<root BTCPP_format="4" main_tree_to_execute="NavigateWithReplanningAndSmoothing">
  <BehaviorTree ID="NavigateWithReplanningAndSmoothing">
    <RecoveryNode name="R" number_of_retries="1">
      <AlwaysFailure/>
      <Sequence>
        <Recovery recovery="{spin_id}"/>
        <AlwaysFailure/>
      </Sequence>
    </RecoveryNode>
  </BehaviorTree>
</root>)");

  writeFile(dir / "config_logging.json",
             "{\n  \"logging\": { \"log_dir\": \"logs\", \"log_level\": \"info\" }\n}\n");

  return dir;
}

}  // namespace

TEST(ConfigConsistencyTest, btRecoveryBranchInvokesSpinPlugin) {
  const auto dir = makeSpinConfigDir();
  CostmapNavigation* host = createHost(dir.string());
  ASSERT_NE(host, nullptr);

  // spin 插件已由 loadPlugins 装入 recovery_actions (name=costmap_spin_recovery)。
  ASSERT_NE(host->navigationServer().recoveryAction("costmap_spin_recovery"),
            nullptr);

  attachSpinObserver();

  auto bb = BT::Blackboard::create();
  auto runtime = ayanami::components::NavigationRuntime::Ptr(
      &host->navigationServer(), [](ayanami::components::NavigationRuntime*) {});
  bb->set<ayanami::components::NavigationRuntime::Ptr>(k_navigation_runtime,
                                                       runtime);
  bb->set<std::string>(k_spin_id, "costmap_spin_recovery");

  BT::BehaviorTreeFactory factory;
  ayanami::behavior_tree::registerNavigationBehaviorTreeNodes(factory);
  auto tree = factory.createTreeFromText(k_recovery_only_xml, bb);

  // 首 tick: onStart → requestRecovery("costmap_spin_recovery") → 命中 → RUNNING。
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);

  // spin runRecovery 逐帧发角速度 → 等插件被实际调用 (runRecovery 计数 >=1)。
  EXPECT_TRUE(waitUntil([&] { return g_spin_velocity_cmds.load() >= 1; }))
      << "costmap_spin_recovery plugin runRecovery not invoked";

  host->stop();
  detachSpinObserver();
  delete host;
}

}  // namespace

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  if (argc > 1) g_config_dir = argv[1];
  if (argc > 2) g_host_so = argv[2];
  if (argc > 3) g_smac_so = argv[3];
  if (argc > 4) g_savitzky_so = argv[4];
  if (argc > 5) g_pid_so = argv[5];
  if (argc > 6) g_spin_so = argv[6];
  if (argc > 7) g_backup_so = argv[7];
  if (argc > 8) g_wait_so = argv[8];
  if (argc > 9) g_clear_so = argv[9];
  return RUN_ALL_TESTS();
}
