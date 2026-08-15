// Task 26: config/*.json 全套 + id↔name 一致性检查 (spec §6.6 / §9)。
// R1 (Controller Ruling #8 选项 a): *_id 值统一为插件 full name (含 _recovery 后缀),
// 使 BT `{spin_id}` 等黑板书值直接等于 PluginManager/recovery_actions_ 的 name key,
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
#include <ayanami/components/plugin_manager.hpp>
#include <ayanami/core/abstract_controller.hpp>
#include <ayanami/core/abstract_navigation.hpp>
#include <ayanami/core/abstract_planner.hpp>
#include <ayanami/core/abstract_recovery.hpp>
#include <ayanami/core/abstract_smoother.hpp>
#include <ayanami/interfaces/callbacks.hpp>
#include <ayanami/messages/actions.hpp>
#include <ayanami/messages/geometry_msgs.hpp>
#include <ayanami/modules/costmap_navigation/costmap_navigation.hpp>
#include <ayanami/utility/config.hpp>
#include <ayanami/utility/load_module.hpp>

namespace {

using ayanami::abstract::AbstractController;
using ayanami::abstract::AbstractNavigation;
using ayanami::abstract::AbstractPlanner;
using ayanami::abstract::AbstractRecovery;
using ayanami::abstract::AbstractSmoother;
using ayanami::actions::RecoveryGoal;
using ayanami::behavior_tree::kNavigationServer;
using ayanami::behavior_tree::kSpinId;
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

NavIds readNavIds() {
  NavIds ids;
  ayanami::Config nav(ayanami::GlobalConfig::get_config_path("config_navigation"));
  const std::string kSec = "navigation";
  ids.planner_id = nav.param<std::string>(kSec, "planner_id", "");
  ids.smoother_id = nav.param<std::string>(kSec, "smoother_id", "");
  ids.controller_id = nav.param<std::string>(kSec, "controller_id", "");
  ids.recovery_id = nav.param<std::string>(kSec, "recovery_id", "");
  ids.clear_costmap_id = nav.param<std::string>(kSec, "clear_costmap_id", "");
  ids.spin_id = nav.param<std::string>(kSec, "spin_id", "");
  ids.wait_id = nav.param<std::string>(kSec, "wait_id", "");
  ids.backup_id = nav.param<std::string>(kSec, "backup_id", "");
  return ids;
}

// 读某插件 json 的 name 字段。module_section 为 json 顶层段名。
std::string pluginName(const std::string& config_key,
                       const std::string& module_section) {
  ayanami::Config cfg(ayanami::GlobalConfig::get_config_path(config_key));
  return cfg.param<std::string>(module_section, "name", "");
}

std::string pluginSymbol(const std::string& config_key,
                         const std::string& module_section) {
  ayanami::Config cfg(ayanami::GlobalConfig::get_config_path(config_key));
  return cfg.param<std::string>(module_section, "create_symbol", "");
}

// dlopen host 模块 → create_navigation_module → CostmapNavigation*, 已 initialize。
CostmapNavigation* createHost(const std::string& config_dir) {
  void* handle = ayanami::open_so(g_host_so);
  if (handle == nullptr) {
    return nullptr;
  }
  using Factory = AbstractNavigation* (*)();
  void* symbol = ayanami::load_symbol(handle, "create_navigation_module");
  if (symbol == nullptr) {
    return nullptr;
  }
  auto factory = reinterpret_cast<Factory>(symbol);
  AbstractNavigation* nav = factory();
  auto* host = dynamic_cast<CostmapNavigation*>(nav);
  if (host == nullptr) {
    delete nav;
    return nullptr;
  }
  host->initialize(config_dir);
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

TEST(ConfigConsistencyTest, AllIdsNonEmpty) {
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

TEST(ConfigConsistencyTest, IdsAreFullPluginNames) {
  ayanami::GlobalConfig::instance(g_config_dir, /*override_path=*/true);
  const NavIds ids = readNavIds();

  // planner/controller/smoother: name == id (本就等于插件 name / PluginManager key)。
  EXPECT_EQ(pluginName("config_planner", "planner"), ids.planner_id);
  EXPECT_EQ(pluginName("config_controller", "controller"), ids.controller_id);
  EXPECT_EQ(pluginName("config_smoother", "smoother"), ids.smoother_id);

  // recovery: id 值直接 == 插件 name (full plugin name, 含 _recovery 后缀)。
  // BT `{spin_id}` 黑板书值 = costmap_spin_recovery → requestRecovery 直接命中。
  EXPECT_EQ(ids.spin_id, "costmap_spin_recovery");
  EXPECT_EQ(pluginName("config_spin_recovery", "recovery"), ids.spin_id);

  EXPECT_EQ(ids.wait_id, "costmap_wait_recovery");
  EXPECT_EQ(pluginName("config_wait_recovery", "recovery"), ids.wait_id);

  EXPECT_EQ(ids.backup_id, "costmap_back_up_recovery");
  EXPECT_EQ(pluginName("config_back_up_recovery", "recovery"), ids.backup_id);

  // clear_costmap_id 与 recovery_id 指向同一插件 (full name)。
  EXPECT_EQ(ids.recovery_id, ids.clear_costmap_id);
  EXPECT_EQ(ids.clear_costmap_id, "costmap_clear_costmap_recovery");
  EXPECT_EQ(pluginName("config_recovery", "recovery"), ids.recovery_id);
  EXPECT_EQ(pluginName("config_clear_costmap_recovery", "recovery"),
            ids.recovery_id);
}

// ---- B. 真实 dlopen: PluginManager 装载名与 costmap_navigation.json 一致 ----

TEST(ConfigConsistencyTest, PluginManagersLoadMatchingNames) {
  ayanami::GlobalConfig::instance(g_config_dir, /*override_path=*/true);
  const NavIds ids = readNavIds();

  {
    PluginManager<AbstractPlanner> mgr;
    ayanami::components::PluginSpec spec;
    spec.name = pluginName("config_planner", "planner");
    spec.so_name = g_smac_so;
    spec.create_symbol = pluginSymbol("config_planner", "planner");
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
    spec.create_symbol = pluginSymbol("config_smoother", "smoother");
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
    spec.create_symbol = pluginSymbol("config_controller", "controller");
    ASSERT_TRUE(
        mgr.loadPlugin(spec, [](const std::string&, std::shared_ptr<AbstractController>) {
          return true;
        }));
    EXPECT_TRUE(mgr.hasPlugin(ids.controller_id));
  }
  {
    // 4 个 recovery 全装载, 名必须 == id 值 (full plugin name)。
    PluginManager<AbstractRecovery> mgr;
    const std::pair<const char*, const char*> kSecAndId[] = {
        {"config_spin_recovery", ids.spin_id.c_str()},
        {"config_back_up_recovery", ids.backup_id.c_str()},
        {"config_wait_recovery", ids.wait_id.c_str()},
        {"config_recovery", ids.clear_costmap_id.c_str()},
    };
    const std::string so[] = {g_spin_so, g_backup_so, g_wait_so, g_clear_so};
    for (int i = 0; i < 4; ++i) {
      ayanami::components::PluginSpec spec;
      spec.name = pluginName(kSecAndId[i].first, "recovery");
      spec.so_name = so[i];
      spec.create_symbol = pluginSymbol(kSecAndId[i].first, "recovery");
      ASSERT_TRUE(mgr.loadPlugin(
          spec, [](const std::string&, std::shared_ptr<AbstractRecovery>) {
            return true;
          }))
          << kSecAndId[i].first;
    }
    EXPECT_TRUE(mgr.hasPlugin(ids.spin_id));
    EXPECT_TRUE(mgr.hasPlugin(ids.backup_id));
    EXPECT_TRUE(mgr.hasPlugin(ids.wait_id));
    EXPECT_TRUE(mgr.hasPlugin(ids.clear_costmap_id));
    EXPECT_EQ(mgr.getLoadedNames().size(), 4u);
  }
}

// ---- C. 宿主初始化: dlopen host → initialize(config/) → 装载名与 json 一致 ----

TEST(ConfigConsistencyTest, HostInitializeLoadsConsistentPlugins) {
  CostmapNavigation* host = createHost(g_config_dir);
  ASSERT_NE(host, nullptr);

  // loadPlugins 按 config_planner/.../config_recovery 装载 → createActions。
  // 注意: 真实 pid 控制器是零地图 AbstractController (Task 23 设计),
  // 无法注入 costmap 族 host, 其 id↔name 一致性在 PluginManager<AbstractController>
  // 层单独验证 (PluginManagersLoadMatchingNames)。这里只断言 host 能装载的
  // costmap 族插件。
  ASSERT_NE(host->navigationServer().plannerAction(), nullptr);
  ASSERT_NE(host->navigationServer().smootherAction(), nullptr);

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

TEST(ConfigConsistencyTest, RequestRecoveryUsesFullPluginName) {
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
    }
  }
  // 反向证明: 旧无后缀 id (costmap_clear_costmap) 现真是 NOT FOUND → invalid。
  {
    RecoveryGoal goal;
    goal.recovery_id = "costmap_clear_costmap";
    auto task = host->navigationServer().requestRecovery(goal);
    EXPECT_FALSE(task.valid());
  }
  // 同理 spin: full name 已装载? host 只装 config_recovery(clear), spin 未装载,
  // 故 requestRecovery("costmap_spin_recovery") 应 invalid —— 一致性由 B 层
  // PluginManager 全装载保证, 宿主仅装默认 recovery (spec §4.5 纪律)。
  {
    RecoveryGoal goal;
    goal.recovery_id = "costmap_spin_recovery";
    auto task = host->navigationServer().requestRecovery(goal);
    EXPECT_FALSE(task.valid());
  }

  host->stop();
  delete host;
}

// ---- E. BT 恢复分支: `<Recovery recovery="{spin_id}"/>` 实际调用 spin 插件 ----

namespace {

// 最小 recovery BT 树: 单 Recovery 节点, 端口来自黑板 {spin_id} (R1 后为 full name)。
constexpr const char* kRecoveryOnlyXml = R"(
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
// loadPlugins 把 costmap_spin_recovery 装入 recovery_actions_。其余插件 json
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
             "    \"config_costmap\": \"costmap.json\",\n"
             "    \"config_recovery\": \"spin_recovery.json\",\n"
             "    \"config_logging\": \"logging.json\"\n"
             "  }\n}\n");

  writeFile(dir / "costmap_navigation.json",
             "{\n  \"navigation\": {\n"
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
             "  }\n}\n");

  // spin 恢复插件: full name + 真实 .so。spin_dist 设小以缩短测试时长。
  writeFile(dir / "spin_recovery.json",
             "{\n  \"recovery\": {\n"
             "    \"name\": \"costmap_spin_recovery\",\n"
             "    \"so_name\": \"" +
                 g_spin_so +
                 "\",\n"
                 "    \"create_symbol\": \"create_costmap_recovery_module\",\n"
                 "    \"spin_dist\": 0.1,\n    \"max_w\": 1.0,\n"
                 "    \"sim_cycle_dt\": 0.01,\n    \"collision_radius\": 0.3\n"
                 "  }\n}\n");

  // 最小 global/local costmap (configureCoreMaps 需要)。
  writeFile(dir / "costmap.json",
             "{\n  \"costmap\": {\n"
             "    \"global_frame\": \"map\",\n"
             "    \"local_frame\": \"odom\",\n"
             "    \"robot_frame\": \"base_link\",\n"
             "    \"resolution\": 0.5\n"
             "  },\n"
             "  \"costmap_global\": {\n"
             "    \"size_x\": 20, \"size_y\": 20,\n"
             "    \"rolling_window\": false, \"track_unknown\": false,\n"
             "    \"static_map\": \"\",\n"
             "    \"inflation_radius\": 0.6, \"inscribed_radius\": 0.3,\n"
             "    \"cost_scaling_factor\": 3.0\n"
             "  },\n"
             "  \"costmap_local\": {\n"
             "    \"size_x\": 20, \"size_y\": 20,\n"
             "    \"rolling_window\": true, \"track_unknown\": false,\n"
             "    \"inflation_radius\": 0.6, \"inscribed_radius\": 0.3,\n"
             "    \"cost_scaling_factor\": 3.0\n"
             "  }\n}\n");

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

  writeFile(dir / "logging.json",
             "{\n  \"logging\": { \"log_dir\": \"logs\", \"log_level\": \"info\" }\n}\n");

  return dir;
}

}  // namespace

TEST(ConfigConsistencyTest, BtRecoveryBranchInvokesSpinPlugin) {
  const auto dir = makeSpinConfigDir();
  CostmapNavigation* host = createHost(dir.string());
  ASSERT_NE(host, nullptr);

  // spin 插件已由 loadPlugins 装入 recovery_actions_ (name=costmap_spin_recovery)。
  ASSERT_NE(host->navigationServer().recoveryAction("costmap_spin_recovery"),
            nullptr);

  attachSpinObserver();

  auto bb = BT::Blackboard::create();
  bb->set<ayanami::components::NavigationServerBase*>(kNavigationServer,
                                                       &host->navigationServer());
  bb->set<std::string>(kSpinId, "costmap_spin_recovery");

  BT::BehaviorTreeFactory factory;
  ayanami::behavior_tree::registerNavigationBehaviorTreeNodes(factory);
  auto tree = factory.createTreeFromText(kRecoveryOnlyXml, bb);

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
