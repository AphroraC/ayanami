// Full config/*.json suite + id<->name consistency check (/ §9).
// R1 (Controller option a): *_id values are uniformly the plugin full name (including the _recovery suffix),
// so BT blackboard writes like {spin_id} directly equal the PluginManager/recovery_actions name key,
// and requestRecovery no longer misses (fixes the historical pitfall of recovery_id not matching the plugin name).
//
// Test layers:
// A. Pure config layer: load config/ -> read the costmap_navigation.json ids -> assert each id value
// equals the name field of the corresponding plugin json (full plugin name).
// B. Real dlopen: PluginManager<Abstract* family> loads the real module .so files; the loaded names match
// the costmap_navigation.json id values.
// C. Host init: dlopen host -> initialize(config/) -> enumerate the loaded recovery actions.
// D. Real requestRecovery calls: full name -> valid TaskHandle; unsuffixed id -> invalid
// (NOT FOUND; proves negatively that the old unsuffixed id is now truly a miss).
// E. BT recovery branch: run the real BT tree <Recovery recovery="{spin_id}"/> -> triggers
// requestRecovery(full name) -> the costmap_spin_recovery plugin is actually invoked
// (the on_velocity_command angular-velocity events = the runRecovery count).
//
// argv: 1=config_dir, 2=host.so, 3=smac.so, 4=savitzky.so, 5=pid.so,
// 6=spin.so, 7=back_up.so, 8=wait.so, 9=clear.so

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

// The *_id field set of costmap_navigation.json. After R1, id values == the plugin full name.
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

// The ordered config_recovery file family lives in the navigation section of config_navigation
// (a single string or a string array), resolved to full paths relative to config_path.
std::vector<std::string> recoveryFamilyFiles() {
  const ayanami::GlobalConfig* g = ayanami::GlobalConfig::instance();
  const ayanami::Config navigation(ayanami::GlobalConfig::getConfigPath(
      "config_navigation"));
  std::vector<std::string> entries;
  if (navigation.hasParam("navigation", "config_recovery")) {
    try {
      entries = navigation.param<std::vector<std::string>>(
          "navigation", "config_recovery", {});
    } catch (const std::exception&) {
      // config_recovery is a single string.
    }
    if (entries.empty()) {
      const std::string single =
          navigation.param<std::string>("navigation", "config_recovery", "");
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

/// Reads the name field of a plugin json. module_section is the top-level json section name.
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

/// Locates the plugin json by role in the config_recovery array and reads its name (unified common factory symbol).
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

/// dlopen the host module (common factory) -> AbstractNavigation -> CostmapNavigation*, already initialized.
/// Owning reference: component lifetime = process lifetime (no dlclose); the host shared_ptr is intentionally leaked to stay alive,
/// and the test is still responsible for deleting the host.
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

// ---- A. Pure config layer: id value == plugin full name ----

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

  // planner/controller/smoother: name == id (already equal to the plugin name / PluginManager key).
  EXPECT_EQ(pluginName("config_planner", "planner"), ids.planner_id);
  EXPECT_EQ(pluginName("config_controller", "controller"), ids.controller_id);
  EXPECT_EQ(pluginName("config_smoother", "smoother"), ids.smoother_id);

  // recovery: the id value directly == the plugin name (full plugin name, including the _recovery suffix).
  // The BT {spin_id} blackboard write = costmap_spin_recovery -> requestRecovery hits directly.
  EXPECT_EQ(ids.spin_id, "costmap_spin_recovery");
  EXPECT_EQ(recoveryPluginName("spin"), ids.spin_id);

  EXPECT_EQ(ids.wait_id, "costmap_wait_recovery");
  EXPECT_EQ(recoveryPluginName("wait"), ids.wait_id);

  EXPECT_EQ(ids.backup_id, "costmap_back_up_recovery");
  EXPECT_EQ(recoveryPluginName("backup"), ids.backup_id);

  // clear_costmap_id and recovery_id point to the same plugin (full name).
  EXPECT_EQ(ids.recovery_id, ids.clear_costmap_id);
  EXPECT_EQ(ids.clear_costmap_id, "costmap_clear_costmap_recovery");
  EXPECT_EQ(recoveryPluginName("clear_costmap"), ids.recovery_id);
}

// ---- B. Real dlopen: PluginManager loaded names match costmap_navigation.json ----

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
    // All 4 recoveries loaded; each name must == the id value (full plugin name).
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

// ---- C. Host init: dlopen host -> initialize(config/) -> loaded names match the json ----

TEST(ConfigConsistencyTest, hostInitializeLoadsConsistentPlugins) {
  CostmapNavigation* host = createHost(g_config_dir);
  ASSERT_NE(host, nullptr);

  // loadPlugins loads via config_planner/.../config_recovery -> createActions.
  // pid is already in the costmap family (CostmapController), so the costmap host can inject it;
  // the controller action is created as well (all four costmap families accepted).
  ASSERT_NE(host->navigationServer().plannerAction(), nullptr);
  ASSERT_NE(host->navigationServer().smootherAction(), nullptr);
  ASSERT_NE(host->navigationServer().controllerAction(), nullptr);

  // recovery: the clear_costmap plugin action pointed to by config_recovery has been created
  // (loaded name = the clear_costmap_id value, i.e. the full plugin name).
  const NavIds ids = readNavIds();
  ASSERT_NE(host->navigationServer().recoveryAction(ids.clear_costmap_id),
            nullptr);

  // Enumerate the recovery action names; must include the plugin name for clear_costmap.
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

// ---- D. Real requestRecovery calls: full name -> valid; unsuffixed -> invalid ----

TEST(ConfigConsistencyTest, requestRecoveryUsesFullPluginName) {
  CostmapNavigation* host = createHost(g_config_dir);
  ASSERT_NE(host, nullptr);

  const NavIds ids = readNavIds();

  // Full plugin name (the recovery_id value = costmap_clear_costmap_recovery) -> valid.
  {
    RecoveryGoal goal;
    goal.recovery_id = ids.recovery_id;
    auto task = host->navigationServer().requestRecovery(goal);
    EXPECT_TRUE(task.valid());
    if (task.valid()) {
      task.cancel();
      // Recovery is exclusive: only after the previous recovery task completes does its onComplete release
      // recovery_exclusive. Wait for its terminal state; otherwise the next requestRecovery reports RecoveryFailed
      // because "another recovery is active" (semantically aliased with an invalid id).
      ASSERT_TRUE(waitUntil([&task] { return task.ready(); }));
    }
  }
  // Negative proof: the old unsuffixed id (costmap_clear_costmap) is now truly NOT FOUND -> a structured
  // failure TaskHandle (valid + ABORTED + InvalidInput), not a silent fallback to the default.
  {
    RecoveryGoal goal;
    goal.recovery_id = "costmap_clear_costmap";
    auto task = host->navigationServer().requestRecovery(goal);
    ASSERT_TRUE(task.valid());
    ASSERT_TRUE(task.ready());
    EXPECT_EQ(task.status(), ayanami::actions::GoalStatus::ABORTED);
    EXPECT_EQ(task.result().error, ayanami::ErrorCode::INVALID_INPUT);
  }
  // The config_recovery array now loads all 4 recovery plugins (WIP one-config-many-files),
  // and the spin full name is loaded too -> valid.
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

// ---- E. BT recovery branch: <Recovery recovery="{spin_id}"/> actually invokes the spin plugin ----

namespace {

// Minimal recovery BT tree: a single Recovery node whose port comes from the blackboard {spin_id} (the full name after R1).
constexpr const char* k_recovery_only_xml = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="recovery_only">
    <Recovery recovery="{spin_id}"/>
  </BehaviorTree>
</root>)";

std::atomic<int> g_spin_velocity_cmds{0};
// RAII subscription handle: replaced on attach, unsubscribed on destruction at detach (no dangling id).
ayanami::CallbackSubscription g_spin_cb;

void attachSpinObserver() {
  g_spin_velocity_cmds.store(0);
  g_spin_cb = ayanami::interfaces::NavigationCallbacks::on_velocity_command.subscribe(
      [](const TwistStamped& t) {
        // Each spin recovery tick emits a non-zero angular velocity -> proves the plugin is actually invoked.
        if (std::abs(t.twist.angular.z()) > 1e-6) {
          g_spin_velocity_cmds.fetch_add(1);
        }
      });
}

void detachSpinObserver() {
  g_spin_cb.reset();
}

void writeFile(const std::filesystem::path& path,
               const std::string& content) {
  std::ofstream ofs(path);
  ofs << content;
}

// Build a temp config dir: config_recovery points at the spin recovery plugin (full name) so the host
// loadPlugins loads costmap_spin_recovery into recovery_actions. The other plugin jsons
// can be omitted (loadPlugins falls back to empty config for missing files, no crash).
std::filesystem::path makeSpinConfigDir() {
  std::filesystem::path dir = std::filesystem::temp_directory_path() /
                              "ayanami_config_consistency_spin";
  std::filesystem::remove_all(dir);
  std::filesystem::create_directories(dir);
  std::filesystem::create_directories(dir / "BT");

  writeFile(dir / "config.json",
             "{\n  \"global\": {\n"
             "    \"config_navigation\": \"costmap_navigation.json\"\n"
             "  }\n}\n");

  writeFile(dir / "costmap_navigation.json",
             "{\n  \"navigation\": {\n"
             "    \"behavior_tree_extension\": \"" AYANAMI_TEST_BT_EXT_SO "\",\n"
             "    \"behavior_tree_xml\": \"BT/navigation.xml\",\n"
             "    \"behavior_tree_id\": \"NavigateWithReplanningAndSmoothing\",\n"
             "    \"config_recovery\": [\"spin_recovery.json\"],\n"
             "    \"clear_costmap_id\": \"costmap_spin_recovery\",\n"
             "    \"spin_id\": \"costmap_spin_recovery\",\n"
             "    \"wait_id\": \"costmap_spin_recovery\",\n"
             "    \"backup_id\": \"costmap_spin_recovery\",\n"
             "    \"planner_frequency\": -1.0,\n"
             "    \"smoother_frequency\": -1.0,\n"
             "    \"controller_frequency\": 20.0,\n"
             "    \"tick_frequency\": 100.0\n"
             "  }\n");

  // Spin recovery plugin: full name + a real .so. spin_dist is set small to shorten the test.
  writeFile(dir / "spin_recovery.json",
             "{\n  \"recovery\": {\n"
             "    \"name\": \"costmap_spin_recovery\",\n"
             "    \"so_name\": \"" +
                 g_spin_so +
                 "\",\n"
                 "    \"role\": \"clear_costmap\",\n"
                 "    \"spin_dist\": 0.1,\n    \"max_w\": 1.0,\n"
                 "    \"sim_cycle_dt\": 0.01,\n    \"collision_radius\": 0.3\n"
                 "  }\n}\n");

  // Minimal global/local costmap (needed by configureCoreMaps; merged into the navigation file).
  {
    std::ofstream nav(dir / "costmap_navigation.json", std::ios::app);
    nav << ",\n  \"costmap\": {\n"
        << "    \"map_frame\": \"map\",\n"
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

  // pilot.loadFromFile needs BT/navigation.xml (id uses the default tree name).
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

  return dir;
}

}  // namespace

TEST(ConfigConsistencyTest, btRecoveryBranchInvokesSpinPlugin) {
  const auto dir = makeSpinConfigDir();
  CostmapNavigation* host = createHost(dir.string());
  ASSERT_NE(host, nullptr);

  // The spin plugin was loaded into recovery_actions by loadPlugins (name=costmap_spin_recovery).
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

  // First tick: onStart -> requestRecovery("costmap_spin_recovery") -> hit -> RUNNING.
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);

  // spin runRecovery emits angular velocity every frame -> wait for the plugin to be actually invoked (runRecovery count >= 1).
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
