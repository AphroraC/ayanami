// Task 26: config/*.json 全套 + id↔name 一致性检查 (spec §6.6 / §9)。
//
// 修历史坑: recovery_id 与插件 name 不匹配 (如 recovery_id="costmap_cleanup" 但
// name="costmap_cleanup_recovery") 导致 BT 恢复分支实际失败。本测试从真实
// config/ 目录加载 costmap_navigation.json, 断言每个 *_id 字段都能反向解析到
// 对应插件 json 的 name 字段:
//   - planner/controller/smoother_id: name == id
//   - spin/wait/backup/clear_costmap/recovery_id: name == id + "_recovery"
//
// 分两层:
//   A. 纯配置层: 加载 config/ → 读 costmap_navigation.json ids → 读各插件 json
//      name 字段 → 断言反向解析成立 (不依赖 dlopen, 纯文件一致性)。
//   B. 真实 dlopen: 用 PluginManager<Abstract* 族> + argv 注入的真实模块 .so,
//      验证每个 id 对应的插件名出现在 getLoadedNames() (装载名 = name 字段)。
//   C. 宿主初始化: dlopen libayanami_costmap_navigation.so → initialize(config/)
//      → 枚举 server 的 loaded 插件名与 costmap_navigation.json 一致。
//
// argv: 1=config_dir, 2=host.so, 3=smac.so, 4=savitzky.so, 5=pid.so,
//       6=spin.so, 7=back_up.so, 8=wait.so, 9=clear.so

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <ayanami/components/plugin_manager.hpp>
#include <ayanami/core/abstract_controller.hpp>
#include <ayanami/core/abstract_navigation.hpp>
#include <ayanami/core/abstract_planner.hpp>
#include <ayanami/core/abstract_recovery.hpp>
#include <ayanami/core/abstract_smoother.hpp>
#include <ayanami/modules/costmap_navigation/costmap_navigation.hpp>
#include <ayanami/utility/config.hpp>
#include <ayanami/utility/load_module.hpp>

namespace {

using ayanami::abstract::AbstractController;
using ayanami::abstract::AbstractNavigation;
using ayanami::abstract::AbstractPlanner;
using ayanami::abstract::AbstractRecovery;
using ayanami::abstract::AbstractSmoother;
using ayanami::components::PluginManager;
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

// costmap_navigation.json 的 *_id 字段集合 (spec §9.3)。
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

// ---- 纯配置层: id ↔ name 反向解析 ----

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

TEST(ConfigConsistencyTest, IdsReverseResolveToPluginNames) {
  ayanami::GlobalConfig::instance(g_config_dir, /*override_path=*/true);
  const NavIds ids = readNavIds();

  // planner/controller/smoother: name == id (无后缀)。
  EXPECT_EQ(pluginName("config_planner", "planner"), ids.planner_id);
  EXPECT_EQ(pluginName("config_controller", "controller"), ids.controller_id);
  EXPECT_EQ(pluginName("config_smoother", "smoother"), ids.smoother_id);

  // recovery: name == id + "_recovery" (spec §6.6 命名统一)。
  EXPECT_EQ(pluginName("config_spin_recovery", "recovery"),
            ids.spin_id + "_recovery");
  EXPECT_EQ(pluginName("config_back_up_recovery", "recovery"),
            ids.backup_id + "_recovery");
  EXPECT_EQ(pluginName("config_wait_recovery", "recovery"),
            ids.wait_id + "_recovery");
  // clear_costmap_id 与 recovery_id 应指向同一个插件。
  EXPECT_EQ(ids.recovery_id, ids.clear_costmap_id);
  EXPECT_EQ(pluginName("config_recovery", "recovery"),
            ids.clear_costmap_id + "_recovery");
  EXPECT_EQ(pluginName("config_clear_costmap_recovery", "recovery"),
            ids.recovery_id + "_recovery");

  // 反向解析: 每个 id 都能在对应 plugins 列表里查到 name (避免历史 bug)。
  EXPECT_EQ(ids.spin_id + "_recovery", "costmap_spin_recovery");
  EXPECT_EQ(ids.backup_id + "_recovery", "costmap_back_up_recovery");
  EXPECT_EQ(ids.wait_id + "_recovery", "costmap_wait_recovery");
  EXPECT_EQ(ids.clear_costmap_id + "_recovery",
            "costmap_clear_costmap_recovery");
}

// ---- 真实 dlopen: PluginManager 装载名与 costmap_navigation.json 一致 ----

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
    // 4 个 recovery 全装载, 名必须 = id + "_recovery"。
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
    EXPECT_TRUE(mgr.hasPlugin(ids.spin_id + "_recovery"));
    EXPECT_TRUE(mgr.hasPlugin(ids.backup_id + "_recovery"));
    EXPECT_TRUE(mgr.hasPlugin(ids.wait_id + "_recovery"));
    EXPECT_TRUE(mgr.hasPlugin(ids.clear_costmap_id + "_recovery"));
    EXPECT_EQ(mgr.getLoadedNames().size(), 4u);
  }
}

// ---- 宿主初始化: dlopen host → initialize(config/) → 装载名与 json 一致 ----

TEST(ConfigConsistencyTest, HostInitializeLoadsConsistentPlugins) {
  void* handle = ayanami::open_so(g_host_so);
  ASSERT_NE(handle, nullptr);
  using Factory = AbstractNavigation* (*)();
  void* symbol = ayanami::load_symbol(handle, "create_navigation_module");
  ASSERT_NE(symbol, nullptr);
  auto factory = reinterpret_cast<Factory>(symbol);
  AbstractNavigation* nav = factory();
  ASSERT_NE(nav, nullptr);
  auto* host = dynamic_cast<CostmapNavigation*>(nav);
  ASSERT_NE(host, nullptr);

  host->initialize(g_config_dir);

  // loadPlugins 按 config_planner/.../config_recovery 装载 → createActions。
  // 注意: 真实 pid 控制器是零地图 AbstractController (Task 23 设计),
  // 无法注入 costmap 族 host, 其 id↔name 一致性在 PluginManager<AbstractController>
  // 层单独验证 (PluginManagersLoadMatchingNames)。这里只断言 host 能装载的
  // costmap 族插件。
  ASSERT_NE(host->navigationServer().plannerAction(), nullptr);
  ASSERT_NE(host->navigationServer().smootherAction(), nullptr);

  // recovery: config_recovery 指向的 clear_costmap 插件 action 已建
  // (装载名 = clear_costmap_id + "_recovery", spec §6.6)。
  const NavIds ids = readNavIds();
  ASSERT_NE(host->navigationServer().recoveryAction(
                ids.clear_costmap_id + "_recovery"),
            nullptr);

  // 枚举恢复 action 名, 必须含 clear_costmap 对应插件名。
  bool found = false;
  for (const auto& name : host->navigationServer().recoveryActionNames()) {
    if (name == ids.clear_costmap_id + "_recovery") {
      found = true;
    }
  }
  EXPECT_TRUE(found);

  host->stop();
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
