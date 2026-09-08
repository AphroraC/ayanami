#include <gtest/gtest.h>

#include <ayanami/abstract/components/plugin_manager.hpp>
#include <ayanami/abstract/core/abstract_planner.hpp>

#include <Eigen/Geometry>

#include <filesystem>
#include <fstream>
#include <string>

namespace {

// Read by main from argv (ctest passes $<TARGET_FILE:dummy_planner_module> etc.)
std::string g_dummy_planner_so;
std::string g_dummy_extension_so;

std::filesystem::path makeTempDir() {
  std::filesystem::path dir =
      std::filesystem::temp_directory_path() / "ayanami_plugin_manager_test";
  std::filesystem::remove_all(dir);
  std::filesystem::create_directories(dir);
  return dir;
}

} // namespace

TEST(PluginManagerTest, loadPluginInvokesInitFnAndStores) {
  ayanami::components::PluginManager<ayanami::abstract::AbstractPlanner> mgr;

  bool init_called = false;
  std::string init_name;
  std::shared_ptr<ayanami::abstract::AbstractPlanner> init_plugin;
  // Common factory create_extension_module -> ExtensionModule::Ptr -> typed cast.
  bool ok = mgr.loadPlugin(
      ayanami::components::PluginSpec{"dummy", g_dummy_planner_so},
      [&](const std::string& name,
          std::shared_ptr<ayanami::abstract::AbstractPlanner> plugin) {
        init_called = true;
        init_name = name;
        init_plugin = std::move(plugin);
        return init_plugin != nullptr;
      });
  ASSERT_TRUE(ok);
  EXPECT_TRUE(init_called);
  EXPECT_EQ(init_name, "dummy");
  ASSERT_NE(init_plugin, nullptr);

  EXPECT_TRUE(mgr.hasPlugin("dummy"));
  EXPECT_FALSE(mgr.hasPlugin("missing"));
  ASSERT_NE(mgr.getPlugin("dummy"), nullptr);
  EXPECT_EQ(mgr.getPlugin("missing"), nullptr);
  EXPECT_EQ(mgr.getLoadedNames().size(), 1u);
  EXPECT_EQ(mgr.getLoadedNames().at(0), "dummy");
  EXPECT_EQ(mgr.getSoName("dummy"), g_dummy_planner_so);

  mgr.clearPlugins();
  EXPECT_FALSE(mgr.hasPlugin("dummy"));
  EXPECT_EQ(mgr.getPlugin("dummy"), nullptr);
  EXPECT_TRUE(mgr.getLoadedNames().empty());
}

TEST(PluginManagerTest, initFnRejectDoesNotStore) {
  ayanami::components::PluginManager<ayanami::abstract::AbstractPlanner> mgr;
  bool ok = mgr.loadPlugin(
      ayanami::components::PluginSpec{"rejected", g_dummy_planner_so},
      [](const std::string&,
         std::shared_ptr<ayanami::abstract::AbstractPlanner>) { return false; });
  EXPECT_FALSE(ok);
  EXPECT_FALSE(mgr.hasPlugin("rejected"));
}

TEST(PluginManagerTest, nonDerivedModuleFailsTypeCheck) {
  // Load an extension that is not an AbstractPlanner (dummy_extension_module.so, only derives
  // ExtensionModule) -> the typed cast fails -> not registered.
  auto& g = g_dummy_planner_so;
  ayanami::components::PluginManager<ayanami::abstract::AbstractPlanner> mgr;
  bool ok = mgr.loadPlugin(
      ayanami::components::PluginSpec{"wrong_type", g_dummy_extension_so},
      nullptr);
  EXPECT_FALSE(ok);
  EXPECT_FALSE(mgr.hasPlugin("wrong_type"));
  (void)g;
}

TEST(PluginManagerTest, loadSingleFromConfig) {
  auto dir = makeTempDir();
  std::ofstream ofs(dir / "planner.json");
  ofs << "{ \"mod\": { \"name\": \"x\", \"so_name\": \"" << g_dummy_planner_so
      << "\" } }";
  ofs.close();

  ayanami::Config cfg((dir / "planner.json").string());
  ayanami::components::PluginManager<ayanami::abstract::AbstractPlanner> mgr;
  bool ok = mgr.loadSingleFromConfig(
      cfg, "mod", "default_name",
      [](const std::string&,
         std::shared_ptr<ayanami::abstract::AbstractPlanner> plugin) {
        return plugin != nullptr;
      });
  ASSERT_TRUE(ok);
  EXPECT_TRUE(mgr.hasPlugin("x"));
  EXPECT_EQ(mgr.getSoName("x"), g_dummy_planner_so);
}

TEST(PluginManagerTest, loadFailureReturnsFalse) {
  ayanami::components::PluginManager<ayanami::abstract::AbstractPlanner> mgr;
  bool ok = mgr.loadPlugin(
      ayanami::components::PluginSpec{"bad", "libnonexistent_xyz.so"},
      nullptr);
  EXPECT_FALSE(ok);
  EXPECT_FALSE(mgr.hasPlugin("bad"));
  EXPECT_TRUE(mgr.getLoadedNames().empty());
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  if (argc > 1) {
    g_dummy_planner_so = argv[1];
  }
  if (argc > 2) {
    g_dummy_extension_so = argv[2];
  }
  return RUN_ALL_TESTS();
}
