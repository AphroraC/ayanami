#include <gtest/gtest.h>

#include <ayanami/abstract/components/module_manager.hpp>
#include <ayanami/abstract/core/abstract_planner.hpp>
#include <ayanami/utility/config.hpp>

#include <Eigen/Geometry>

#include <filesystem>
#include <fstream>
#include <string>

namespace {

// Read by main from argv (ctest passes $<TARGET_FILE:dummy_planner_module> etc.)
std::string g_dummy_planner_so;
std::string g_dummy_extension_so;
/// argv[3]: a planner .so that reports a stale interface version.
std::string g_stale_api_planner_so;

std::filesystem::path makeTempDir() {
  std::filesystem::path dir =
      std::filesystem::temp_directory_path() / "ayanami_module_manager_test";
  std::filesystem::remove_all(dir);
  std::filesystem::create_directories(dir);
  return dir;
}

} // namespace

TEST(ModuleManagerTest, loadModuleInvokesInitFnAndStores) {
  ayanami::components::ModuleManager<ayanami::abstract::AbstractPlanner> mgr;

  bool init_called = false;
  std::string init_name;
  std::shared_ptr<ayanami::abstract::AbstractPlanner> init_module;
  // Common factory create_extension_module -> ExtensionModule::Ptr -> typed cast.
  bool ok = mgr.loadModule(
      ayanami::components::ModuleSpec{"dummy", g_dummy_planner_so},
      // No config file: the dummy module reads no parameters.
      ayanami::ModuleContext{"dummy", ayanami::Config("")},
      [&](const std::string& name,
          std::shared_ptr<ayanami::abstract::AbstractPlanner> module) {
        init_called = true;
        init_name = name;
        init_module = std::move(module);
        return init_module != nullptr;
      });
  ASSERT_TRUE(ok);
  EXPECT_TRUE(init_called);
  EXPECT_EQ(init_name, "dummy");
  ASSERT_NE(init_module, nullptr);

  EXPECT_TRUE(mgr.hasModule("dummy"));
  EXPECT_FALSE(mgr.hasModule("missing"));
  ASSERT_NE(mgr.getModule("dummy"), nullptr);
  EXPECT_EQ(mgr.getModule("missing"), nullptr);
  EXPECT_EQ(mgr.getLoadedNames().size(), 1u);
  EXPECT_EQ(mgr.getLoadedNames().at(0), "dummy");
  EXPECT_EQ(mgr.getSoName("dummy"), g_dummy_planner_so);

  mgr.clearModules();
  EXPECT_FALSE(mgr.hasModule("dummy"));
  EXPECT_EQ(mgr.getModule("dummy"), nullptr);
  EXPECT_TRUE(mgr.getLoadedNames().empty());
}

TEST(ModuleManagerTest, initFnRejectDoesNotStore) {
  ayanami::components::ModuleManager<ayanami::abstract::AbstractPlanner> mgr;
  bool ok = mgr.loadModule(
      ayanami::components::ModuleSpec{"rejected", g_dummy_planner_so},
      ayanami::ModuleContext{"rejected", ayanami::Config("")},
      [](const std::string&,
         std::shared_ptr<ayanami::abstract::AbstractPlanner>) { return false; });
  EXPECT_FALSE(ok);
  EXPECT_FALSE(mgr.hasModule("rejected"));
}

TEST(ModuleManagerTest, nonDerivedModuleFailsTypeCheck) {
  // Load an extension that is not an AbstractPlanner (dummy_extension_module.so, only derives
  // ExtensionModule) -> the typed cast fails -> not registered.
  auto& g = g_dummy_planner_so;
  ayanami::components::ModuleManager<ayanami::abstract::AbstractPlanner> mgr;
  bool ok = mgr.loadModule(
      ayanami::components::ModuleSpec{"wrong_type", g_dummy_extension_so},
      ayanami::ModuleContext{"wrong_type", ayanami::Config("")}, nullptr);
  EXPECT_FALSE(ok);
  EXPECT_FALSE(mgr.hasModule("wrong_type"));
  (void)g;
}

TEST(ModuleManagerTest, loadSingleFromConfig) {
  auto dir = makeTempDir();
  std::ofstream ofs(dir / "planner.json");
  // "x": {} declares the module's (empty) parameter section, which the loader
  // requires so a name/section mismatch cannot hide behind "no parameters".
  ofs << "{ \"mod\": { \"name\": \"x\", \"so_name\": \"" << g_dummy_planner_so
      << "\" }, \"x\": {} }";
  ofs.close();

  ayanami::Config cfg((dir / "planner.json").string());
  ayanami::components::ModuleManager<ayanami::abstract::AbstractPlanner> mgr;
  bool ok = mgr.loadSingleFromConfig(
      cfg, "mod", "default_name",
      [](const std::string&,
         std::shared_ptr<ayanami::abstract::AbstractPlanner> module) {
        return module != nullptr;
      });
  ASSERT_TRUE(ok);
  EXPECT_TRUE(mgr.hasModule("x"));
  EXPECT_EQ(mgr.getSoName("x"), g_dummy_planner_so);
}

// ---- Interface-version guard ----
// RTTI only proves the class names line up. A .so built against an older family
// header passes the cast and would then be driven through a virtual surface it
// does not have -- silent UB. The version reported by the module must match the
// one this build compiled in.
TEST(ModuleManagerTest, staleApiVersionIsRefusedWithItsOwnReason) {
  ASSERT_FALSE(g_stale_api_planner_so.empty());
  ayanami::components::ModuleManager<ayanami::abstract::AbstractPlanner> mgr;
  const bool ok = mgr.loadModule(
      ayanami::components::ModuleSpec{"stale", g_stale_api_planner_so},
      ayanami::ModuleContext{"stale", ayanami::Config("")}, nullptr);

  EXPECT_FALSE(ok);
  EXPECT_FALSE(mgr.hasModule("stale"));
  // Not a type mismatch: it IS a planner, just not this build's planner.
  EXPECT_EQ(mgr.lastFailure(),
            ayanami::components::ModuleManager<
                ayanami::abstract::AbstractPlanner>::LoadFailure::
                ApiVersionMismatch);
  EXPECT_NE(mgr.lastFailureMessage().find("999"), std::string::npos)
      << mgr.lastFailureMessage();
}

// ---- Unused-parameter audit ----
// The audit's whole point is to make a misspelled key distinguishable from an
// intentional omission: with a silent default, both look the same.

TEST(ModuleManagerTest, auditReportsDeclaredButUnreadKeys) {
  const auto dir = makeTempDir() / "audit";
  std::filesystem::create_directories(dir);
  {
    std::ofstream ofs(dir / "planner.json");
    ofs << "{ \"planner\": { \"name\": \"audited\", \"so_name\": \"\" },\n"
           "  \"audited\": { \"read_key\": 2.0, \"typoed_key\": 1.0 } }";
  }
  ayanami::Config cfg((dir / "planner.json").string());

  // Nothing read yet: both keys are unused.
  EXPECT_EQ(cfg.unusedKeys("audited").size(), 2u);

  // Consume one of them the way a module would.
  EXPECT_DOUBLE_EQ(cfg.param<double>("audited", "read_key", 0.0), 2.0);
  EXPECT_TRUE(cfg.wasConsumed("audited", "read_key"));
  EXPECT_FALSE(cfg.wasConsumed("audited", "typoed_key"));

  const auto unused = ayanami::auditUnusedParameters(cfg, "audited", "test");
  ASSERT_EQ(unused.size(), 1u);
  EXPECT_EQ(unused.front(), "typoed_key");
}

TEST(ModuleManagerTest, strictParamsTurnsTheAuditIntoAFailure) {
  const auto dir = makeTempDir() / "audit_strict";
  std::filesystem::create_directories(dir);
  {
    std::ofstream ofs(dir / "planner.json");
    ofs << "{ \"global\": { \"strict_params\": true },\n"
           "  \"planner\": { \"name\": \"audited\", \"so_name\": \"\" },\n"
           "  \"audited\": { \"typoed_key\": 1.0 } }";
  }
  ayanami::Config cfg((dir / "planner.json").string());
  // Opt-in strictness: a deployment adopts it once its configs are clean.
  EXPECT_THROW(ayanami::auditUnusedParameters(cfg, "audited", "test"),
               std::runtime_error);
}

// The loader must actually run the audit, otherwise the mechanism is dead code.
TEST(ModuleManagerTest, loaderAuditsTheModuleItJustConfigured) {
  const auto dir = makeTempDir() / "audit_loader";
  std::filesystem::create_directories(dir);
  {
    std::ofstream ofs(dir / "planner.json");
    ofs << "{ \"global\": { \"strict_params\": true },\n"
           "  \"planner\": { \"name\": \"audited\", \"so_name\": \""
        << g_dummy_planner_so
        << "\" },\n"
           "  \"audited\": { \"typoed_key\": 1.0 } }";
  }
  ayanami::components::ModuleManager<ayanami::abstract::AbstractPlanner> mgr;
  // The dummy module reads no parameters, so its section is entirely unused and
  // strict mode aborts the load.
  EXPECT_THROW(
      mgr.loadSingleFromConfig(
          ayanami::Config((dir / "planner.json").string()), "planner",
          "audited", nullptr),
      std::runtime_error);
  EXPECT_FALSE(mgr.hasModule("audited"));
}

// ---- Module identity invariant ----
// A module's name is what the parameter section, the family json and the
// loader all key off. When a json declares a name whose section does not
// exist, the module reads nothing and runs on its built-in defaults while
// looking perfectly healthy -- and the unused-key audit cannot see it, because
// it audits the section the module asked for (which is simply absent).
TEST(ModuleManagerTest, nameWithoutMatchingSectionIsRefused) {
  const auto dir = makeTempDir() / "identity";
  std::filesystem::create_directories(dir);
  {
    std::ofstream ofs(dir / "planner.json");
    // Params live under "planner_params", but the module is named "renamed".
    ofs << "{ \"planner\": { \"name\": \"renamed\", \"so_name\": \""
        << g_dummy_planner_so
        << "\" }, \"planner_params\": { \"goal_tolerance\": 1.0 } }";
  }
  ayanami::components::ModuleManager<ayanami::abstract::AbstractPlanner> mgr;
  EXPECT_FALSE(mgr.loadSingleFromConfig(
      ayanami::Config((dir / "planner.json").string()), "planner", "renamed",
      nullptr));
  EXPECT_FALSE(mgr.hasModule("renamed"));
  EXPECT_EQ(mgr.lastFailure(),
            ayanami::components::ModuleManager<
                ayanami::abstract::AbstractPlanner>::LoadFailure::
                IdentityMismatch);
  // The report names the module and the section it expected, so the fix is
  // obvious from the log line alone.
  EXPECT_NE(mgr.lastFailureMessage().find("renamed"), std::string::npos)
      << mgr.lastFailureMessage();
  EXPECT_NE(mgr.lastFailureMessage().find("no section named"), std::string::npos)
      << mgr.lastFailureMessage();
}

TEST(ModuleManagerTest, loadFailureReturnsFalse) {
  ayanami::components::ModuleManager<ayanami::abstract::AbstractPlanner> mgr;
  bool ok = mgr.loadModule(
      ayanami::components::ModuleSpec{"bad", "libnonexistent_xyz.so"},
      ayanami::ModuleContext{"bad", ayanami::Config("")}, nullptr);
  EXPECT_FALSE(ok);
  EXPECT_FALSE(mgr.hasModule("bad"));
  EXPECT_TRUE(mgr.getLoadedNames().empty());
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  if (argc > 1) {
    g_dummy_planner_so = argv[1];
  }
  if (argc > 3) g_stale_api_planner_so = argv[3];
  if (argc > 2) {
    g_dummy_extension_so = argv[2];
  }
  return RUN_ALL_TESTS();
}
