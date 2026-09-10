#include <gtest/gtest.h>

#include <ayanami/utility/load_module.hpp>
#include <ayanami/utility/extension_module.hpp>

#include <algorithm>
#include <string>

namespace {

// Read by main from argv (ctest passes the $<TARGET_FILE:...> path)
std::string g_dummy_planner_so;
std::string g_dummy_extension_so;

} // namespace

TEST(LoadModuleTest, openAndCallSymbol) {
  auto factory = ayanami::loadModuleFromSo<int(*)()>(
      g_dummy_planner_so, "createDummyModule");
  ASSERT_NE(factory, nullptr);
  EXPECT_EQ(factory(), 42);
}

TEST(LoadModuleTest, missingReturnsNull) {
  auto x = ayanami::loadModuleFromSo<int(*)()>("libnonexistent_xyz.so",
                                                  "create_xxx");
  EXPECT_EQ(x, nullptr);
}

TEST(ExtensionModuleTest, loadViaCreateExtensionModule) {
  auto ext = ayanami::ExtensionModule::loadModule(g_dummy_extension_so);
  ASSERT_NE(ext, nullptr);
}

// ---- Unload preparation: module registry (dedup + refcount + observation) ----

TEST(ModuleRegistryTest, repeatedOpenDedupesAndCountsRefs) {
  const auto before = ayanami::moduleHandleRefs(g_dummy_planner_so);
  void* first = ayanami::openSo(g_dummy_planner_so);
  ASSERT_NE(first, nullptr);
  void* second = ayanami::openSo(g_dummy_planner_so);
  EXPECT_EQ(first, second);
  EXPECT_EQ(ayanami::moduleHandleRefs(g_dummy_planner_so), before + 2);
  EXPECT_TRUE(ayanami::moduleHandleRefs("libnonexistent_xyz.so") == 0);
}

TEST(ModuleRegistryTest, loadedModuleNamesContainsOpenedModules) {
  ASSERT_NE(ayanami::openSo(g_dummy_extension_so), nullptr);
  const auto names = ayanami::loadedModuleNames();
  const auto it = std::find(names.begin(), names.end(), g_dummy_extension_so);
  EXPECT_NE(it, names.end());
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
