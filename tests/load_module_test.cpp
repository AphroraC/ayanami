#include <gtest/gtest.h>

#include <ayanami/utility/load_module.hpp>
#include <ayanami/utility/extension_module.hpp>

#include <string>

namespace {

// 由 main() 从 argv 读取 (ctest 传 $<TARGET_FILE:...> 路径)
std::string g_dummy_planner_so;
std::string g_dummy_extension_so;

} // namespace

TEST(LoadModuleTest, OpenAndCallSymbol) {
  auto factory = ayanami::load_module_from_so<int(*)()>(
      g_dummy_planner_so, "create_dummy_module");
  ASSERT_NE(factory, nullptr);
  EXPECT_EQ(factory(), 42);
}

TEST(LoadModuleTest, MissingReturnsNull) {
  auto x = ayanami::load_module_from_so<int(*)()>("libnonexistent_xyz.so",
                                                  "create_xxx");
  EXPECT_EQ(x, nullptr);
}

TEST(ExtensionModuleTest, LoadViaCreateExtensionModule) {
  auto ext = ayanami::ExtensionModule::load_module(g_dummy_extension_so);
  ASSERT_NE(ext, nullptr);
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
