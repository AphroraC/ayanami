#include <gtest/gtest.h>

#include <ayanami/components/plugin_manager.hpp>
#include <ayanami/components/robot_state_provider.hpp>
#include <ayanami/core/abstract_planner.hpp>

#include <Eigen/Geometry>

#include <filesystem>
#include <fstream>
#include <string>

namespace {

// 由 main() 从 argv 读取 (ctest 传 $<TARGET_FILE:dummy_planner_module>)
std::string g_dummy_planner_so;

std::filesystem::path make_temp_dir() {
  std::filesystem::path dir =
      std::filesystem::temp_directory_path() / "ayanami_plugin_manager_test";
  std::filesystem::remove_all(dir);
  std::filesystem::create_directories(dir);
  return dir;
}

} // namespace

TEST(PluginManagerTest, LoadPluginInvokesInitFnAndStores) {
  ayanami::components::PluginManager<ayanami::abstract::AbstractPlanner> mgr;

  bool init_called = false;
  std::string init_name;
  std::shared_ptr<ayanami::abstract::AbstractPlanner> init_plugin;
  // 工厂符号 create_dummy_planner_plugin 返回真实 AbstractPlanner* (Task 10 在
  // dummy_planner_module 里补充; Task 3 的 create_dummy_module 只是 int 冒烟符号)。
  bool ok = mgr.loadPlugin(
      ayanami::components::PluginSpec{"dummy", g_dummy_planner_so,
                                      "create_dummy_planner_plugin"},
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

TEST(PluginManagerTest, InitFnRejectDoesNotStore) {
  ayanami::components::PluginManager<ayanami::abstract::AbstractPlanner> mgr;
  bool ok = mgr.loadPlugin(
      ayanami::components::PluginSpec{"rejected", g_dummy_planner_so,
                                      "create_dummy_planner_plugin"},
      [](const std::string&,
         std::shared_ptr<ayanami::abstract::AbstractPlanner>) { return false; });
  EXPECT_FALSE(ok);
  EXPECT_FALSE(mgr.hasPlugin("rejected"));
}

TEST(PluginManagerTest, LoadSingleFromConfig) {
  auto dir = make_temp_dir();
  std::ofstream ofs(dir / "planner.json");
  ofs << "{ \"mod\": { \"name\": \"x\", \"so_name\": \"" << g_dummy_planner_so
      << "\", \"create_symbol\": \"create_dummy_planner_plugin\" } }";
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

TEST(PluginManagerTest, LoadFailureReturnsFalse) {
  ayanami::components::PluginManager<ayanami::abstract::AbstractPlanner> mgr;
  bool ok = mgr.loadPlugin(
      ayanami::components::PluginSpec{"bad", "libnonexistent_xyz.so",
                                      "create_dummy_planner_plugin"},
      nullptr);
  EXPECT_FALSE(ok);
  EXPECT_FALSE(mgr.hasPlugin("bad"));
  EXPECT_TRUE(mgr.getLoadedNames().empty());
}

TEST(RobotStateProviderTest, DefaultReturnsIdentity) {
  ayanami::components::RobotStateProvider rsp;
  auto pose = rsp.getRobotPose();
  EXPECT_TRUE(pose.toIsometry().isApprox(Eigen::Isometry3d::Identity(), 1e-9));
  auto velocity = rsp.getRobotVelocity();
  EXPECT_EQ(velocity.twist.linear.x(), 0.0);
  EXPECT_EQ(velocity.twist.angular.z(), 0.0);
}

TEST(RobotStateProviderTest, FramesRoundTrip) {
  ayanami::components::RobotStateProvider rsp;
  rsp.setFrames("map", "odom", "base_link");
  EXPECT_EQ(rsp.getGlobalFrame(), "map");
  EXPECT_EQ(rsp.getOdomFrame(), "odom");
  EXPECT_EQ(rsp.getRobotFrame(), "base_link");
}

TEST(RobotStateProviderTest, SetRobotStateRoundTrip) {
  ayanami::components::RobotStateProvider rsp;
  ayanami::geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose = Eigen::Isometry3d::Identity();
  pose.pose.translation() = Eigen::Vector3d(1.0, 2.0, 0.0);

  ayanami::geometry_msgs::TwistStamped velocity;
  velocity.header.frame_id = "base_link";
  velocity.twist.linear = Eigen::Vector3d(0.5, 0.0, 0.0);

  rsp.setRobotState(pose, velocity);

  auto out_pose = rsp.getRobotPose();
  EXPECT_EQ(out_pose.header.frame_id, "map");
  EXPECT_NEAR(out_pose.pose.translation().x(), 1.0, 1e-9);
  EXPECT_NEAR(out_pose.pose.translation().y(), 2.0, 1e-9);

  auto out_velocity = rsp.getRobotVelocity();
  EXPECT_EQ(out_velocity.header.frame_id, "base_link");
  EXPECT_NEAR(out_velocity.twist.linear.x(), 0.5, 1e-9);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  if (argc > 1) {
    g_dummy_planner_so = argv[1];
  }
  return RUN_ALL_TESTS();
}
