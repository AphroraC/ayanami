// RobotStateController cross-DSO singleton integration test.
//
// Background (same class of key discipline): if RobotStateController::instance were defined inline in the
// header, every .so would hold its own static local instance -- velocities written directly by the ayanami_ros odomCallback
// writer (libayanami_ros.so) would be invisible to the core controller execution reader
// (libayanami_costmap_navigation.so, via the ServerControllerAction factory).
// With the definition sunk into a libayanami .cpp, every DSO linking libayanami resolves to the same instance.
//
// This test uses a dlopen test module (robot_state_dso_extension.so, linked against libayanami) to play
// "another DSO": the main binary writes a velocity and the module reads the same value (shared mutable state); the instance
// addresses resolved by the main binary and the module are identical (a single instance).
//
// argv: 1=robot_state_dso_extension.so

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <ayanami/abstract/components/robot_state_controller.hpp>
#include <ayanami/interfaces/transform_store.hpp>
#include <ayanami/messages/geometry_msgs.hpp>
#include <ayanami/utility/extension_module.hpp>
#include <ayanami/utility/load_module.hpp>

#include <cstdint>
#include <string>

namespace {

using ayanami::components::RobotStateController;
using ayanami::geometry_msgs::TwistStamped;

std::string g_dso_so;

using InstanceFn = uintptr_t (*)();
using ReadFn = double (*)();
using WriteFn = void (*)(double);

}  // namespace

TEST(RobotStateControllerCrossDSO, coreAndModuleResolveSameInstance) {
  auto ext = ayanami::ExtensionModule::loadModule(g_dso_so);
  ASSERT_NE(ext, nullptr);
  const auto dso_instance = ayanami::loadModuleFromSo<InstanceFn>(
      g_dso_so, "robotStateDsoInstance");
  const auto dso_read = ayanami::loadModuleFromSo<ReadFn>(
      g_dso_so, "robotStateDsoReadVelocityX");
  const auto dsoWrite = ayanami::loadModuleFromSo<WriteFn>(
      g_dso_so, "robotStateDsoSetVelocity");
  ASSERT_NE(dso_instance, nullptr);
  ASSERT_NE(dso_read, nullptr);
  ASSERT_NE(dsoWrite, nullptr);

  // Same address: the instance resolved by the module == the instance resolved by the main binary.
  EXPECT_EQ(dso_instance(),
            reinterpret_cast<uintptr_t>(&RobotStateController::instance()));

  // The writer (this binary, standing in for the ayanami_ros odomCallback) writes directly -> the reader (the module DSO,
  // standing in for the core controller execution) reads the same value. Proves cross-DSO shared mutable state.
  RobotStateController& rsc = RobotStateController::instance();
  rsc.initialize("map", "odom", "base_link", 0.25);
  rsc.resetVelocity();

  TwistStamped twist;
  twist.header.frame_id = "base_link";
  twist.twist.linear.x() = 0.75;
  rsc.setVelocity(twist);

  EXPECT_TRUE(rsc.hasFreshVelocity());
  EXPECT_DOUBLE_EQ(dso_read(), 0.75);

  // Reverse: the module DSO writes -> the main binary (standing in for the core reader) reads the same value.
  dsoWrite(1.25);
  EXPECT_DOUBLE_EQ(rsc.getRobotVelocity().twist.linear.x(), 1.25);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  if (argc > 1) {
    g_dso_so = argv[1];
  }
  return RUN_ALL_TESTS();
}