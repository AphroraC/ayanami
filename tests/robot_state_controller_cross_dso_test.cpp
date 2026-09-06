// RobotStateController 跨 DSO 单例集成测试。
//
// 背景 (关键纪律同类): RobotStateController::instance 若在 header
// 内 inline 定义, 每个 .so 各持一份静态局部实例 —— ayanami_ros odomCallback
// 写者 (libayanami_ros.so) 直写的速度, 核心 controller execution 读者
// (libayanami_costmap_navigation.so, 经 ServerControllerAction factory) 读不到。
// 定义下沉到 libayanami 的 .cpp 后, 所有链接 libayanami 的 DSO 解析到同一实例。
//
// 本测试用 dlopen 的测试插件 (robot_state_dso_extension.so, 链接 libayanami)
// 模拟"另一个 DSO": 主二进制写速度 → 插件读到同一值 (共享可变状态); 主二进制
// 与插件解析到的 instance 地址一致 (单个实例)。
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

TEST(RobotStateControllerCrossDSO, coreAndPluginResolveSameInstance) {
  auto ext = ayanami::ExtensionModule::load_module(g_dso_so);
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

  // 同一地址: 插件解析到的 instance == 主二进制解析到的 instance。
  EXPECT_EQ(dso_instance(),
            reinterpret_cast<uintptr_t>(&RobotStateController::instance()));

  // 写者 (本二进制, 对标 ayanami_ros odomCallback) 直写 → 读者 (插件 DSO,
  // 对标核心 controller execution) 读到同一值。证明跨 DSO 共享可变状态。
  RobotStateController& rsc = RobotStateController::instance();
  rsc.initialize("map", "odom", "base_link", 0.25);
  rsc.resetVelocity();

  TwistStamped twist;
  twist.header.frame_id = "base_link";
  twist.twist.linear.x() = 0.75;
  rsc.setVelocity(twist);

  EXPECT_TRUE(rsc.hasFreshVelocity());
  EXPECT_DOUBLE_EQ(dso_read(), 0.75);

  // 反向: 插件 DSO 写入 → 主二进制 (对标核心读者) 读到同一值。
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