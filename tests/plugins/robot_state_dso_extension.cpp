// robot_state_dso_extension.cpp — 跨 DSO RobotStateController 单例验证插件。
//
// 链接 libayanami (SHARED), 经 dlopen + 公共工厂装载。导出 3 个纯函数:
// - robotStateDsoInstance: 经本 .so 解析到的 instance 地址 (uintptr_t)。
// - robotStateDsoReadVelocityX / robotStateDsoSetVelocity:
// 经本 .so 的符号解析读写同一单例。
// 测试断言主二进制 (writer, 对标 ayanami_ros odomCallback) 与 dlopen 的插件
// (reader, 对标核心 controller execution) 解析到同一 instance 且共享状态 ——
// 证明 instance 定义只在 libayanami 内一份, 而非每个 .so header-inline 一份。

#include <ayanami/abstract/components/robot_state_controller.hpp>
#include <ayanami/utility/extension_module.hpp>

#include <cstdint>

namespace {

using ayanami::components::RobotStateController;
using ayanami::geometry_msgs::TwistStamped;

}  // namespace

extern "C" ayanami::ExtensionModule* create_extension_module() {
  return new ayanami::ExtensionModule();
}

extern "C" uintptr_t robotStateDsoInstance() {
  return reinterpret_cast<uintptr_t>(&RobotStateController::instance());
}

extern "C" double robotStateDsoReadVelocityX() {
  return RobotStateController::instance().getRobotVelocity().twist.linear.x();
}

extern "C" void robotStateDsoSetVelocity(double linear_x) {
  TwistStamped twist;
  twist.header.frame_id =
      RobotStateController::instance().getRobotFrame();
  twist.twist.linear.x() = linear_x;
  RobotStateController::instance().setVelocity(twist);
}