// robot_state_dso_extension.cpp -- cross-DSO RobotStateController singleton verification module.
//
// Linked against libayanami (SHARED), loaded via dlopen + the common factory. Exports 3 plain functions:
// - robotStateDsoInstance: the instance address (uintptr_t) as resolved by this .so.
// - robotStateDsoReadVelocityX / robotStateDsoSetVelocity:
// Reads/writes the same singleton through this .so symbol resolution.
// The test asserts that the main binary (writer, standing in for the ayanami_ros odomCallback) and the dlopen module
// (reader, standing in for the core controller execution) resolve to the same instance and share state --
// proving the instance definition lives only once inside libayanami, not header-inlined per .so.

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
      RobotStateController::instance().robotFrame();
  twist.twist.linear.x() = linear_x;
  RobotStateController::instance().setOdometry(twist);
}