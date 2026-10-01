#pragma once

#include <array>

#include "ayanami/messages/geometry_msgs.hpp"

namespace ayanami {
namespace messages {

/// @brief One swerve wheel group command (IROS2024 ControlSpace8D): steer
/// angle (rad, body frame) and rotor speed (rad/s).
struct SwerveWheelCommand {
  double steer{0.0};
  double rotor{0.0};
};

/// @brief All four wheel-group commands of a swerve chassis, ordered
/// fl, fr, rl, rr. Derived telemetry from a body twist via the rigid-body
/// inverse -- the twist channel stays authoritative; this carries the
/// wheel-level view for a future direct-chassis bridge and diagnostics.
struct SwerveCommandsStamped {
  geometry_msgs::Header header;
  std::array<SwerveWheelCommand, 4> wheels{};
};

}  // namespace messages
}  // namespace ayanami
