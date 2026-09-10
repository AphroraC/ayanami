#pragma once

#include <optional>

namespace ayanami {

/**
 * @brief Recovery behavior per-run override params (spec: common
 * RecoveryGoal fields).
 *
 * BT recovery nodes may override common behavior params per run (e.g.
 * <Recovery recovery="{spin_id}" target_yaw="3.14"/>); a field left unset
 * (nullopt) means use the module's configure-time defaults. A nullopt field
 * must keep the configured default; interpreting nullopt as 0 is
 * forbidden.
 */
struct RecoveryBehaviorParams {
  /// @brief In-place rotation target angle (rad), corresponding to spin recovery's spin_dist.
  std::optional<double> target_yaw;
  /// @brief Backup distance (m), corresponding to back_up recovery's backup_dist.
  std::optional<double> backup_distance;
  /// @brief Backup linear speed (m/s), corresponding to back_up recovery's max_v.
  std::optional<double> backup_speed;
  /// @brief Wait duration (s), corresponding to wait recovery's wait_duration.
  std::optional<double> wait_duration;

  bool empty() const {
    return !target_yaw && !backup_distance && !backup_speed && !wait_duration;
  }
};

}  // namespace ayanami
