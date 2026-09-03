#pragma once

#include <optional>

namespace ayanami {

/**
 * @brief 恢复行为按次覆盖参数 (spec: RecoveryGoal 常用字段)。
 *
 * BT 恢复节点可按次覆盖常用行为参数 (如 <Recovery recovery="{spin_id}"
 * target_yaw="3.14"/>); 未提供 (nullopt) 的字段表示使用插件 configure 期
 * 默认值。字段为 nullopt 时必须保持配置默认, 禁止把 nullopt 解释为 0。
 */
struct RecoveryBehaviorParams {
  /// @brief 原地旋转目标角 (rad), 对应 spin 恢复的 spin_dist。
  std::optional<double> target_yaw;
  /// @brief 后退距离 (m), 对应 back_up 恢复的 backup_dist。
  std::optional<double> backup_distance;
  /// @brief 后退线速度 (m/s), 对应 back_up 恢复的 max_v。
  std::optional<double> backup_speed;
  /// @brief 等待时长 (s), 对应 wait 恢复的 wait_duration。
  std::optional<double> wait_duration;

  bool empty() const {
    return !target_yaw && !backup_distance && !backup_speed && !wait_duration;
  }
};

}  // namespace ayanami
