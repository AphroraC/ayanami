#pragma once

#include <cstdint>

namespace ayanami {

/**
 * @brief 全框架统一强类型失败码 (替换 planner/controller/smoother/recovery 的
 * 50/100/200/300 局部码段与 uint32 outcome)。
 *
 * 纪律:
 * - 四类插件接口 (AbstractPlanner/Smoother/Controller/Recovery) 一律返回本类型;
 * - 各 action Result 用统一字段 `failure`, None 表示成功 (对应
 *   GoalStatus::SUCCEEDED);
 * - GoalStatus 保留; message 仅作诊断, BT/宿主只依据 failure 决策, 不解析 message。
 */
enum class FailureCode : uint8_t {
  None = 0,             ///< 成功 (无失败)。
  Canceled,             ///< 被取消。
  Timeout,              ///< 耐心/预算超时 (PAT_EXCEEDED)。
  InvalidInput,         ///< 输入非法 (空路径等)。
  TransformUnavailable, ///< 坐标变换不可用 / frame 不匹配。
  MapUnavailable,       ///< 地图未就绪。
  MapStale,             ///< 地图过期 (isCurrent 不满足)。
  StartInvalid,         ///< 起点非法 (越界 / 被阻挡)。
  GoalInvalid,          ///< 目标非法 (越界 / 被阻挡 / 无可行目标)。
  PathNotFound,         ///< 未找到路径。
  PathInvalid,          ///< 路径非法 (无法跟踪)。
  SmoothingFailed,      ///< 平滑失败。
  NoValidCommand,       ///< 控制器无有效速度命令。
  NoProgress,           ///< 无进展 (卡住)。
  GoalNotReached,       ///< 未到达目标。
  RecoveryFailed,       ///< 恢复行为失败。
  InternalError,        ///< 内部错误 / 插件未初始化。
};

}  // namespace ayanami