#pragma once

#include <cstdint>
#include <string>

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

/**
 * @brief 失败发生的处理阶段 (BT action 节点写 FailureContext 用)。
 */
enum class FailureStage : uint8_t {
  Planner = 0,
  Smoother = 1,
  Controller = 2,
  Recovery = 3,
  Navigation = 4,
  Unknown = 5,
};

/**
 * @brief 结构化失败上下文 (spec §6.2 关键纪律扩展)。
 *
 * 每个 BT action 节点 (ComputePathToPose / SmoothPath / FollowPath / Recovery)
 * 在返回 FAILURE 前把精确的失败上下文写入黑板 `failure_context`
 * (blackboard_protocol.hpp)。RecoveryGoal.cause 携带该上下文, 使恢复行为可识别
 * 触发它的失败 (插件 id / 失败码 / 所属会话版本)。
 *
 * 黑板键值全量替换 (非增量 merge): 节点的 latest 失败覆盖前次失败。
 */
struct FailureContext {
  FailureStage stage = FailureStage::Unknown;
  FailureCode code = FailureCode::None;
  std::string plugin_id;   ///< 触发失败的插件逻辑名
  std::string message;     ///< 诊断消息 (仅供日志, BT/宿主不解析)
  uint64_t session_id = 0;  ///< 失败所属会话
  uint64_t plan_version = 0;  ///< 失败所属规划版本
};

}  // namespace ayanami