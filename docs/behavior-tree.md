# 行为树层

ayanami 使用 vendored 的 **BehaviorTree.CPP v4**，并做了"常驻树 + v4 语义
适配"。核心（BehaviorTreePilot）不含任何默认导航 XML/策略——树与节点注册
全部下沉到 `navigation_behavior_tree` 扩展模块（`BehaviorTreeExtension`）。

## 节点清单

**Action（StatefulActionNode，TaskHandle 直连主流）**
- `ComputePathToPose`：端口 `goal(in)/path(out)/path_id(out)`；session+plan_version
  双校验；path_id 单调递增（smoother 分支的 reset_token）。
- `SmoothPath`：端口 `input_path/path`；成功后 `incrementPlanVersion`。
- `FollowPath`：端口 `path/frequency`；plan_version 变化时对运行中的
  controller 做 in-place `requestPlanUpdate`；"已到达"的 SUCCESS 不因
  pending 版本差异被拒。
- `Recovery`：端口 `recovery(id)/target_yaw/backup_distance/backup_speed/
  wait_duration`（覆盖端口 NaN=未提供，绝不解释为 0）；cause = 黑板
  failure_context。

**Condition**
- `IsPathValid`（委托 runtime 的路径校验；默认非空即有效）
- `IsGoalReached` / `IsFarFromGoal`
- `IsRobotStuck`（连续 tick 判定，`reset_stuck_token` 随会话复位）
- `GoalUpdated`（首 tick 缓存并返回 FAILURE，goal 变化返回 SUCCESS）

**Control / Decorator**
- `PipelineSequence`：nav2 语义。从 child_0 开始每 tick 重 tick；前序
  RUNNING 且已推进过则继续推进流水线；任一 FAILURE → halt 全部 + FAILURE。
- `RecoveryNode`：2 children（主任务/恢复），`number_of_retries`。child0
  SUCCESS → 复位整体 SUCCESS；child0 FAILURE 且预算未尽 → halt child0 进
  恢复；child1 SUCCESS → `++retry_count` + `resetChildren()` + RUNNING
  （回主任务重试）；预算耗尽 → 复位 + FAILURE。
- `RateController`：`hz`（<=0 单发缓存）/`reset_token`。reset_token 变化时
  halt 在飞 child 并重置缓存（新 goal / 新 path 触发重新执行）。
- `RoundRobin` / `RecoverySequence`：轮询/顺序恢复，均带 `reset_token`
  （新导航轮游标复位、在飞 child 被 halt）。

## 常驻树协议（关键！）

树只建一次、跨 goal 常驻。为此有三条必须遵守的协议：

### 1. latest-wins goal 邮箱
`beginNavigation` 只覆写 `pending_goal` 并唤醒 tick 线程；`applyPendingGoal`
在 **tick 线程内** 消费邮箱、推进 revision、重写黑板。绝不跨线程 haltTree。

### 2. reset_token 复位协议
跨 goal 的"新一轮"语义由黑板令牌驱动：planner/smoother 分支与
RecoverySequence/RoundRobin 都监听 `{goal_revision}`（或 `{path_id}`），
令牌变化 → 内部复位 + halt 在飞 child。跨 goal 不继承恢复游标。

### 3. 树生命终端全量重置（BT.CPP v4 halt 适配，load-bearing）

BT.CPP v4 的 `ControlNode::haltChildren()` **只对 status==RUNNING 的 child
调用 haltNode()**（v3 是无条件全量）——SUCCESS/FAILURE 终态缓存的子树不会
被重置。常驻树模型下，RateController 的单发缓存、RecoveryNode 的重试计数、
RecoverySequence/RoundRobin 的游标会跨树生命残留，造成
"planner 永不重规划 + 恢复风暴"（实测 rviz 场景：28 次恢复、FollowPath
零重入、path 恒 1）。

修复：五个自定义控制/装饰节点实现 `resetState()`；pilot 在 `tickOnce`
返回非 RUNNING 时调用 `resetNavigationTreeState(tree)`
（`behavior_tree/navigation.hpp`）——等价于 nav2(v3) 的 haltAllActions。
任何**绕过 pilot 直接驱动树**的代码（含测试）必须复刻同一契约：
tick 返回终端状态后调用 `resetNavigationTreeState`。

## 失败语义与黑板

- BT 节点**只依据结构化的 `ErrorCode` 做决策，绝不解析 message**。
- 任何 FAILURE 前调用 `writeFailureContext` 把
  `FailureContext{stage, code, plugin_id, message, session_id, plan_version}`
  写入黑板 `failure_context`——Recovery 节点的 `RecoveryGoal.cause` 由此
  获得"谁/为什么触发恢复"。
- stale 结果（session 或 plan_version 过期）一律视为 INTERNAL_ERROR 类
  FAILURE，不进入重试统计。
- 特例：controller "已到达"（reached=true）的 SUCCESS 不因 pending 版本
  差异被拒（更新在途时到达是合法竞态）。

## 黑板键清单（blackboard_protocol.hpp）

| 键 | 类型 | 写者 |
|---|---|---|
| `goal` | PoseStamped | pilot（applyPendingGoal） |
| `raw_path` / `smoothed_path` | Path | ComputePathToPose / SmoothPath |
| `path_id` | uint64 | ComputePathToPose（每次成功规划 ++） |
| `planner_frequency / smoother_frequency / controller_frequency` | double | pilot（configure 期） |
| `clear_costmap_id / spin_id / wait_id / backup_id` | string | pilot（由 config_recovery 的 role 映射） |
| `session_id / plan_version / goal_revision / reset_stuck_token` | uint64 | pilot + 共享 NavigationSessionState |
| `failure_context` | FailureContext | 失败节点 |
| `navigation_runtime` | NavigationRuntime::Ptr | pilot（注入） |

必需板项由 `validateRequiredBlackboardEntries` 在建树后校验（17 项）。

## 与 nav2 的差异提示

- nav2（BT.CPP v3）依赖强 halt 语义；**迁移到 v4 后必须补"树生命终端重置"**，
  否则单发缓存/游标跨生命残留。
- nav2 的 GoalUpdated 短路恢复、RoundRobin 轮换语义已保留；
  nav2 的 `ClearEntireCostmap` 服务调用在 ayanami 中变为
  `Recovery` 节点 → 命令端口（进程内，无 service）。
