# 行为树层

ayanami 使用 vendored 的 **BehaviorTree.CPP v4**，并做了"常驻树 + v4 语义
适配"。核心（BehaviorTreePilot）不含任何默认导航 XML/策略——树与节点注册
全部下沉到 `navigation_behavior_tree` 扩展模块（`BehaviorTreeExtension`）。

## 节点清单

**Action（StatefulActionNode，统一继承 CRTP 基类
`NavigationActionNode<Derived, Goal, Result>`，样板收敛于基类：onStart
组装/提交、onRunning 轮询+陈旧校验+失败上下文、onHalted 取消；节点只剩
端口组装、request* 调用与成功分支）**

- `ComputePathToPose`：端口
  `goal(in)/planner_id(in)/path(out)/plan_version(out)`；成功后
  **plan_version +1** 并写黑板（smoother 分支 reset_token 的数据来源）。
  planner 结果不做 plan_version 陈旧拒绝（自己就是版本推进者，轮次新鲜度
  由 RateController reset_token 保证）。
- `SmoothPath`：端口 `input_path(in)/smoother_id(in)/path(out)`；成功后
  **plan_version 再 +1**。**内容签名跳过**：输入路径签名与上次平滑一致时
  直接 SUCCESS（相同输入必得相同输出，不请求、不推版本）——这是合并计数
  后防"自触发循环"的关键。
- `FollowPath`：端口 `path(in)/controller_id(in)/frequency(in)`；
  plan_version 变化时对运行中的 controller 做 in-place
  `requestPlanUpdate`；"已到达"的 SUCCESS 不因 pending 版本差异被拒。
- `Recovery`：端口
  `recovery(id)/target_yaw/backup_distance/backup_speed/wait_duration`
  （覆盖端口 NaN=未提供，绝不解释为 0）；cause = 黑板 failure_context，
  **消费后清零**（单槽黑板键，避免后续恢复读到陈旧原因）。

**Condition**

- `IsPathValid`（委托 runtime 的路径校验；默认非空即有效）
- `GoalUpdated`（首 tick 缓存并返回 FAILURE，goal 变化返回 SUCCESS）

**Selector（nav2 PlannerSelector/ControllerSelector 同构，无 ROS topic）**

- `PlannerSelector` / `ControllerSelector` / `SmootherSelector`（同一类
  `PluginSelectorNode` 三个注册名）：每 tick 把选中插件 id 写到输出端口
  `selected_id`；取值优先 `override_id` 输入端口绑定的黑板键（外部写入即
  热切换），否则 `default_id`。输出接 ComputePathToPose/FollowPath/
  SmoothPath 的 *_id 端口（goal 级插件选择）。

**Control / Decorator**

- `PipelineSequence`：nav2 语义。从 child_0 开始每 tick 重 tick；前序
  RUNNING 且已推进过则继续推进流水线；任一 FAILURE → halt 全部 + FAILURE。
- `RecoveryNode`：2 children（主任务/恢复），`number_of_retries`。
  child0 SUCCESS → 复位整体 SUCCESS；child0 FAILURE 且预算未尽 → halt
  child0 进 恢复；child1 SUCCESS → `++retry_count` + `resetChildren()` +
  RUNNING（回主任务重试）；预算耗尽 → 复位 + FAILURE。不可恢复失败
  （INVALID_INPUT/START_INVALID/GOAL_INVALID/TRANSFORM_UNAVAILABLE/
  CANCELED）短路恢复分支直接失败；RESOURCE_BUSY **可重试**（瞬时态）。
- `RateController`：`hz`（<=0 单发缓存）/`reset_token`。reset_token 变化时
  halt 在飞 child 并重置缓存。planner 分支 token={goal_revision}（新导航
  轮重规划），smoother 分支 token={plan_version}（规划落地即重平滑，由
  签名跳过过滤平滑自身的 bump）。**nav2 语义对齐**（hz>0）：在飞 RUNNING
  child 每个 BT tick 都被轮询，速率只限制"启动新一轮"；计时器仅在 child
  SUCCESS 时复位。
- `RoundRobin` / `RecoverySequence`：轮询/顺序恢复，均带 `reset_token`
  （{goal_revision}）。语义差异：RoundRobin 的游标**跨成功保持前进**
  （spin→wait→backup 轮换，nav2 默认树语义）；RecoverySequence 每轮从第
  一个恢复重试。**生产树恢复分支使用 RoundRobin**。

## 常驻树协议（关键！）

树只建一次、跨 goal 常驻。为此有以下必须遵守的协议：

### 1. latest-wins goal 邮箱
`beginNavigation` 只覆写 `pending_goal` 并唤醒 tick 线程；`applyPendingGoal`
在 **tick 线程内** 消费邮箱、推进 revision、重写黑板。绝不跨线程 haltTree。

### 2. reset_token 复位协议
跨 goal 的"新一轮"语义由黑板令牌驱动：planner 分支与 RoundRobin/
RecoverySequence 监听 `{goal_revision}`；smoother 分支监听
`{plan_version}`（由 ComputePathToPose 成功后写黑板）。令牌变化 → 内部
复位 + halt 在飞 child。跨 goal 不继承恢复游标。

### 2b. 导航完成后的空闲状态（nav2 navigator 语义）
树返回 SUCCESS/FAILURE 表示**本次导航尝试结束**：pilot 复位树后进入空闲
（不再 tick），等待下一个 `beginNavigation` 邮箱投递才恢复。空闲期仍消费
halt 请求与 kill_switch。

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

**建树期强制校验**：pilot 建树时调
`validateTreeNodeStateProtocol(factory)`——所有非 BT 内置的
Control/Decorator 节点必须实现 `NavigationTreeNodeState`，否则建树即抛
异常（第三方扩展节点漏实现不再静默泄漏状态）。

## 失败语义与黑板

- BT 节点**只依据结构化的 `ErrorCode` 做决策，绝不解析 message**。
- 任何 FAILURE 前调用 `writeFailureContext` 把
  `FailureContext{stage, code, plugin_id, message, session_id, plan_version}`
  写入黑板 `failure_context`——Recovery 节点的 `RecoveryGoal.cause` 由此
  获得"谁/为什么触发恢复"，**并随即清零该槽位**。
- stale 结果（session 过期）一律视为 INTERNAL_ERROR 类 FAILURE，不进入
  重试统计；plan_version 陈旧拒绝只适用于 FollowPath（"已到达"豁免）与
  Recovery。
- RESOURCE_BUSY 是瞬时拒绝（恢复互斥等），**可恢复**（交恢复分支重试），
  不再归为致命失败。

## 黑板键清单（blackboard_protocol.hpp）

| 键 | 类型 | 写者 |
|---|---|---|
| `goal` | PoseStamped | pilot（applyPendingGoal） |
| `raw_path` / `smoothed_path` | Path | ComputePathToPose / SmoothPath |
| `plan_version` | uint64 | pilot（applyPendingGoal 快照）+ ComputePathToPose 输出端口（每次成功规划 ++） |
| `planner_frequency / smoother_frequency / controller_frequency` | double | pilot（configure 期） |
| `clear_costmap_id / spin_id / wait_id / backup_id` | string | BT 扩展（config_recovery role 映射） |
| `session_id / goal_revision` | uint64 | pilot |
| `failure_context` | FailureContext | 失败节点写 / Recovery 节点消费清零 |
| `navigation_runtime` | NavigationRuntime::Ptr | pilot（注入） |

版本计数已**合并为单一单调 plan_version**（规划 +1、平滑 +1；原 path_id
已删除）。必需板项由 `validateRequiredBlackboardEntries` 在建树后校验。

## 与 nav2 的差异提示

- nav2（BT.CPP v3）依赖强 halt 语义；**迁移到 v4 后必须补"树生命终端重置"
  + 建树期接口校验**，否则单发缓存/游标跨生命残留。
- nav2 的 GoalUpdated 短路恢复、RoundRobin 轮换语义已保留；
  nav2 的 `ClearEntireCostmap` 服务调用在 ayanami 中变为
  `Recovery` 节点 → 命令端口（进程内，无 service）。
- nav2 selector 节点靠 topic 热切换；ayanami 以黑板键写入等价实现。

## 常驻树控制节点协议（v4 适配）

- `NavigationTreeNodeState`：导航控制/装饰节点的公共状态复位接口；
  `resetNavigationTreeState` walker 只经该接口调 `resetState()`，并对所有
  非 IDLE 节点做 v3 式强复位（树生命期终态时终态缓存节点必须回到 IDLE，
  否则下一轮不再执行）。**建树期由 validateTreeNodeStateProtocol 强制。**
- `RecoveryNode::budget_reset_token`：令牌（{goal_revision}）变化时重置
  恢复预算，**不**打断在飞恢复——goal 更新永不消耗恢复预算。
- 恢复角色映射（role→插件 id）属于 **BT 扩展**：
  `BehaviorTreeExtension::requiredBlackboardEntries/provideBlackboardEntries`
  声明并写入角色 id 板项；核心 pilot 与黑板协议不感知具体恢复行为，
  BT 只做编排。
- 会话级导航终态：`NavigationCallbacks::on_navigation_result`
  （SUCCESS/FAILURE/CANCELED + FailureContext，每树生命期恰一次）。
