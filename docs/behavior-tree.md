# 行为树层

ayanami 使用 vendored 的 **BehaviorTree.CPP v4**，采用"**工厂常驻、树实例按
尝试新建**"的模型（见下节）。核心（BehaviorTreePilot）不含任何默认导航
XML/策略——树与节点注册全部下沉到 `navigation_behavior_tree` 扩展模块
（`BehaviorTreeExtension`）。

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
  `ModuleSelectorNode` 三个注册名）：每 tick 把选中模块 id 写到输出端口
  `selected_id`；取值优先 `override_id` 输入端口绑定的黑板键（外部写入即
  热切换），否则 `default_id`。输出接 ComputePathToPose/FollowPath/
  SmoothPath 的 *_id 端口（goal 级模块选择）。

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
- `RecoverySequence`：**轮换恢复**（nav2 RoundRobin 语义，生产树恢复分支
  唯一实现），带 `reset_token`（{goal_revision}）。游标**跨成功保持前进**
  （spin→wait→backup 轮换，连续恢复轮换手段而不是反复执行第一个）；当前
  child RUNNING → 整体 RUNNING；child SUCCESS → 游标前进并返回 SUCCESS；
  全部失败 → 整体 FAILURE。

## 树实例模型（关键！）

**工厂建一次，树实例每次导航尝试建一次。**

- `loadFromFile` 只建**工厂**：加载 BT 扩展、注册节点与树 XML，解析出要实例化的
  tree id。树 XML 是**必需**的配置资产（`behavior_tree_xml`），没有内嵌默认树，
  缺失即启动报错——否则栈会跑一棵磁盘上任何文件都没描述的策略。
- 每次**新的导航尝试**（`applyPendingGoal` 发现当前没有活实例时）调用
  `createTreeInstance()`：全新的黑板 + 全新的树对象。
- 尝试到达终态（SUCCESS/FAILURE）时 `releaseTreeInstance()`：**树对象释放，
  黑板保留**作为这次尝试的记录（路径/会话/版本/失败上下文仍可观测，但没有任何
  节点状态存活）。

树生命期 == 树实例，所以跨生命期状态泄漏按构造不可能：RateController 的单发缓存、
RecoveryNode 的重试预算、RecoverySequence 的游标都没有"下一次"可泄漏到。**原先为
模拟这一点而自建的三样设施已整体删除**：`NavigationTreeNodeState` 接口、
`resetNavigationTreeState()` walker、`validateTreeNodeStateProtocol()` 建树期
抛异常校验。第三方 control/decorator 节点不再需要实现任何东西。

（历史背景：BT.CPP v4 的 `haltChildren()` 只对 RUNNING 子节点生效，常驻树模型下
终态缓存会跨 goal 残留，实测造成"28 次恢复、FollowPath 零重入、path 恒 1"。目标级
重建从根上消除了这类问题，而不是继续打补丁。）

### 1. latest-wins goal 邮箱
`beginNavigation` 只覆写 `pending_goal` 并唤醒 tick 线程；`applyPendingGoal`
在 **tick 线程内** 消费邮箱、推进 revision、重写黑板。绝不跨线程 haltTree。

### 2. 两次尝试之间没有状态，尝试之内靠 reset_token
跨尝试不需要任何复位。**同一次尝试内的目标微调**（goal revision）才需要：
planner 分支与 RecoverySequence 监听 `{goal_revision}`，
smoother 分支监听 `{raw_path_revision}`（由 ComputePathToPose 成功后写黑板）。
令牌变化 → 内部复位 + halt 在飞 child。

### 2b. 空闲状态（nav2 navigator 语义）
树返回 SUCCESS/FAILURE 表示**本次导航尝试结束**：pilot 释放实例后进入空闲
（不再 tick、不触碰树对象），在邮箱条件变量上等待下一次 `beginNavigation`。
空闲期仍消费 halt 请求与 kill_switch。

### 3. 建树期端口与角色校验（load-bearing）
实例建成后立即遍历树，解析每个模块引用：

- `Recovery` 节点的 `recovery` 端口必须解析为**非空**且已加载的模块 id；
- `ComputePathToPose`/`SmoothPath`/`FollowPath` 的**非空** `*_id` 端口必须指向
  已加载模块（空 = 该家族的默认，合法）。

这条校验针对的具体故障：某个 role 没有任何已加载模块声明时，黑板键是空字符串，
`Recovery` 节点提交空 id——错误只在**恢复分支真的被派发时**才出现，也就是机器人
已经卡住的那一刻。现在它在建树期就被拒绝。

建树/校验抛出的异常由 pilot 捕获（tick 线程上的未捕获异常会直接 terminate 进程）：
记录 critical 日志、发出本次尝试的 `on_navigation_result(ABORTED)`、释放实例并停止
tick。配置错误不会靠重试自愈，所以不继续循环。

## 失败语义与黑板

- BT 节点**只依据结构化的 `ErrorCode` 做决策，绝不解析 message**。
- 任何 FAILURE 前调用 `writeFailureContext` 把
  `FailureContext{stage, code, module_id, message, session_id, plan_version}`
  写入黑板 `failure_context`——Recovery 节点的 `RecoveryGoal.cause` 由此
  获得"谁/为什么触发恢复"，**并随即清零该槽位**。
- stale 结果（session 过期）一律视为 INTERNAL_ERROR 类 FAILURE，不进入
  重试统计；plan_version 陈旧拒绝只适用于 FollowPath（"已到达"豁免）与
  Recovery。
- RESOURCE_BUSY 是瞬时拒绝（恢复互斥等），**可恢复**（交恢复分支重试），
  不再归为致命失败。

## 黑板键清单（blackboard_protocol.hpp）

**每棵树的黑板都是全新的**：下面是尝试开始时写入的必需键（建树后由
`validateRequiredBlackboardEntries` 校验）。

| 键 | 类型 | 写者 |
|---|---|---|
| `goal` | PoseStamped | pilot（applyPendingGoal） |
| `raw_path` / `smoothed_path` | Path | ComputePathToPose / SmoothPath |
| `raw_path_revision` | uint64 | pilot（每轮置 0）+ ComputePathToPose（成功后 +1） |
| `planner_frequency` / `smoother_frequency` / `controller_frequency` | double | pilot（configure 期） |
| `session_id` / `goal_revision` | uint64 | pilot |
| `failure_context` | FailureContext | 失败节点写 / Recovery 节点消费清零 |
| `navigation_runtime` | NavigationRuntime::Ptr | pilot（注入） |
| `clear_costmap_id` / `spin_id` / `wait_id` / `backup_id` | string | BT 扩展（role→模块映射） |

**`raw_path_revision` 与实时 plan 版本的区别（刻意区分）**：

- **黑板键 `raw_path_revision`**：生产者作用域的"新的原始路径落地了"。只有
  planner 推进它，smoother 分支的 RateController 把 reset_token 绑在它上面——
  于是平滑自己推进的版本号不可能反过来触发自己（这正是当年需要"内容签名跳过"
  的原因，该 hack 已删除）。
- **`NavigationSessionState::currentPlanVersion()`（实时版本）**：单调计数，
  规划 +1、平滑再 +1，驱动控制器的 in-place 路径更新与陈旧结果拒绝。它是这个
  概念的唯一权威，**黑板不再镜像它**（同名但语义不同的第二份拷贝正是当年
  "两个 plan_version"混淆的来源）。

## 与 nav2 的差异提示

- nav2（BT.CPP v3）依赖强 halt 语义；**迁移到 v4 后必须补"树生命终端重置"
  + 建树期接口校验**，否则单发缓存/游标跨生命残留。
- nav2 的 GoalUpdated 短路恢复、RoundRobin 轮换语义已保留（节点定名
  RecoverySequence；nav2 旧版"每轮从第一个恢复重试"的顺序语义未采用）；
  nav2 的 `ClearEntireCostmap` 服务调用在 ayanami 中变为
  `Recovery` 节点 → 命令端口（进程内，无 service）。
- nav2 selector 节点靠 topic 热切换；ayanami 以黑板键写入等价实现。

## 控制/装饰节点契约

- `ResettableNode` / `consumeResetToken`：令牌（{goal_revision}、
  {raw_path_revision}）变化检测的共用 mixin，服务**同一次尝试内的**目标微调。
  缓存不参与任何"树生命复位"——尝试之间根本没有复位这回事。
- `RecoveryNode::budget_reset_token`：令牌变化时重置恢复预算，**不**打断在飞
  恢复——goal 更新永不消耗恢复预算。
- 恢复角色映射（role→模块 id）属于 **BT 扩展**：
  `BehaviorTreeExtension::requiredBlackboardEntries/provideBlackboardEntries`
  声明并写入角色 id 板项；核心 pilot 与黑板协议不感知具体恢复行为，BT 只做编排。
  角色若无人声明则写空字符串——**建树期校验会拒绝**（见上）。
- 会话级导航终态：`NavigationCallbacks::on_navigation_result`
  （SUCCESS/FAILURE/CANCELED + FailureContext，每次尝试恰一次）。
