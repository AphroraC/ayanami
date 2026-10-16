# 单体 FSM（FsmDriver）

ayanami 的导航编排已从行为树整体迁移为**单体有限状态机**：`FsmDriver`
（`include/ayanami/abstract/components/navigation_fsm.hpp`）。BehaviorTree.CPP、
`behavior_tree/` 目录、`navigation_behavior_tree` 扩展模块与整个 Action 层已删除，
BT 的全部导航语义被平移为**普通代码**——没有树、没有黑板、没有端口绑定，
策略即状态机的转移条件。

FsmDriver 自身**不持地图、不加载模块**：宿主（`AbstractNavigation` 模块 so）
向它注入三族 Execution（planner/smoother/controller，已与模块装配完毕）、
有序 recovery 列表和 path validator（`setPathValidator`，通常接 planner 模块的
`isPathValid`——planner 是地图读者）。

## 状态机

```
                     start()
        ┌──────────► kWaitGoal ◄────────────────────────┐
        │               │ beginNavigation(goal)          │
        │               ▼                                │ finishAttempt
        │             kPlan ──plan 失败──► kRecovering    │ (恰一次
        │               │ plan+smooth 成功               │  on_navigation_result)
        │               ▼                     │恢复失败   │
        │             kFollow ◄─恢复成功─重规划─┘          │
        │               │ controller 到达                 │
        │               │                                │
        │   kPlan ──cancel──┐  ┌──cancel── kRecovering    │
        │   kFollow ────────┼──┘                          │
        │               ▼   ▼                             │
        │          kSucceeded / kCanceled / kAborted ─────┘
        │           （驻留终态，直到下一个 goal 消费）
        └── stop()
```

状态全集：`kInit / kWaitGoal / kPlan / kFollow / kRecovering / kSucceeded /
kAborted / kCanceled`（`FsmDriver::State`）。

## 每状态职责

| 状态 | 职责 |
|---|---|
| kInit | 构造后初始态；`start()` 置 kWaitGoal 并拉起两条常驻线程 |
| kWaitGoal | 空闲：plan 线程在 goal 邮箱条件变量上等待；不触碰任何 Execution |
| kPlan | 一拍 = `tickPlan`：plan → `isPathValid` → smooth（失败降级 DegradeToRawPath）→ `isPathValid` → `kickFollow` 进 kFollow。失败进恢复/终态判定 |
| kFollow | 一拍 = `tickFollow`：终态检查（等 control 线程握手）→ 振荡门 → 监督分支（都经 controller `updatePlan` 原地落地）——节流重规划（`planner_frequency`≤0 关闭，plan-once 契约）/ **失效宽限**（`replan_grace` 窗内不调 planner：强制本拍重平滑，由 smoother 的 ESDF 推离修复路径；持续失效超窗才 planner 重规划）/ 迭代重平滑。controller 到达 → kSucceeded；失败 → 恢复/终态判定 |
| kRecovering | 一拍 = `tickRecovering`：轮转游标取下一个 recovery（出厂序 = nav2 默认树：cleanup→spin→wait→back_up），`preemptController` 后**内联** `runBlocking` 执行（进入时记录机器人位置；运行期间挂 goal 邮箱中断谓词——新 goal 到达 → INTERRUPTED 抢占：消费原地修订、回 kPlan，对应 nav2 ReactiveFallback(GoalUpdated) 短路）；成功 → 重试计数 +1、清失败上下文、强制重规划回 kPlan；失败 → kAborted。**止损守卫**：follow 失败进入恢复前核对自上次恢复入口以来的净位移——位移 <5cm（恢复周期内机器人没动）计一次 stuck，连续 2 次 → 直接 kAborted 并给出诊断（占位 recovery 无法改变结局时不再烧满预算） |
| kSucceeded / kCanceled / kAborted | 尝试终态：结果已发射，**驻留**（不空转、不重新规划），下一个 goal 从任何非活跃态直接开新尝试 |

## 线程模型

| 线程 | 数量 | 职责 |
|---|---|---|
| PLAN 线程 | 1 | FSM tick 循环（`fsm_frequency` 默认 20Hz）：每拍先消费 goal 邮箱，再按状态执行 tick；planner/smoother/recovery 的 `runBlocking` **内联**发生在此线程 |
| CONTROL 线程 | 1 | 在 follow 握手（条件变量）上等待 `kickFollow`，随后 `controller_->runBlocking()` **阻塞到终态**（含 cancel 落地的 CANCELED 终态），写回到达标志 |
| （宿主侧） | — | costmap 唯一写者线程、ESDF 写者线程、ayanami_ros loop 均在 FsmDriver 之外（见 [architecture.md](architecture.md)） |

- **goal 邮箱（latest-wins）**：`beginNavigation` 只覆写 `pending_goal_` 并
  `notify_all`；消费永远发生在 PLAN 线程的 tick 边界——空闲则 `startAttempt`，
  尝试中则是**原地修订**（`goal_revision` +1、重试预算复位、`force_replan_`，
  controller 不被打断）。
- 停机 `stop()`：置位 running=false → 对全部 Execution `stop()`（解除内联
  `runBlocking`，execution 以 STOPPED 退出）→ join 两线程。幂等，析构兜底。
- 取消 `cancelNavigation()`：置位 + 对在飞 Execution 非阻塞 `cancel()`
  （controller 的 cancel 内同步发恰一次零脉冲），下一拍边界结算 CANCELED。

## 从 BT 平移的五项语义（对照表）

| BT 构造（已删除） | FSM 代码 |
|---|---|
| `RecoveryNode`（number_of_retries）+ `budget_reset_token` | `max_nav_retries` 重试预算 + goal 修订复位（`consumeGoalMailbox` 里 `retry_count_ = 0`；goal 更新永不消耗预算） |
| 不可恢复失败短路恢复分支 | `isRecoverable()`（`messages/failure.hpp`：INVALID_INPUT/START_INVALID/GOAL_INVALID/TRANSFORM_UNAVAILABLE/CANCELED 直接 ABORT；RESOURCE_BUSY 属瞬时态**可恢复**）→ `enterRecoveryOrTerminal` |
| `RecoverySequence`（RoundRobin 轮换） | `recovery_cursor_` 对注入的有序 recovery 列表轮转（跨成功保持前进） |
| `GoalUpdated` + 原地修订 | goal_revision 计数 + 邮箱消费即 `force_replan_`；kFollow 中经 `controller_->updatePlan` 原地换路径——无 halt、无零脉冲，controller 持续跟随旧路径直到新路径落地 |
| `RateController`（planner/smoother 分支） | `planner_frequency` / `smoother_frequency` 节流（steady_clock 计时；planner 另有"路径失效即刻重规划"旁路） |
| `IsRobotProgressing` 振荡检测 | 位置 deque 窗口：最近 `oscillation_timeout` 秒内位移 ≤ `oscillation_distance` → NO_PROGRESS 失败交恢复 |
| 树终态 + on_navigation_result | `finishAttempt` 每次尝试**恰一次** `on_navigation_result`（SUCCESS/CANCELED/ABORTED + FailureContext） |

其余平移：

- **重规划内联在 kFollow**：BT 时代是 PipelineSequence 里 RateController 重跑
  ComputePathToPose/SmoothPath；现在 `tickFollow` 直接调 `runPlanOnce`
  （plan → 校验 → smooth → 校验 → plan_version +1），成功后
  `controller_->updatePlan` 原地下发。
- **迭代重平滑**：输入是**当前平滑路径**（收敛式精修）；只有新 raw path
  （`runPlanOnce`）才让平滑从 raw 重启。被拒绝的候选不替换当前路径，
  平滑失败只告警（增强性一步的失败不是导航失败）。
- **DegradeToRawPath**：kPlan 一拍内平滑失败/平滑产物校验失败时回退 raw path
  （raw 也不合法才真正失败）——平滑是增强，不是门槛。
- **每次尝试干净开局**：`startAttempt` 复位预算/游标/版本/位置历史并
  `controller_->resetCheckers()`（Execution 复用模型下防止上一尝试的
  checker 基线泄漏）。

## FsmDriverConfig 参数表

结构体字段默认值（`navigation_fsm.hpp`）；全部为 configure 期常量，出厂值在
路由 json 的 `navigation` 段（键名见 [configuration.md](configuration.md)）：

| 字段 | 代码默认 | 说明 |
|---|---|---|
| `fsm_frequency` | 20.0 | PLAN 线程 tick 频率（Hz） |
| `planner_frequency` | -1.0 | kFollow 期间节流重规划（Hz；≤0 = 关闭，plan-once 契约） |
| `replan_grace` | 2.0 | 路径失效宽限窗（秒）：窗内 smoother 独占修复（每拍强制重平滑）；超窗 planner 重规划。0 = 失效立即重规划（旧行为） |
| `smoother_frequency` | 20.0 | kFollow 期间迭代重平滑节流（Hz；≤0 = 每条 raw path 只平滑一次） |
| `max_smooth_duration` | 2.0 | 单次平滑的建议时间预算（s；路由键 `smoother_timeout`） |
| `max_nav_retries` | 6 | 每次尝试的恢复预算（goal 修订复位） |
| `oscillation_distance` | 0.1 | 振荡门位移阈值（m） |
| `oscillation_timeout` | 5.0 | 振荡门时间窗（s；两项任一 ≤0 关闭振荡门） |

## 与宿主的注入关系

```
宿主模块 so（routes/*.json::navigation.so_name 选择，实现 AbstractNavigation）
initialize()
 ├─ bootstrap()（config root + logging）
 ├─ 地图机器装配（CostmapNavigationServer 成员 / EsdfWriter 派生场）
 ├─ NavigationServerBase::loadModules()
 │    ├─ 6 族 dlopen（planner/smoother/controller/recovery + 两个 checker 扩展族）
 │    └─ 虚钩子 initialize*Module：宿主注入地图族句柄
 │         （rose: planner←costmap reader、smoother←esdf 唯一读者、
 │           controller/recovery 无地图；costmap: 全 costmap 族）
 └─ createActions()（loadModules 尾钩子，Action 层已删、名存实用）
      ├─ 每族一个 Execution（planner/smoother/controller + recovery 列表）
      ├─ configureControllerExecution（checker 注入/速度上限/patience/velocity_timeout）
      ├─ 读 navigation 段组装 FsmDriverConfig → new FsmDriver(...)
      └─ fsm_->setPathValidator(planner->isPathValid)
start()：地图写者线程 → 模块 activate → fsm_->start()
setGoal(pose) → fsm_->beginNavigation(pose)（latest-wins 邮箱，非阻塞）
```

宿主实现有两份：`modules/rose_navigation_fsm`（costmap + 派生 ESDF 场 +
混合族模块）与 `modules/costmap_navigation_fsm`（纯 costmap 栈 +
costmap 族模块）。两者对 FsmDriver 完全同构——换宿主 = 换路由 json 的
`so_name`。

## 观测接口（测试/遥测）

`state()` / `attemptActive()` / `goalRevision()` / `planVersion()` /
`config()` 均为原子读；`tests/fsm_test.cpp` 用进程内 mock Execution 直驱
状态机（不 dlopen），`tests/fsm_e2e_test.cpp` 与 `fsm_costmap_e2e_test.cpp`
则按生产方式 dlopen 宿主 so 跑无头 E2E。
